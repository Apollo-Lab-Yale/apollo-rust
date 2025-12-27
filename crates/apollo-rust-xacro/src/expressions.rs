use crate::context::XacroContext;
use evalexpr::{
    Context, ContextWithMutableFunctions, ContextWithMutableVariables, Function, HashMapContext,
    Value, eval_with_context,
};
use regex::Regex;
use std::collections::HashSet;

pub struct ExpressionEvaluator;

impl ExpressionEvaluator {
    pub fn evaluate(expression: &str, context: &XacroContext) -> Result<String, String> {
        // 1. Handle $(arg name)
        let re_arg = Regex::new(r"\$\(arg\s+([a-zA-Z0-9_.]+)\)").unwrap();
        let mut error = None;
        let content_with_args = re_arg.replace_all(expression, |caps: &regex::Captures| {
            let arg_name = &caps[1];
            match context.get_argument(arg_name) {
                Some(val) => val.clone(),
                None => {
                    error = Some(format!("Argument '{}' not found", arg_name));
                    "ERROR".to_string()
                }
            }
        });

        if let Some(e) = error {
            return Err(e);
        }

        // 2. Handle ${...}
        let re = Regex::new(r"\$\{([^}]+)\}").unwrap();
        let mut error = None;

        let new_content = re.replace_all(&content_with_args, |caps: &regex::Captures| {
            let mut inner_expr = caps[1].to_string();

            // Transform ['key'] and ["key"] to ___key for internal evaluation
            let re_idx_lit = Regex::new(r#"\[['"]([^'"]+)['"]\]"#).unwrap();
            while re_idx_lit.is_match(&inner_expr) {
                inner_expr = re_idx_lit.replace_all(&inner_expr, "___$1").to_string();
            }

            // Transform [identifier] to ___identifier_value
            let re_idx_var = Regex::new(r"\[([a-zA-Z0-9_.]+)\]").unwrap();
            while let Some(caps) = re_idx_var.captures(&inner_expr) {
                let var_name = &caps[1];
                let var_val = match eval_with_custom_context(var_name, context) {
                    Ok(v) => v,
                    Err(_) => "ERROR_RESOLVING_INDEX".to_string(),
                };
                let range = caps.get(0).unwrap().range();
                inner_expr.replace_range(range, &format!("___{}", var_val));
            }

            match eval_with_custom_context(&inner_expr, context) {
                Ok(val) => val,
                Err(e) => {
                    error = Some(e);
                    "ERROR".to_string()
                }
            }
        });

        if let Some(e) = error {
            return Err(e);
        }

        Ok(new_content.to_string())
    }

    pub fn evaluate_boolean(expression: &str, context: &XacroContext) -> Result<bool, String> {
        let val_str = Self::evaluate(expression, context)?;

        match val_str.trim().to_lowercase().as_str() {
            "true" | "1" | "1.0" => Ok(true),
            "false" | "0" | "0.0" => Ok(false),
            other => {
                if let Ok(n) = other.parse::<f64>() {
                    Ok(n != 0.0)
                } else {
                    Err(format!("Cannot evaluate '{}' as boolean", val_str))
                }
            }
        }
    }
}

fn escape_dots(expr: &str) -> String {
    let re = Regex::new(r"([a-zA-Z_][a-zA-Z0-9_]*)\.([a-zA-Z_][a-zA-Z0-9_]*)").unwrap();
    let mut current = expr.to_string();
    while re.is_match(&current) {
        let next = re.replace_all(&current, "${1}___${2}").to_string();
        if next == current {
            break;
        }
        current = next;
    }
    current
}

fn eval_with_custom_context(expr: &str, xacro_context: &XacroContext) -> Result<String, String> {
    let mut context = HashMapContext::new();
    // Inject constants
    context
        .set_value("pi".into(), Value::Float(std::f64::consts::PI))
        .unwrap();
    context
        .set_value("PI".into(), Value::Float(std::f64::consts::PI))
        .unwrap();
    context
        .set_value("tau".into(), Value::Float(std::f64::consts::TAU))
        .unwrap();

    let props = xacro_context.get_all_properties();

    // Map properties with escaped dots
    for (k, v) in &props {
        let escaped_key = k.replace(".", "___");
        if let Ok(f) = v.parse::<f64>() {
            context.set_value(escaped_key, Value::Float(f)).unwrap();
        } else {
            context
                .set_value(escaped_key, Value::String(v.clone()))
                .unwrap();
        }
    }

    // Map prefixes with escaped dots
    let mut prefixes = HashSet::new();
    for k in props.keys() {
        let parts: Vec<&str> = k.split('.').collect();
        for i in 1..parts.len() {
            let prefix = parts[..i].join(".");
            prefixes.insert(prefix);
        }
    }
    for prefix in prefixes {
        let escaped_prefix = prefix.replace(".", "___");
        if context.get_value(&escaped_prefix).is_none() {
            context
                .set_value(escaped_prefix.clone(), Value::String(prefix.clone()))
                .unwrap();
        }
    }

    // Add xacro.load_yaml function (pre-escaped)
    context
        .set_function(
            "xacro___load_yaml".to_string(), // escaped
            Function::new(move |argument| {
                let filename = argument.as_string().map_err(|e| {
                    evalexpr::EvalexprError::CustomMessage(format!(
                        "Expected string filename: {}",
                        e
                    ))
                })?;
                Ok(Value::String(format!("RESERVED_LOAD_YAML:{}", filename)))
            }),
        )
        .unwrap();

    // Add exists function
    // The keys in the escaped_keys set should also represent the "schema"
    let mut escaped_keys = HashSet::new();
    for k in props.keys() {
        escaped_keys.insert(k.replace(".", "___"));
    }
    let mut escaped_prefixes = HashSet::new();
    for p in props.keys() {
        let parts: Vec<&str> = p.split('.').collect();
        for i in 1..parts.len() {
            let prefix = parts[..i].join(".");
            escaped_prefixes.insert(prefix.replace(".", "___"));
        }
    }

    // Register exists function
    // Actually HashMapContext might be deep.
    // We can just share the map logic or capture check.
    context
        .set_function(
            "exists".into(),
            Function::new(move |argument| {
                if let Ok(val) = argument.as_string() {
                    // Check if this key exists in the "keys/prefixes" sets we built?
                    // Or simpler: does it exist in the context?
                    // The context keys are escaped. So we need to escape the argument to check.
                    let escaped_arg = val.replace(".", "___");
                    // We can't easily check 'cloned_context' here because of borrowing/moving.
                    // But we can check keys/prefixes sets if we move them in.
                    // For now, let's just assume if it's in the context, it exists.
                    // But wait, the context has values for keys.
                    // A simpler way: 'exists' takes a string "path.to.prop".
                    // That prop corresponds to "path___to___prop" in context.
                    // If we find that key, true.
                    // But we don't have access to context inside this closure easily without RefCell/Arc.
                    // Let's use the sets.
                    Ok(Value::Boolean(
                        escaped_keys.contains(&escaped_arg)
                            || escaped_prefixes.contains(&escaped_arg),
                    ))
                } else {
                    Ok(Value::Boolean(false))
                }
            }),
        )
        .unwrap();

    // 1. Transform 'key' in map and variable in map to exists(map + "." + key)
    let re_in_quoted = Regex::new(r#"(['"])([a-zA-Z0-9_.]+)['"]\s+in\s+([a-zA-Z0-9_.]+)"#).unwrap();
    let mut expr_transformed = re_in_quoted
        .replace_all(expr, |caps: &regex::Captures| {
            format!("exists({} + \".{}\")", &caps[3], &caps[2])
        })
        .to_string();

    let re_in_var =
        Regex::new(r#"([a-zA-Z_][a-zA-Z0-9_]*)\s+in\s+([a-zA-Z_][a-zA-Z0-9_.]*)"#).unwrap();
    expr_transformed = re_in_var
        .replace_all(&expr_transformed, |caps: &regex::Captures| {
            format!("exists({} + \".\" + {})", &caps[2], &caps[1])
        })
        .to_string();

    // 1.5 Normalize single quotes to double quotes for evalexpr
    // Only safely replace '...' with "..."
    // Note: This matches standard single quoted strings.
    let re_single_quote = Regex::new(r"'([^']*)'").unwrap();
    expr_transformed = re_single_quote
        .replace_all(&expr_transformed, "\"$1\"")
        .to_string();

    // 2. Escape dots in identifiers in the expression
    let expr_final = escape_dots(&expr_transformed);

    match eval_with_context(&expr_final, &context) {
        Ok(v) => match v {
            Value::String(s) => Ok(s),
            Value::Float(f) => Ok(f.to_string()),
            Value::Int(i) => Ok(i.to_string()),
            Value::Boolean(b) => Ok(b.to_string()),
            Value::Tuple(_) => Err("Tuples not supported in xacro output".into()),
            Value::Empty => Ok("".to_string()),
        },
        Err(e) => Err(format!(
            "Eval error for '{}' (originally '{}'): {}",
            expr_final, expr, e
        )),
    }
}
