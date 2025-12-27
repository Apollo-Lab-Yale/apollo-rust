use crate::XacroError;
use crate::context::XacroContext;
use crate::expressions::ExpressionEvaluator;
use apollo_rust_file::ApolloPathBufTrait;
use roxmltree::Document;
use serde_yaml::Value as YamlValue;
// use serde_yaml::value::TaggedValue; // Unused
use colored::*;
use std::f64::consts::PI;
use std::path::PathBuf;

pub struct XacroParser;

impl XacroParser {
    pub fn parse<P: ApolloPathBufTrait + Clone>(
        xml_content: &str,
        base_path: &P,
        search_dirs: &[P],
        arguments: Option<&std::collections::HashMap<String, String>>,
    ) -> Result<String, XacroError> {
        let mut context = XacroContext::new();
        context.set_search_dirs(
            search_dirs
                .iter()
                .map(|d| d.to_path_buf().to_string_lossy().to_string())
                .collect(),
        );
        if let Some(args) = arguments {
            for (k, v) in args {
                context.set_argument(k.clone(), v.clone());
            }
        }
        Self::parse_with_context(xml_content, base_path, search_dirs, &mut context)
    }

    pub(crate) fn parse_with_context<P: ApolloPathBufTrait + Clone>(
        xml_content: &str,
        base_path: &P,
        search_dirs: &[P],
        context: &mut XacroContext,
    ) -> Result<String, XacroError> {
        context.set_search_dirs(
            search_dirs
                .iter()
                .map(|d| d.to_path_buf().to_string_lossy().to_string())
                .collect(),
        );
        let doc = Document::parse(xml_content)?;
        let root = doc.root_element();
        let raw_output = process_node(root, context, base_path, search_dirs)?;

        // Post-processing to remove blank lines (xacro artifacts)
        let re = regex::Regex::new(r"(?m)^\s*\r?\n")
            .map_err(|_| XacroError::Custom("Regex error".into()))?;
        let cleaned_output = re.replace_all(&raw_output, "").to_string();

        Ok(cleaned_output)
    }
}

fn resolve_path<P: ApolloPathBufTrait + Clone>(
    raw_path: &str,
    base_path: &P,
    search_dirs: &[P],
) -> Result<P, XacroError> {
    // 1. Handle $(find pkg)
    let re = regex::Regex::new(r"\$\(find\s+([a-zA-Z0-9_]+)\)")
        .map_err(|_| XacroError::Custom("Regex error".into()))?;
    let path_str = re.replace_all(raw_path, |caps: &regex::Captures| {
        let pkg_name = &caps[1];
        if let Some(pkg_path) = resolve_package_path(pkg_name, search_dirs) {
            pkg_path.to_path_buf().to_string_lossy().to_string()
        } else {
            eprintln!("Package NOT FOUND: {}", pkg_name);
            format!("pkg_not_found_{}", pkg_name)
        }
    });

    // 2. Handle package://
    let path_str_result = if path_str.starts_with("package://") {
        let suffix = &path_str["package://".len()..];
        let parts: Vec<&str> = suffix.splitn(2, '/').collect();
        if parts.len() < 2 {
            return Err(XacroError::Custom(format!(
                "Invalid package url: {}",
                path_str
            )));
        }
        let pkg_name = parts[0];
        let relative_path = parts[1];

        if let Some(pkg_path) = resolve_package_path(pkg_name, search_dirs) {
            pkg_path
                .append(relative_path)
                .to_path_buf()
                .to_string_lossy()
                .to_string()
        } else {
            return Err(XacroError::Custom(format!(
                "Package not found: {}",
                pkg_name
            )));
        }
    } else {
        path_str.to_string()
    };

    // 3. Relative path handling
    let path_buf = PathBuf::from(&path_str_result);
    if path_buf.is_absolute() {
        if !path_buf.exists() {
            eprintln!("Absolute path NOT FOUND: {:?}", path_buf);
        }
        Ok(P::new_from_path(&path_buf))
    } else {
        let final_path = base_path.clone().append(&path_str_result);
        if !final_path.path_exists() {
            eprintln!(
                "Relative path NOT FOUND: {:?} (base: {:?})",
                path_str_result, base_path
            );
        }
        Ok(final_path)
    }
}

fn process_node<P: ApolloPathBufTrait + Clone>(
    node: roxmltree::Node,
    context: &mut XacroContext,
    base_path: &P,
    search_dirs: &[P],
) -> Result<String, XacroError> {
    if node.is_comment() {
        let full_doc = node.document().input_text();
        let range = node.range();
        return Ok(full_doc[range.start..range.end].to_string());
    }

    if node.is_text() {
        let text = node.text().unwrap_or("");
        return ExpressionEvaluator::evaluate(text, context).map_err(XacroError::Custom);
    }

    if !node.is_element() {
        return Ok("".to_string());
    }

    let tag_name = node.tag_name().name();
    let is_xacro = is_xacro_element(&node);

    // Simplification: match on local name if it is xacro
    if is_xacro {
        // Extract local name involves checking if it has prefix or not. roxmltree name() is local name usually?
        // Let's assume name() is local name if valid namespace, or full name if not?
        // Let's print to be sure.
        let local_name = if tag_name.starts_with("xacro:") {
            &tag_name[6..]
        } else {
            tag_name
        };

        match local_name {
            "include" => {
                let filename = node
                    .attribute("filename")
                    .ok_or(XacroError::Custom("Include tag missing filename".into()))?;

                // Eval filename for expressions inside it?
                let eval_filename =
                    ExpressionEvaluator::evaluate(filename, context).map_err(XacroError::Custom)?;

                let resolved_path = resolve_path(&eval_filename, base_path, search_dirs)?;

                let included_content = resolved_path
                    .read_file_contents_to_string_result()
                    .map_err(|e| {
                        XacroError::IoError(std::io::Error::new(std::io::ErrorKind::NotFound, e))
                    })?;

                let included_base =
                    P::new_from_path(&resolved_path.to_path_buf().parent().unwrap().to_path_buf());

                let processed_include = XacroParser::parse_with_context(
                    &included_content,
                    &included_base,
                    search_dirs,
                    context,
                )?;
                Ok(processed_include)
            }
            "property" => {
                let name = node
                    .attribute("name")
                    .ok_or(XacroError::Custom("Property missing name".into()))?;
                let scope = node.attribute("scope");
                let val = node.attribute("value");

                if let Some(v) = val {
                    let evaluated =
                        ExpressionEvaluator::evaluate(v, context).map_err(XacroError::Custom)?;

                    if evaluated.starts_with("RESERVED_LOAD_YAML:") {
                        let filename = &evaluated["RESERVED_LOAD_YAML:".len()..];
                        let resolved_path = resolve_path(filename, base_path, search_dirs)?;
                        let yaml_content = resolved_path
                            .read_file_contents_to_string_result()
                            .map_err(|e| {
                                XacroError::IoError(std::io::Error::new(
                                    std::io::ErrorKind::NotFound,
                                    e,
                                ))
                            })?;
                        let yaml_val: crate::parser::YamlValue =
                            serde_yaml::from_str(&yaml_content).map_err(|e| {
                                XacroError::Custom(format!("YAML parse error: {}", e))
                            })?;
                        inject_yaml_to_context(&yaml_val, name, context);
                    } else {
                        // Check if evaluated refers to an existing prefix/branch
                        let all_props = context.get_all_properties();
                        let prefix = format!("{}.", evaluated);
                        for (key, val) in &all_props {
                            if key.starts_with(&prefix) {
                                let suffix = &key[prefix.len()..];
                                let new_key = format!("{}.{}", name, suffix);
                                match scope {
                                    Some("parent") => {
                                        context.set_property_parent(new_key, val.clone())
                                    }
                                    Some("global") => {
                                        context.set_property_global(new_key, val.clone())
                                    }
                                    _ => context.set_property(new_key, val.clone()),
                                }
                            }
                        }

                        // If it's just a value, or even if we found a branch, set the base property
                        match scope {
                            Some("parent") => {
                                context.set_property_parent(name.to_string(), evaluated)
                            }
                            Some("global") => {
                                context.set_property_global(name.to_string(), evaluated)
                            }
                            _ => context.set_property(name.to_string(), evaluated),
                        }
                    }
                } else if let Some(_child_el) = node.first_element_child() {
                    // Handle property defined by child element (e.g., <xacro:property name="foo"><bar/></xacro:property>)
                    // This is a simplified approach, usually xacro properties are simple values or blocks.
                    // For now, we'll just take the inner text if it's a text node, or serialize the first child element.
                    // A more robust solution would involve capturing the raw XML of the children.
                    let mut content = String::new();
                    for child in node.children() {
                        content.push_str(&process_node(child, context, base_path, search_dirs)?);
                    }
                    let evaluated = ExpressionEvaluator::evaluate(&content, context)
                        .map_err(XacroError::Custom)?;
                    match scope {
                        Some("parent") => context.set_property_parent(name.to_string(), evaluated),
                        Some("global") => context.set_property_global(name.to_string(), evaluated),
                        _ => context.set_property(name.to_string(), evaluated),
                    }
                }
                Ok("".to_string())
            }
            "macro" => {
                // Define macro stub
                let name = node
                    .attribute("name")
                    .ok_or(XacroError::Custom("Macro missing name".into()))?;
                let params_str = node.attribute("params").unwrap_or("");
                let params: Vec<(String, String)> = params_str
                    .split_whitespace()
                    .map(|s| {
                        if let Some((name, default)) = s.split_once(":=") {
                            (name.to_string(), default.to_string())
                        } else {
                            (s.to_string(), "".to_string())
                        }
                    })
                    .collect();

                // TODO: Properly capture macro content.
                // For now, we stub it to allow parsing property blocks.
                // We actually need to capture the inner XML string of the macro to re-parse it upon call.
                // roxmltree doesn't expose "inner_xml". We might have to reconstruct it or use range.
                // node.range() gives byte range in original document.
                // internal logic:
                // let range = node.range(); // This includes the tag itself.
                // We want children range.
                // But we don't have easy access to children range directly as a single block without iterating.
                // Actually, if we have the original `xml_content` string (which we passed to parse), we could slice it.
                // But `process_node` doesn't have `xml_content`. It has `node` which has `document()`.
                // `node.document().input_text()` gives us the full text!

                // Let's get the inner content range.
                // It's from end of opening tag to start of closing tag.
                // roxmltree doesn't easily give "end of opening tag".
                // Workaround: Reconstruct XML from children by calling node.children() and re-serializing?
                // This might lose comments or formatting but is functionally correct.
                // Wait, `process_node` does re-serialization for normal tags.
                // We can use similar logic but WITHOUT evaluating expressions yet.
                // We want RAW content.

                // Better approach: Use `node.range()` to get the whole tag, and then try to strip the outer tag.
                // Or just iterate children and print them raw.

                let mut macro_content = String::new();
                for child in node.children() {
                    // We need the raw text of the child.
                    // roxmltree range is helpful here.
                    let child_range = child.range();
                    let full_doc = node.document().input_text();
                    macro_content.push_str(&full_doc[child_range.start..child_range.end]);
                }

                use crate::context::MacroDefinition;
                let def = MacroDefinition {
                    name: name.to_string(),
                    params,
                    content: macro_content,
                };
                context.register_macro(def);

                Ok("".to_string())
            }
            "if" => {
                let value = node
                    .attribute("value")
                    .ok_or(XacroError::Custom("If tag missing value".into()))?;

                let should_run = ExpressionEvaluator::evaluate_boolean(value, context)
                    .map_err(XacroError::Custom)?;

                if should_run {
                    let mut output = String::new();
                    for child in node.children() {
                        if child.is_element() {
                            output.push_str(&process_node(child, context, base_path, search_dirs)?);
                        } else if child.is_text() {
                            let text = child.text().unwrap_or("");
                            let val = ExpressionEvaluator::evaluate(text, context)
                                .map_err(XacroError::Custom)?;
                            output.push_str(&val);
                        }
                    }
                    Ok(output)
                } else {
                    Ok("".to_string())
                }
            }
            "unless" => {
                let value = node
                    .attribute("value")
                    .ok_or(XacroError::Custom("Unless tag missing value".into()))?;

                let should_skip = ExpressionEvaluator::evaluate_boolean(value, context)
                    .map_err(XacroError::Custom)?;

                if !should_skip {
                    let mut output = String::new();
                    for child in node.children() {
                        if child.is_element() {
                            output.push_str(&process_node(child, context, base_path, search_dirs)?);
                        } else if child.is_text() {
                            let text = child.text().unwrap_or("");
                            let val = ExpressionEvaluator::evaluate(text, context)
                                .map_err(XacroError::Custom)?;
                            output.push_str(&val);
                        }
                    }
                    Ok(output)
                } else {
                    Ok("".to_string())
                }
            }
            "arg" => {
                let name = node
                    .attribute("name")
                    .ok_or(XacroError::Custom("Arg missing name".into()))?;
                let default = node.attribute("default");

                // Only set if not already present (global args override defaults)
                if context.get_argument(name).is_none() {
                    if let Some(d) = default {
                        let evaluated = ExpressionEvaluator::evaluate(d, context)
                            .map_err(XacroError::Custom)?;
                        context.set_argument(name.to_string(), evaluated);
                    }
                }
                Ok("".to_string())
            }
            "load_yaml" => {
                let filename = node
                    .attribute("filename")
                    .ok_or(XacroError::Custom("load_yaml missing filename".into()))?;
                let var_name = node
                    .attribute("value")
                    .or(node.attribute("name")) // Xacro uses 'value' or 'name' sometimes? Standard is 'value'.
                    .ok_or(XacroError::Custom("load_yaml missing value/name".into()))?;

                let eval_filename =
                    ExpressionEvaluator::evaluate(filename, context).map_err(XacroError::Custom)?;
                let resolved_path = resolve_path(&eval_filename, base_path, search_dirs)?;

                let yaml_content = resolved_path
                    .read_file_contents_to_string_result()
                    .map_err(|e| {
                        XacroError::IoError(std::io::Error::new(std::io::ErrorKind::NotFound, e))
                    })?;

                let yaml_val: YamlValue = serde_yaml::from_str(&yaml_content)
                    .map_err(|e| XacroError::Custom(format!("YAML parse error: {}", e)))?;

                inject_yaml_to_context(&yaml_val, var_name, context);

                Ok("".to_string())
            }
            "element" => {
                let name_attr = node
                    .attribute("name")
                    .ok_or(XacroError::Custom("element missing name".into()))?;
                let name = ExpressionEvaluator::evaluate(name_attr, context)
                    .map_err(XacroError::Custom)?;

                let mut output = format!("<{}", name);
                // Child attributes and nodes are processed normally
                // Wait, xacro:element can have xacro:attribute children
                // We need to process children and catch attributes.

                let mut inner_content = String::new();
                for child in node.children() {
                    let res = process_node(child, context, base_path, search_dirs)?;
                    if res.trim().starts_with("ATTR:") {
                        output.push_str(&res["ATTR:".len()..]);
                    } else {
                        inner_content.push_str(&res);
                    }
                }

                if inner_content.is_empty() {
                    output.push_str("/>");
                } else {
                    output.push('>');
                    output.push_str(&inner_content);
                    output.push_str(&format!("</{}>", name));
                }
                Ok(output)
            }
            "attribute" => {
                let name_attr = node
                    .attribute("name")
                    .ok_or(XacroError::Custom("attribute missing name".into()))?;
                let value_attr = node
                    .attribute("value")
                    .ok_or(XacroError::Custom("attribute missing value".into()))?;

                let name = ExpressionEvaluator::evaluate(name_attr, context)
                    .map_err(XacroError::Custom)?;
                let value = ExpressionEvaluator::evaluate(value_attr, context)
                    .map_err(XacroError::Custom)?;

                Ok(format!("ATTR: {}=\"{}\"", name, value))
            }
            "insert_block" => {
                let name_attr = node
                    .attribute("name")
                    .ok_or(XacroError::Custom("insert_block missing name".into()))?;
                let name = ExpressionEvaluator::evaluate(name_attr, context)
                    .map_err(XacroError::Custom)?;

                match context.get_block(&name) {
                    Some(block_content) => {
                        // Re-parse the block content within the current context
                        let wrapped = format!(
                            "<root xmlns:xacro=\"http://www.ros.org/wiki/xacro\">{}</root>",
                            block_content
                        );
                        let doc = Document::parse(&wrapped)?;
                        let root = doc.root_element();
                        let mut output = String::new();
                        for child in root.children() {
                            output.push_str(&process_node(child, context, base_path, search_dirs)?);
                        }
                        Ok(output)
                    }
                    None => Err(XacroError::Custom(format!("Block '{}' not found", name))),
                }
            }
            _ => {
                // Check if macro
                let def_opt = context.get_macro(local_name).cloned();

                if let Some(def) = def_opt {
                    context.with_scope(|context| {
                        for (param_name, default_val) in &def.params {
                            if param_name.starts_with('*') {
                                let block_name = &param_name[1..];
                                // Look for child element with this name
                                let mut captured_block = String::new();
                                for child in node.children() {
                                    if child.is_element() && child.tag_name().name() == block_name {
                                        // Capture children of this child
                                        for sub_child in child.children() {
                                            let child_range = sub_child.range();
                                            let full_doc = node.document().input_text();
                                            captured_block.push_str(
                                                &full_doc[child_range.start..child_range.end],
                                            );
                                        }
                                        break;
                                    }
                                }
                                context.set_block(block_name.to_string(), captured_block);
                            } else if let Some(val) = node.attribute(param_name.as_str()) {
                                let evaluated = ExpressionEvaluator::evaluate(val, context)
                                    .map_err(XacroError::Custom)?;

                                // Branch copy for macro parameters
                                let all_props = context.get_all_properties();
                                let prefix = format!("{}.", evaluated);
                                for (key, val) in &all_props {
                                    if key.starts_with(&prefix) {
                                        let suffix = &key[prefix.len()..];
                                        let new_key = format!("{}.{}", param_name, suffix);
                                        // eprintln!("Macro Branch copy: {} -> {}", key, new_key);
                                        context.set_property(new_key, val.clone());
                                    }
                                }

                                context.set_property(param_name.clone(), evaluated);
                            } else {
                                // Default value logic
                                let (evaluated_val, should_eval) = if default_val.starts_with('^') {
                                    // Syntax: ^ or ^|default
                                    // Check if param_name exists in scope
                                    // Note: context.get_property searches up the scope stack.
                                    // Since we just pushed a new scope and haven't set param_name in it yet,
                                    // this correctly searches the parent scope(s).
                                    if let Some(val) = context.get_property(param_name) {
                                        (val.clone(), false)
                                    } else if let Some((_, fallback)) = default_val.split_once('|')
                                    {
                                        (fallback.to_string(), true)
                                    } else {
                                        // If just ^, acts as required if not found.
                                        if default_val == "^" {
                                            return Err(XacroError::Custom(format!(
                                                "Macro param '{}' default '^' not found in scope",
                                                param_name
                                            )));
                                        }
                                        // Fallthrough (e.g. ^strange) - treat as regular string?
                                        // Or error? treating as string for safety.
                                        (default_val.to_string(), true)
                                    }
                                } else if default_val.is_empty() {
                                    // Treat empty default as required parameter
                                    return Err(XacroError::Custom(format!(
                                        "Macro '{}' missing parameter '{}'",
                                        local_name, param_name
                                    )));
                                } else {
                                    (default_val.to_string(), true)
                                };

                                let final_val = if should_eval {
                                    ExpressionEvaluator::evaluate(&evaluated_val, context)
                                        .map_err(XacroError::Custom)?
                                } else {
                                    evaluated_val
                                };
                                context.set_property(param_name.clone(), final_val);
                            }
                        }

                        let wrapped = format!(
                            "<root xmlns:xacro=\"http://www.ros.org/wiki/xacro\">{}</root>",
                            def.content
                        );
                        let doc = Document::parse(&wrapped)?;
                        let root = doc.root_element();
                        let mut output = String::new();
                        for child in root.children() {
                            if child.is_element() {
                                output.push_str(&process_node(
                                    child,
                                    context,
                                    base_path,
                                    search_dirs,
                                )?);
                            } else if child.is_text() {
                                output.push_str(
                                    &ExpressionEvaluator::evaluate(child.text().unwrap(), context)
                                        .map_err(XacroError::Custom)?,
                                );
                            }
                        }
                        Ok(output)
                    })
                } else {
                    Ok(format!("<!-- unknown macro {} -->", local_name))
                }
            }
        }
    } else {
        // --- PRE-SCAN FOR ARGS ---
        // Many xacro files define args even after they are used in parent attributes (like <robot name="$(arg name)">)
        // or out of order. We do a quick pass of direct children to find these first.
        for child in node.children() {
            if child.is_element() {
                if is_xacro_element(&child) {
                    let tag_name = child.tag_name().name();
                    let actual_tag = if tag_name.starts_with("xacro:") {
                        &tag_name[6..]
                    } else {
                        tag_name
                    };
                    if actual_tag == "arg" {
                        let _ = process_node(child, context, base_path, search_dirs)?;
                    }
                }
            }
        }

        let mut output = String::new();
        output.push('<');
        output.push_str(node.tag_name().name());

        for attr in node.attributes() {
            let val =
                ExpressionEvaluator::evaluate(attr.value(), context).map_err(XacroError::Custom)?;
            output.push_str(&format!(" {}=\"{}\"", attr.name(), val));
        }

        if !node.has_children() {
            output.push_str("/>");
            return Ok(output);
        }
        output.push('>');

        for child in node.children() {
            if child.is_element() {
                if is_xacro_element(&child) {
                    let tag_name = child.tag_name().name();
                    let actual_tag = if tag_name.starts_with("xacro:") {
                        &tag_name[6..]
                    } else {
                        tag_name
                    };
                    if actual_tag == "arg" {
                        // Already processed in pre-scan
                        continue;
                    }
                }
                output.push_str(&process_node(child, context, base_path, search_dirs)?);
            } else if child.is_text() {
                let text = child.text().unwrap_or("");
                let val =
                    ExpressionEvaluator::evaluate(text, context).map_err(XacroError::Custom)?;
                output.push_str(&val);
            }
        }

        output.push_str(&format!("</{}>", node.tag_name().name()));
        Ok(output)
    }
}

fn inject_yaml_to_context(val: &YamlValue, prefix: &str, context: &mut XacroContext) {
    match val {
        YamlValue::Mapping(m) => {
            for (k, v) in m {
                if let Some(k_str) = k.as_str() {
                    let new_prefix = format!("{}.{}", prefix, k_str);
                    inject_yaml_to_context(v, &new_prefix, context);
                }
            }
        }
        YamlValue::Sequence(s) => {
            for (i, v) in s.iter().enumerate() {
                let new_prefix = format!("{}[{}]", prefix, i);
                inject_yaml_to_context(v, &new_prefix, context);
            }
        }
        YamlValue::String(s) => {
            eprintln!(
                "{} {} {} {}",
                "Injecting property:".blue().bold(),
                prefix.yellow(),
                "=".white(),
                s.green()
            );
            context.set_property(prefix.to_string(), s.clone());
        }
        YamlValue::Number(n) => {
            eprintln!(
                "{} {} {} {}",
                "Injecting property:".blue().bold(),
                prefix.yellow(),
                "=".white(),
                n.to_string().green()
            );
            context.set_property(prefix.to_string(), n.to_string());
        }
        YamlValue::Bool(b) => {
            eprintln!(
                "{} {} {} {}",
                "Injecting property:".blue().bold(),
                prefix.yellow(),
                "=".white(),
                b.to_string().green()
            );
            context.set_property(prefix.to_string(), b.to_string());
        }
        YamlValue::Tagged(tv) => {
            let tag = tv.tag.to_string();
            match &tv.value {
                YamlValue::Number(n) => {
                    let mut val_f = n.as_f64().unwrap_or(0.0);
                    if tag == "!degrees" {
                        val_f = val_f * PI / 180.0;
                    }
                    eprintln!(
                        "{} {} {} {} {} {}",
                        "Injecting tagged property:".blue().bold(),
                        prefix.yellow(),
                        "=".white(),
                        val_f.to_string().green(),
                        "(tag:".white(),
                        format!("{})", tag).magenta()
                    );
                    context.set_property(prefix.to_string(), val_f.to_string());
                }
                YamlValue::String(s) => {
                    eprintln!(
                        "{} {} {} {} {} {}",
                        "Injecting tagged property:".blue().bold(),
                        prefix.yellow(),
                        "=".white(),
                        s.green(),
                        "(tag:".white(),
                        format!("{})", tag).magenta()
                    );
                    context.set_property(prefix.to_string(), s.clone());
                }
                _ => {
                    eprintln!(
                        "Injecting tagged property (complex): {} (tag: {})",
                        prefix, tag
                    );
                    inject_yaml_to_context(&tv.value, prefix, context);
                }
            }
        }
        _ => {
            eprintln!("Unknown YamlValue variant for prefix {}: {:?}", prefix, val);
        }
    }
}

fn is_xacro_element(node: &roxmltree::Node) -> bool {
    let tag_name = node.tag_name().name();
    if tag_name.starts_with("xacro:") {
        return true;
    }
    match node.tag_name().namespace() {
        Some("http://www.ros.org/wiki/xacro") => true,
        Some("http://ros.org/wiki/xacro") => true,
        Some("http://wiki.ros.org/xacro") => true,
        _ => false,
    }
}

fn resolve_package_path<P: ApolloPathBufTrait + Clone>(
    pkg_name: &str,
    search_dirs: &[P],
) -> Option<P> {
    for dir in search_dirs {
        // 1. Direct child (standard workspace)
        let candidate = dir.clone().append(pkg_name);
        if candidate.path_exists() {
            return Some(candidate);
        }

        // 2. Is the dir itself the package?
        let pkg_xml = dir.clone().append("package.xml");
        if pkg_xml.path_exists() {
            if let Ok(content) = pkg_xml.read_file_contents_to_string_result() {
                let needle = format!("<name>{}</name>", pkg_name);
                if content.contains(&needle) {
                    return Some(dir.clone());
                }
            }
        }

        // 3. One level deep scan
        if let Ok(entries) = std::fs::read_dir(dir.to_path_buf()) {
            for entry in entries.flatten() {
                if let Ok(file_type) = entry.file_type() {
                    if file_type.is_dir() {
                        let sub_dir = P::new_from_path(&entry.path());
                        let pkg_xml = sub_dir.clone().append("package.xml");
                        if pkg_xml.path_exists() {
                            if let Ok(content) = pkg_xml.read_file_contents_to_string_result() {
                                let needle = format!("<name>{}</name>", pkg_name);
                                if content.contains(&needle) {
                                    return Some(sub_dir);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    None
}
