use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct MacroDefinition {
    pub name: String,
    pub params: Vec<(String, String)>,
    pub content: String,
}

#[derive(Clone, Debug)]
struct XacroScope {
    properties: HashMap<String, String>,
    blocks: HashMap<String, String>,
}

#[derive(Clone, Debug)]
pub struct XacroContext {
    scopes: Vec<XacroScope>,
    macros: HashMap<String, MacroDefinition>,
    arguments: HashMap<String, String>,
    search_dirs: Vec<String>,
}

impl XacroContext {
    pub fn new() -> Self {
        Self {
            scopes: vec![XacroScope {
                properties: HashMap::new(),
                blocks: HashMap::new(),
            }],
            macros: HashMap::new(),
            arguments: HashMap::new(),
            search_dirs: Vec::new(),
        }
    }

    // Compatibility shim for new_child
    pub fn push_scope(&mut self) {
        self.scopes.push(XacroScope {
            properties: HashMap::new(),
            blocks: HashMap::new(),
        });
    }

    pub fn pop_scope(&mut self) {
        if self.scopes.len() > 1 {
            self.scopes.pop();
        }
    }

    pub fn with_scope<F, T, E>(&mut self, f: F) -> Result<T, E>
    where
        F: FnOnce(&mut XacroContext) -> Result<T, E>,
    {
        self.push_scope();
        let res = f(self);
        self.pop_scope();
        res
    }

    pub fn get_property(&self, key: &str) -> Option<&String> {
        for scope in self.scopes.iter().rev() {
            if let Some(val) = scope.properties.get(key) {
                return Some(val);
            }
        }
        None
    }

    pub fn set_property(&mut self, key: String, value: String) {
        if let Some(scope) = self.scopes.last_mut() {
            scope.properties.insert(key, value);
        }
    }

    pub fn set_property_parent(&mut self, key: String, value: String) {
        let len = self.scopes.len();
        if len > 1 {
            self.scopes[len - 2].properties.insert(key, value);
        } else {
            self.set_property(key, value);
        }
    }

    pub fn set_property_global(&mut self, key: String, value: String) {
        self.scopes[0].properties.insert(key, value);
    }

    pub fn get_macro(&self, key: &str) -> Option<&MacroDefinition> {
        self.macros.get(key)
    }

    pub fn register_macro(&mut self, def: MacroDefinition) {
        self.macros.insert(def.name.clone(), def);
    }

    pub fn get_argument(&self, key: &str) -> Option<&String> {
        self.arguments.get(key)
    }

    pub fn set_argument(&mut self, key: String, value: String) {
        self.arguments.insert(key, value);
    }

    pub fn get_block(&self, key: &str) -> Option<&String> {
        for scope in self.scopes.iter().rev() {
            if let Some(val) = scope.blocks.get(key) {
                return Some(val);
            }
        }
        None
    }

    pub fn set_block(&mut self, key: String, value: String) {
        if let Some(scope) = self.scopes.last_mut() {
            scope.blocks.insert(key, value);
        }
    }

    pub fn get_all_properties(&self) -> HashMap<String, String> {
        let mut props = HashMap::new();
        // Traverse from bottom up so top scopes overwrite bottom ones
        for scope in &self.scopes {
            for (k, v) in &scope.properties {
                props.insert(k.clone(), v.clone());
            }
        }
        props
    }

    pub fn get_search_dirs(&self) -> &Vec<String> {
        &self.search_dirs
    }

    pub fn set_search_dirs(&mut self, dirs: Vec<String>) {
        self.search_dirs = dirs;
    }
}
