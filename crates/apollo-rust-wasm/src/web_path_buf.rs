use apollo_rust_file::ApolloPathBufTrait;
use lazy_static::lazy_static;
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt::{Debug, Display, Formatter};
use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

// Global in-memory filesystem
lazy_static! {
    static ref VIRTUAL_FS: Arc<RwLock<HashMap<String, Vec<u8>>>> =
        Arc::new(RwLock::new(HashMap::new()));
}

#[derive(Serialize, Deserialize, Clone, Eq, PartialEq, Hash)]
pub struct WebPathBuf {
    path: String,
}

impl Debug for WebPathBuf {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.path)
    }
}

impl Display for WebPathBuf {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.path)
    }
}

impl WebPathBuf {
    fn normalize_path(path: &str) -> String {
        // Simple normalization: replace backslashes, remove consecutive slashes, trim
        // This is a basic implementation for the virtual FS
        let p = path.replace("\\", "/");
        // Remove double slashes?
        // keeping it simple for now.
        if p.starts_with("./") {
            p[2..].to_string()
        } else {
            p
        }
    }

    pub fn insert_file(path: &str, content: Vec<u8>) {
        let normalized = Self::normalize_path(path);
        VIRTUAL_FS.write().unwrap().insert(normalized, content);
    }
}

impl ApolloPathBufTrait for WebPathBuf {
    fn path_exists(&self) -> bool {
        VIRTUAL_FS.read().unwrap().contains_key(&self.path)
    }

    fn append_another(self, other: &Self) -> Self {
        self.append(&other.path)
    }

    fn new_from_str(s: &str) -> Self {
        Self {
            path: Self::normalize_path(s),
        }
    }

    fn new_from_path(p: &PathBuf) -> Self {
        Self {
            path: Self::normalize_path(p.to_str().unwrap_or("")),
        }
    }

    fn new_from_append(s: &str) -> Self {
        Self {
            path: Self::normalize_path(s),
        }
    }

    fn new_from_home_dir() -> Self {
        Self {
            path: "home".to_string(),
        }
    }

    fn new_from_documents_dir() -> Self {
        Self {
            path: "documents".to_string(),
        }
    }

    fn new_from_desktop_dir() -> Self {
        Self {
            path: "desktop".to_string(),
        }
    }

    fn new_from_default_apollo_robots_dir() -> Self {
        Self {
            path: "documents/apollo-resources/robots".to_string(),
        }
    }

    fn new_from_default_apollo_bevy_assets_dir() -> Self {
        Self {
            path: "documents/apollo-rust/crates/apollo-rust-bevy/assets".to_string(),
        }
    }

    fn new_from_default_apollo_environments_dir() -> Self {
        Self {
            path: "documents/apollo-resources/environments".to_string(),
        }
    }

    fn new_from_walk_dir_and_find<P: AsRef<Path> + Debug>(s: P) -> Self {
        // Mock implementation: just append the file name to home/found
        let s_str = s.as_ref().to_str().unwrap();
        Self {
            path: format!("home/found/{}", s_str),
        }
    }

    fn append(self, s: &str) -> Self {
        let separator = if self.path.ends_with("/") || self.path.is_empty() {
            ""
        } else {
            "/"
        };
        let new_path = format!("{}{}{}", self.path, separator, s);
        Self {
            path: Self::normalize_path(&new_path),
        }
    }

    fn append_vec(self, v: &Vec<String>) -> Self {
        let mut curr = self;
        for s in v {
            curr = curr.append(s);
        }
        curr
    }

    fn append_without_separator(self, s: &str) -> Self {
        Self {
            path: format!("{}{}", self.path, s),
        }
    }

    fn append_path(self, other: &PathBuf) -> Self {
        self.append(other.to_str().unwrap())
    }

    fn to_path_buf(&self) -> PathBuf {
        PathBuf::from(&self.path)
    }

    fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    fn split_into_strings(&self) -> Vec<String> {
        self.path.split('/').map(|s| s.to_string()).collect()
    }

    fn split_into_path_bufs(&self) -> Vec<Self> {
        self.split_into_strings()
            .into_iter()
            .map(|s| Self::new_from_str(&s))
            .collect()
    }

    fn walk_directory_and_find_first<P: AsRef<Path> + Debug>(self, _s: P) -> Self {
        panic!("walk_directory_and_find_first not implemented for WebPathBuf")
    }

    fn walk_directory_and_find_first_result<P: AsRef<Path> + Debug>(
        self,
        _s: P,
    ) -> Result<Self, String> {
        Err("walk_directory_and_find_first_result not implemented for WebPathBuf".to_string())
    }

    fn walk_directory_and_find_all<P: AsRef<Path> + Debug>(self, _s: P) -> Vec<Self> {
        vec![]
    }

    fn create_directory(&self) {
        // No-op for virtual FS
    }

    fn delete_file(&self) {
        VIRTUAL_FS.write().unwrap().remove(&self.path);
    }

    fn delete_directory(&self) {
        // Naive implementation: remove all keys starting with this path
        let mut fs = VIRTUAL_FS.write().unwrap();
        let keys_to_remove: Vec<String> = fs
            .keys()
            .filter(|k| k.starts_with(&self.path))
            .cloned()
            .collect();
        for k in keys_to_remove {
            fs.remove(&k);
        }
    }

    fn delete_directory_result(&self) -> Result<(), String> {
        self.delete_directory();
        Ok(())
    }

    fn delete_all_items_in_directory(&self) {
        self.delete_directory();
    }

    fn copy_file_to_destination_file_path(&self, destination: &Self) {
        let fs_lock = VIRTUAL_FS.read().unwrap();
        if let Some(content) = fs_lock.get(&self.path) {
            let content_clone = content.clone();
            drop(fs_lock);
            VIRTUAL_FS
                .write()
                .unwrap()
                .insert(destination.path.clone(), content_clone);
        } else {
            panic!("Source file {:?} does not exist", self);
        }
    }

    fn path_extension(&self) -> Option<String> {
        self.to_path_buf()
            .extension()
            .map(|s| s.to_str().unwrap().to_string())
    }

    fn copy_file_to_destination_directory(&self, destination: &Self) {
        let file_name = self
            .to_path_buf()
            .file_name()
            .unwrap()
            .to_str()
            .unwrap()
            .to_string();
        let dest_file = destination.clone().append(&file_name);
        self.copy_file_to_destination_file_path(&dest_file);
    }

    fn extract_last_n_segments(&self, n: usize) -> Self {
        let parts = self.split_into_strings();
        let start = if parts.len() > n { parts.len() - n } else { 0 };
        let new_path = parts[start..].join("/");
        Self { path: new_path }
    }

    fn parent(&self) -> Option<Self> {
        let mut parts = self.split_into_strings();
        if parts.len() > 1 {
            parts.pop();
            Some(Self {
                path: parts.join("/"),
            })
        } else {
            None
        }
    }

    fn get_all_items_in_directory(
        &self,
        _include_directories: bool,
        _include_hidden_directories: bool,
        _include_files: bool,
        _include_hidden_files: bool,
    ) -> Vec<Self> {
        // Naive: return all keys starting with this path
        let fs = VIRTUAL_FS.read().unwrap();
        let prefix = if self.path.ends_with("/") {
            self.path.clone()
        } else {
            format!("{}/", self.path)
        };

        fs.keys()
            .filter(|k| k.starts_with(&prefix))
            .map(|k| Self { path: k.clone() })
            .collect()
    }

    fn get_all_filenames_in_directory(&self, _include_hidden_files: bool) -> Vec<String> {
        // Just return full paths for now as filenames
        self.get_all_items_in_directory(false, false, true, true)
            .iter()
            .map(|p| p.path.clone())
            .collect()
    }

    fn read_file_contents_to_string(&self) -> String {
        self.read_file_contents_to_string_result()
            .expect(&format!("Could not read file {:?}", self))
    }

    fn read_file_contents_to_string_result(&self) -> Result<String, String> {
        let fs = VIRTUAL_FS.read().unwrap();
        match fs.get(&self.path) {
            Some(bytes) => String::from_utf8(bytes.clone()).map_err(|e| e.to_string()),
            None => Err(format!("File not found: {:?}", self)),
        }
    }

    fn read_file_contents_to_bytes(&self) -> Vec<u8> {
        self.read_file_contents_to_bytes_result()
            .expect(&format!("Could not read file {:?}", self))
    }

    fn read_file_contents_to_bytes_result(&self) -> Result<Vec<u8>, String> {
        let fs = VIRTUAL_FS.read().unwrap();
        match fs.get(&self.path) {
            Some(bytes) => Ok(bytes.clone()),
            None => Err(format!("File not found: {:?}", self)),
        }
    }

    fn write_string_to_file(&self, s: &String) {
        VIRTUAL_FS
            .write()
            .unwrap()
            .insert(self.path.clone(), s.as_bytes().to_vec());
    }

    // Default implementations for object loading/saving usually rely on read_file_contents_to_string
    // But since they are default methods in the trait (if they were), we might not need to impl them.
    // However, ApolloPathBufTrait defines them without default impls?
    // Checking lib.rs... yes, they are required.

    fn load_object_from_json_file<T: Serialize + DeserializeOwned>(&self) -> T {
        serde_json::from_str(&self.read_file_contents_to_string()).unwrap()
    }

    fn load_object_from_json_file_result<T: Serialize + DeserializeOwned>(
        &self,
    ) -> Result<T, String> {
        let s = self.read_file_contents_to_string_result()?;
        serde_json::from_str(&s).map_err(|e| e.to_string())
    }

    fn save_object_to_json_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        let s = serde_json::to_string_pretty(object).unwrap();
        self.write_string_to_file(&s);
    }

    fn load_object_from_toml_file<T: Serialize + DeserializeOwned>(&self) -> T {
        panic!("TOML not supported in WebPathBuf yet");
    }

    fn load_object_from_toml_file_result<T: Serialize + DeserializeOwned>(
        &self,
    ) -> Result<T, String> {
        Err("TOML not supported".to_string())
    }

    fn save_object_to_toml_file<T: Serialize + DeserializeOwned>(&self, _object: &T) {
        panic!("TOML not supported");
    }

    fn load_object_from_ron_file<T: Serialize + DeserializeOwned>(&self) -> T {
        panic!("RON not supported");
    }

    fn load_object_from_ron_file_result<T: Serialize + DeserializeOwned>(
        &self,
    ) -> Result<T, String> {
        Err("RON not supported".to_string())
    }

    fn save_object_to_ron_file<T: Serialize + DeserializeOwned>(&self, _object: &T) {
        panic!("RON not supported");
    }

    fn load_object_from_yaml_file<T: Serialize + DeserializeOwned>(&self) -> T {
        panic!("YAML not supported");
    }

    fn load_object_from_yaml_file_result<T: Serialize + DeserializeOwned>(
        &self,
    ) -> Result<T, String> {
        Err("YAML not supported".to_string())
    }

    fn save_object_to_yaml_file<T: Serialize + DeserializeOwned>(&self, _object: &T) {
        panic!("YAML not supported");
    }

    fn get_file_for_writing(&self) -> File {
        panic!("get_file_for_writing returns std::fs::File, which is not supported in WASM");
    }

    fn get_file_for_reading(&self) -> File {
        panic!("get_file_for_reading returns std::fs::File, which is not supported in WASM");
    }

    fn verify_extension(&self, extensions: &Vec<&str>) -> Result<(), String> {
        if let Some(ext) = self.path_extension() {
            if extensions.contains(&ext.as_str()) {
                return Ok(());
            }
        }
        Err("Invalid extension".to_string())
    }

    fn get_a_to_b_path(&self, b: &PathBuf) -> PathBuf {
        PathBuf::from(&self.path).join(b)
    }
}
