pub mod traits;

use std::fmt::Debug;
use std::fs;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use serde::de::DeserializeOwned;
use serde::Serialize;
use walkdir::WalkDir;
use crate::traits::{ToJsonString, ToRonString, ToTomlString, ToYamlString};

/// A trait for various file path operations, providing a set of methods for file and directory manipulation, as well as handling JSON, TOML, YAML, and RON formats.
pub trait ApolloPathBufTrait: Sized {
    /// Creates a new `PathBuf` by appending a string to the root path.
    fn new_from_append(s: &str) -> Self;

    /// Creates a new `PathBuf` from the user's home directory.
    fn new_from_home_dir() -> Self;

    /// Creates a new `PathBuf` from the user's documents directory.
    fn new_from_documents_dir() -> Self;

    /// Creates a new `PathBuf` from the user's desktop directory.
    fn new_from_desktop_dir() -> Self;

    /// Creates a new `PathBuf` pointing to the default Apollo robots directory.
    fn new_from_default_apollo_robots_dir() -> Self;

    /// Creates a new `PathBuf` pointing to the default Apollo Bevy assets directory.
    fn new_from_default_apollo_bevy_assets_dir() -> Self;

    /// Creates a new `PathBuf` pointing to the default Apollo environments directory.
    fn new_from_default_apollo_environments_dir() -> Self;

    /// Recursively walks a directory and finds a file matching the provided path `s`.
    fn new_from_walk_dir_and_find<P: AsRef<Path> + Debug>(s: P) -> Self;

    /// Appends a string to the existing `PathBuf`.
    fn append(self, s: &str) -> Self;

    /// Appends multiple strings from a vector to the existing `PathBuf`.
    fn append_vec(self, v: &Vec<String>) -> Self;

    /// Appends a string without inserting a path separator.
    fn append_without_separator(self, s: &str) -> Self;

    /// Appends another path to the existing `PathBuf`.
    fn append_path<P: AsRef<Path>>(self, s: P) -> Self;

    /// Splits the `PathBuf` into a vector of individual path components as strings.
    fn split_into_strings(&self) -> Vec<String>;

    /// Splits the `PathBuf` into a vector of `PathBuf`s.
    fn split_into_path_bufs(&self) -> Vec<Self>;

    /// Recursively walks a directory and finds the first file matching the path `s`.
    fn walk_directory_and_find_first<P: AsRef<Path> + Debug>(self, s: P) -> Self;

    /// Same as `walk_directory_and_find_first`, but returns a `Result`, handling potential errors.
    fn walk_directory_and_find_first_result<P: AsRef<Path> + Debug>(self, s: P) -> Result<Self, String>;

    /// Walks a directory and finds all files matching the path `s`.
    fn walk_directory_and_find_all<P: AsRef<Path> + Debug>(self, s: P) -> Vec<Self>;

    /// Creates a directory at the `PathBuf` location if it doesn't exist.
    fn create_directory(&self);

    /// Deletes the file at the `PathBuf` location.
    fn delete_file(&self);

    /// Deletes the directory at the `PathBuf` location.
    fn delete_directory(&self);

    /// Attempts to delete the directory, returning a `Result`.
    fn delete_directory_result(&self) -> Result<(), String>;

    /// Deletes all items within the directory at the `PathBuf` location.
    fn delete_all_items_in_directory(&self);

    /// Copies a file to a destination file path.
    fn copy_file_to_destination_file_path(&self, destination: &Self);

    /// Copies a file to a destination directory.
    fn copy_file_to_destination_directory(&self, destination: &Self);

    /// Extracts the last `n` path segments from the `PathBuf`.
    fn extract_last_n_segments(&self, n: usize) -> Self;

    /// Retrieves all items in a directory with various filtering options (directories, hidden files, etc.).
    fn get_all_items_in_directory(
        &self, include_directories: bool, include_hidden_directories: bool,
        include_files: bool, include_hidden_files: bool
    ) -> Vec<Self>;

    /// Retrieves all filenames in a directory, optionally including hidden files.
    fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String>;

    /// Reads the contents of a file into a string.
    fn read_file_contents_to_string(&self) -> String;

    /// Same as `read_file_contents_to_string`, but returns a `Result` to handle errors.
    fn read_file_contents_to_string_result(&self) -> Result<String, String>;

    /// Writes a string to a file.
    fn write_string_to_file(&self, s: &String);

    /// Loads an object from a JSON file.
    fn load_object_from_json_file<T: Serialize + DeserializeOwned>(&self) -> T;

    /// Same as `load_object_from_json_file`, but returns a `Result`.
    fn load_object_from_json_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    /// Saves an object to a JSON file.
    fn save_object_to_json_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    /// Loads an object from a TOML file.
    fn load_object_from_toml_file<T: Serialize + DeserializeOwned>(&self) -> T;

    /// Same as `load_object_from_toml_file`, but returns a `Result`.
    fn load_object_from_toml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    /// Saves an object to a TOML file.
    fn save_object_to_toml_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    /// Loads an object from a RON file.
    fn load_object_from_ron_file<T: Serialize + DeserializeOwned>(&self) -> T;

    /// Same as `load_object_from_ron_file`, but returns a `Result`.
    fn load_object_from_ron_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    /// Saves an object to a RON file.
    fn save_object_to_ron_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    /// Loads an object from a YAML file.
    fn load_object_from_yaml_file<T: Serialize + DeserializeOwned>(&self) -> T;

    /// Same as `load_object_from_yaml_file`, but returns a `Result`.
    fn load_object_from_yaml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    /// Saves an object to a YAML file.
    fn save_object_to_yaml_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    /// Gets a file handle for writing.
    fn get_file_for_writing(&self) -> File;

    /// Gets a file handle for reading.
    fn get_file_for_reading(&self) -> File;

    /// Verifies that the file has one of the specified extensions.
    fn verify_extension(&self, extensions: &Vec<&str>) -> Result<(), String>;

    /// Gets the relative path from the current `PathBuf` to another `PathBuf`.
    fn get_a_to_b_path(&self, b: &PathBuf) -> PathBuf;
}

impl ApolloPathBufTrait for PathBuf {
    fn new_from_append(s: &str) -> Self {
        let mut p = PathBuf::new();
        p.push(std::path::MAIN_SEPARATOR_STR);
        return p.append(s);
    }

    fn new_from_home_dir() -> Self {
        dirs::home_dir().unwrap().to_path_buf()
    }

    fn new_from_documents_dir() -> Self {
        dirs::document_dir().unwrap().to_path_buf()
    }

    fn new_from_desktop_dir() -> Self {
        dirs::desktop_dir().unwrap().to_path_buf()
    }

    fn new_from_default_apollo_robots_dir() -> Self {
        let out = Self::new_from_documents_dir().append("apollo-resources/robots");
        assert!(out.exists(), "default apollo robots dir path {:?} does not exist.", out);
        out
    }

    fn new_from_default_apollo_bevy_assets_dir() -> Self {
        let out = PathBuf::new_from_documents_dir().append("apollo-rust/crates/apollo-rust-bevy/assets");
        assert!(out.exists(), "default apollo bevy assets path {:?} does not exist.", out);
        out
    }

    fn new_from_default_apollo_environments_dir() -> Self {
        let out = Self::new_from_documents_dir().append("apollo-resources/environments");
        assert!(out.exists(), "default apollo environments dir path {:?} does not exist.", out);
        out
    }

    fn new_from_walk_dir_and_find<P: AsRef<Path> + Debug>(s: P) -> Self {
        let p = PathBuf::new_from_home_dir();
        return p.walk_directory_and_find_first(s);
    }

    fn append(self, s: &str) -> Self {
        let mut out = self.clone();
        let do_s1 = s.find("/").is_some();
        let do_s2 = s.find(r"\").is_some();

        if do_s1 && do_s2 {
            panic!(r"cannot have both / and \ in append");
        }

        if do_s1 {
            let s1 = s.split("/");
            s1.for_each(|x| { out.push(x) });
        } else if do_s2 {
            let s2 = s.split(r"\");
            s2.for_each(|x| { out.push(x) });
        } else {
            out.push(s);
        }

        out
    }

    fn append_vec(self, v: &Vec<String>) -> Self {
        let mut out = PathBuf::new();
        for s in v {
            out = out.append(s);
        }
        out
    }

    fn append_without_separator(self, s: &str) -> Self {
        let mut ss = self.to_str().expect("error").to_string();
        ss += s;
        return Self::from(ss);
    }

    fn append_path<P: AsRef<Path>>(self, s: P) -> Self {
        let ss = s.as_ref().to_str().expect("error");
        return self.append(ss);
    }

    fn split_into_strings(&self) -> Vec<String> {
        let mut out = vec![];
        let s = self.to_str().expect("error");

        let do_s1 = s.contains("/");
        let do_s2 = s.contains(r"\");

        let pat = if do_s1 { "/" } else if do_s2 { r"\" } else { return vec![s.to_string()] };

        let ss = s.split(pat);
        ss.for_each(|x| out.push(x.to_string()));

        out
    }

    fn split_into_path_bufs(&self) -> Vec<Self> {
        let mut out = vec![];
        let s = self.to_str().expect("error");

        let do_s1 = s.contains("/");
        let do_s2 = s.contains(r"\");

        let pat = if do_s1 { "/" } else if do_s2 { r"\" } else { return vec![self.clone()] };

        let ss = s.split(pat);
        ss.for_each(|x| out.push(PathBuf::new_from_append(x)));

        out
    }

    fn walk_directory_and_find_first<P: AsRef<Path> + Debug>(self, s: P) -> Self {
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res {
                let entry_path = entry.into_path();
                if entry_path.ends_with(&s) { return entry_path; }
            }
        }

        panic!("could not find {:?}", s);
    }

    fn walk_directory_and_find_first_result<P: AsRef<Path> + Debug>(self, s: P) -> Result<Self, String> {
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res {
                let entry_path = entry.into_path();
                if entry_path.ends_with(&s) { return Ok(entry_path); }
            }
        }

        Err(format!("could not find {:?}", s))
    }

    fn walk_directory_and_find_all<P: AsRef<Path> + Debug>(self, s: P) -> Vec<Self> {
        let mut out = vec![];
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res {
                let entry_path = entry.into_path();
                if entry_path.ends_with(&s) { out.push(entry_path); }
            }
        }
        out
    }

    fn create_directory(&self) {
        if self.exists() { return; }
        fs::create_dir_all(self).expect("could not create directory");
    }

    fn delete_file(&self) {
        fs::remove_file(self).expect("could not delete file");
    }

    fn delete_directory(&self) {
        fs::remove_dir_all(self).expect("could not delete directory");
    }

    fn delete_directory_result(&self) -> Result<(), String> {
        let res = fs::remove_dir_all(self);

        return match res {
            Ok(_) => { Ok(()) }
            Err(e) => { Err(e.to_string()) }
        }
    }

    fn delete_all_items_in_directory(&self) {
        self.delete_directory();
        self.create_directory();
    }

    fn copy_file_to_destination_file_path(&self, destination: &Self) {
        assert!(self.is_file(), "must be a file");

        if !destination.exists() {
            let par = destination.parent().expect("error").to_path_buf();
            par.create_directory();
        }

        fs::copy(self, destination).expect("could not copy");
    }

    fn copy_file_to_destination_directory(&self, destination: &Self) {
        assert!(self.is_file(), "must be a file");

        let f = self.file_name().unwrap().to_str().unwrap();
        let d = destination.clone().append(f);

        self.copy_file_to_destination_file_path(&d);
    }

    fn extract_last_n_segments(&self, n: usize) -> Self {
        assert!(n > 0);

        let s = self.split_into_path_bufs();
        let n = n.min(s.len());

        let mut out = PathBuf::new();
        for i in 0..n {
            out = out.append_path(&s[s.len() - (n - i)]);
        }

        out
    }

    fn get_all_items_in_directory(
        &self,
        include_directories: bool,
        include_hidden_directories: bool,
        include_files: bool,
        include_hidden_files: bool
    ) -> Vec<Self> {
        let mut out = vec![];

        let res = self.read_dir();
        if let Ok(read_dir) = res {
            for dir_entry_res in read_dir {
                if let Ok(dir_entry) = dir_entry_res {
                    let filename = dir_entry.file_name();
                    let f = filename.to_str().unwrap().to_string();

                    if include_directories && dir_entry.path().is_dir() {
                        if include_hidden_directories {
                            out.push(dir_entry.path());
                        } else {
                            if !(f.chars().nth(0).unwrap().to_string() == ".") {
                                out.push(dir_entry.path());
                            }
                        }
                    }
                    else if include_files && dir_entry.path().is_file() {
                        if include_hidden_files {
                            out.push(dir_entry.path());
                        } else {
                            if !(f.chars().nth(0).unwrap().to_string() == ".") {
                                out.push(dir_entry.path());
                            }
                        }
                    }
                }
            }
        }

        out
    }

    fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String> {
        let mut out = vec![];
        let items = self.get_all_items_in_directory(false, false, true, include_hidden_files);

        items.iter().for_each(|x| {
            out.push(x.file_name().expect("error").to_str().expect("error").to_string())
        });

        out
    }

    fn read_file_contents_to_string(&self) -> String {
        let mut file_res = File::open(self);
        return match &mut file_res {
            Ok(f) => {
                let mut contents = String::new();
                let res = f.read_to_string(&mut contents);
                if res.is_err() {
                    panic!("could not load file with path {:?}.  Error: {:?}", self, res.err());
                }
                contents
            }
            Err(e) => {
                panic!("could not load file with path {:?}.  Error: {}", self, e);
            }
        }
    }

    fn read_file_contents_to_string_result(&self) -> Result<String, String> {
        let mut file_res = File::open(self);
        return match &mut file_res {
            Ok(f) => {
                let mut contents = String::new();
                let res = f.read_to_string(&mut contents);
                match res {
                    Ok(_) => { Ok(contents) }
                    Err(e) => { Err(format!("could not load file with path {:?}.  Error: {:?}", self, e)) }
                }
            }
            Err(e) => {
                Err(format!("could not load file with path {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn write_string_to_file(&self, s: &String) {
        let parent_option = self.parent();
        match parent_option {
            None => {
                panic!("Could not get parent of path in save_object_to_file_as_json.");
            }
            Some(parent) => {
                fs::create_dir_all(parent).expect("error");
            }
        }

        if self.exists() { fs::remove_file(self).expect("error"); }

        let mut file_res = OpenOptions::new()
            .write(true)
            .create(true)
            .open(self);

        match &mut file_res {
            Ok(f) => {
                f.write(s.as_bytes()).expect("error");
            }
            Err(e) => {
                panic!("could not write string to file.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_json_file<T: Serialize + DeserializeOwned>(&self) -> T {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "json");

        let json_str = self.read_file_contents_to_string();
        let o_res = serde_json::from_str(&json_str);
        return match o_res {
            Ok(o) => {
                o
            }
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_json_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "json");

        let json_str = self.read_file_contents_to_string_result()?;
        let o_res = serde_json::from_str(&json_str);
        return match o_res {
            Ok(o) => {
                Ok(o)
            }
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_json_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "json");

        let s = object.to_json_string();
        self.write_string_to_file(&s);
    }

    fn load_object_from_toml_file<T: Serialize + DeserializeOwned>(&self) -> T {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "toml");

        let toml_str = self.read_file_contents_to_string();
        let o_res = toml::from_str(&toml_str);
        return match o_res {
            Ok(o) => {
                o
            }
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_toml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "toml");

        let toml_str = self.read_file_contents_to_string_result()?;
        let o_res = toml::from_str(&toml_str);
        return match o_res {
            Ok(o) => {
                Ok(o)
            }
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_toml_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "toml");

        let s = object.to_toml_string();
        self.write_string_to_file(&s);
    }

    fn load_object_from_ron_file<T: Serialize + DeserializeOwned>(&self) -> T {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "ron");

        let ron_str = self.read_file_contents_to_string();
        let o_res = ron::from_str(&ron_str);
        return match o_res {
            Ok(o) => {
                o
            }
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_ron_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "ron");

        let ron_str = self.read_file_contents_to_string_result()?;
        let o_res = ron::from_str(&ron_str);
        return match o_res {
            Ok(o) => {
                Ok(o)
            }
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_ron_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "ron");

        let s = object.to_ron_string();
        self.write_string_to_file(&s);
    }

    fn load_object_from_yaml_file<T: Serialize + DeserializeOwned>(&self) -> T {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "yaml");

        let yaml_str = self.read_file_contents_to_string();
        let o_res = serde_yaml::from_str(&yaml_str);
        return match o_res {
            Ok(o) => {
                o
            }
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_yaml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "yaml");

        let yaml_str = self.read_file_contents_to_string_result()?;
        let o_res = serde_yaml::from_str(&yaml_str);
        return match o_res {
            Ok(o) => {
                Ok(o)
            }
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_yaml_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "yaml");

        let s = object.to_yaml_string();
        self.write_string_to_file(&s);
    }

    fn get_file_for_writing(&self) -> File {
        let prefix = self.parent().unwrap();
        fs::create_dir_all(prefix).unwrap();
        if self.exists() { fs::remove_file(self).expect("error"); }
        let file = OpenOptions::new().write(true).create_new(true).open(self).expect("error");
        file
    }

    fn get_file_for_reading(&self) -> File {
        let file = OpenOptions::new().read(true).open(self).expect("error");
        file
    }

    fn verify_extension(&self, extensions: &Vec<&str>) -> Result<(), String> {
        let ext_option = self.extension();
        match ext_option {
            None => {
                return Err(format!("Path {:?} does not have one of the following extensions: {:?}", self, extensions))
            }
            Some(ext) => {
                for e in extensions {
                    if e == &ext {
                        return Ok(());
                    }
                }
            }
        }
        Err(format!("Path {:?} does not have one of the following extensions: {:?}", self, extensions))
    }

    fn get_a_to_b_path(&self, b: &PathBuf) -> PathBuf {
        let mut out = self.clone();

        let s = self.split_into_path_bufs();
        let l = s.len();
        for _ in 0..l-1 {
            out = out.clone().append("..");
        }

        return out.append_path(b);
    }
}
