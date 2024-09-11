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

// Define a trait for file path operations, providing a set of methods
// for file and directory manipulation, as well as JSON, TOML, YAML, and RON handling.
pub trait ApolloPathBufTrait: Sized {
    // Create a new PathBuf by appending a string to the root path ("/").
    fn new_from_append(s: &str) -> Self;

    // Create a new PathBuf from the user's home directory.
    fn new_from_home_dir() -> Self;

    // Create a PathBuf from the user's documents directory.
    fn new_from_documents_dir() -> Self;

    // Create a PathBuf from the user's desktop directory.
    fn new_from_desktop_dir() -> Self;

    // Create a PathBuf pointing to the default Apollo robots directory.
    fn new_from_default_apollo_robots_dir() -> Self;

    // Create a PathBuf pointing to the default Apollo Bevy assets directory.
    fn new_from_default_apollo_bevy_assets_dir() -> Self;

    // Create a PathBuf pointing to the default Apollo environments directory.
    fn new_from_default_apollo_environments_dir() -> Self;

    // Recursively walk a directory and find a file that matches the provided path `s`.
    fn new_from_walk_dir_and_find<P: AsRef<Path> + Debug>(s: P) -> Self;

    // Append a string to an existing PathBuf.
    fn append(self, s: &str) -> Self;

    // Append multiple strings from a Vec to an existing PathBuf.
    fn append_vec(self, v: &Vec<String>) -> Self;

    // Append a string without inserting a separator.
    fn append_without_separator(self, s: &str) -> Self;

    // Append another path to the existing PathBuf.
    fn append_path<P: AsRef<Path>>(self, s: P) -> Self;

    // Split the PathBuf into a Vec of individual path components as strings.
    fn split_into_strings(&self) -> Vec<String>;

    // Split the PathBuf into a Vec of PathBufs.
    fn split_into_path_bufs(&self) -> Vec<Self>;

    // Recursively walk a directory and find the first matching file.
    fn walk_directory_and_find_first<P: AsRef<Path> + Debug>(self, s: P) -> Self;

    // Same as above, but return a Result, handling potential errors.
    fn walk_directory_and_find_first_result<P: AsRef<Path> + Debug>(self, s: P) -> Result<Self, String>;

    // Walk a directory and find all matching files.
    fn walk_directory_and_find_all<P: AsRef<Path> + Debug>(self, s: P) -> Vec<Self>;

    // Create a directory at the PathBuf location if it doesn't exist.
    fn create_directory(&self);

    // Delete the file at the PathBuf location.
    fn delete_file(&self);

    // Delete the directory at the PathBuf location.
    fn delete_directory(&self);

    // Try to delete the directory, returning a Result.
    fn delete_directory_result(&self) -> Result<(), String>;

    // Delete all items within the directory at the PathBuf location.
    fn delete_all_items_in_directory(&self);

    // Copy a file to a destination file path.
    fn copy_file_to_destination_file_path(&self, destination: &Self);

    // Copy a file to a destination directory.
    fn copy_file_to_destination_directory(&self, destination: &Self);

    // Extract the last `n` path segments from the PathBuf.
    fn extract_last_n_segments(&self, n: usize) -> Self;

    // Retrieve all items in a directory with various filtering options (directories, hidden files, etc.).
    fn get_all_items_in_directory(
        &self, include_directories: bool, include_hidden_directories: bool,
        include_files: bool, include_hidden_files: bool
    ) -> Vec<Self>;

    // Get filenames from a directory, optionally including hidden files.
    fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String>;

    // Read the contents of a file into a string.
    fn read_file_contents_to_string(&self) -> String;

    // Same as above, but return a Result to handle errors.
    fn read_file_contents_to_string_result(&self) -> Result<String, String>;

    // Write a string to a file.
    fn write_string_to_file(&self, s: &String);

    // Load an object from a JSON file.
    fn load_object_from_json_file<T: Serialize + DeserializeOwned>(&self) -> T;

    // Same as above, but return a Result.
    fn load_object_from_json_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    // Save an object to a JSON file.
    fn save_object_to_json_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    // Load an object from a TOML file.
    fn load_object_from_toml_file<T: Serialize + DeserializeOwned>(&self) -> T;

    // Same as above, but return a Result.
    fn load_object_from_toml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    // Save an object to a TOML file.
    fn save_object_to_toml_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    // Load an object from a RON file.
    fn load_object_from_ron_file<T: Serialize + DeserializeOwned>(&self) -> T;

    // Same as above, but return a Result.
    fn load_object_from_ron_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    // Save an object to a RON file.
    fn save_object_to_ron_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    // Load an object from a YAML file.
    fn load_object_from_yaml_file<T: Serialize + DeserializeOwned>(&self) -> T;

    // Same as above, but return a Result.
    fn load_object_from_yaml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String>;

    // Save an object to a YAML file.
    fn save_object_to_yaml_file<T: Serialize + DeserializeOwned>(&self, object: &T);

    // Get a file handle for writing.
    fn get_file_for_writing(&self) -> File;

    // Get a file handle for reading.
    fn get_file_for_reading(&self) -> File;

    // Verify that the file has one of the specified extensions.
    fn verify_extension(&self, extensions: &Vec<&str>) -> Result<(), String>;

    // Get the relative path from the current PathBuf to another PathBuf.
    fn get_a_to_b_path(&self, b: &PathBuf) -> PathBuf;
}


impl ApolloPathBufTrait for PathBuf {
    // Creates a new PathBuf, starting from the root directory ("/" or "\\" depending on the platform)
    // and appending the provided string `s` to it.
    fn new_from_append(s: &str) -> Self {
        let mut p = PathBuf::new(); // Initialize a new empty PathBuf
        p.push(std::path::MAIN_SEPARATOR_STR); // Push the root directory ("/" or "\\")
        return p.append(s); // Append the provided string `s` and return the result
    }

    // Returns a PathBuf pointing to the user's home directory.
    // Uses the `dirs` crate to retrieve the home directory and converts it to a PathBuf.
    fn new_from_home_dir() -> Self {
        dirs::home_dir().unwrap().to_path_buf() // If the home directory exists, unwrap and convert to PathBuf
    }

    // Returns a PathBuf pointing to the user's documents directory.
    // Uses the `dirs` crate to retrieve the documents directory.
    fn new_from_documents_dir() -> Self {
        dirs::document_dir().unwrap().to_path_buf() // If the documents directory exists, unwrap and convert to PathBuf
    }

    // Returns a PathBuf pointing to the user's desktop directory.
    // Uses the `dirs` crate to retrieve the desktop directory.
    fn new_from_desktop_dir() -> Self {
        dirs::desktop_dir().unwrap().to_path_buf() // If the desktop directory exists, unwrap and convert to PathBuf
    }

    // Returns a PathBuf pointing to the default Apollo robots directory within the documents folder.
    // Asserts that the directory exists; otherwise, the program will panic with an error message.
    fn new_from_default_apollo_robots_dir() -> Self {
        let out = Self::new_from_documents_dir().append("apollo-resources/robots"); // Append the specific directory
        assert!(out.exists(), "default apollo robots dir path {:?} does not exist.", out); // Ensure it exists
        out // Return the PathBuf
    }

    // Returns a PathBuf pointing to the default Apollo Bevy assets directory within the documents folder.
    // Asserts that the directory exists, or panics with an error message.
    fn new_from_default_apollo_bevy_assets_dir() -> Self {
        let out = PathBuf::new_from_documents_dir().append("apollo-rust/crates/apollo-rust-bevy/assets"); // Append path
        assert!(out.exists(), "default apollo bevy assets path {:?} does not exist.", out); // Ensure it exists
        out // Return the PathBuf
    }

    // Returns a PathBuf pointing to the default Apollo environments directory within the documents folder.
    // Asserts that the directory exists; panics with an error message if it does not.
    fn new_from_default_apollo_environments_dir() -> Self {
        let out = Self::new_from_documents_dir().append("apollo-resources/environments"); // Append the environment path
        assert!(out.exists(), "default apollo environments dir path {:?} does not exist.", out); // Ensure it exists
        out // Return the PathBuf
    }

    // Returns a PathBuf by walking through the user's home directory and finding the first match for the given path `s`.
    // This uses the `walk_directory_and_find_first` function to perform the search.
    fn new_from_walk_dir_and_find<P: AsRef<Path> + Debug>(s: P) -> Self {
        let p = PathBuf::new_from_home_dir(); // Start from the home directory
        return p.walk_directory_and_find_first(s); // Walk the directory and find the first match
    }

    // Appends a string `s` to the current PathBuf. Handles both `/` and `\` separators, but will panic if both are used.
    // This ensures that no conflicting path separators are present.
    fn append(self, s: &str) -> Self {
        let mut out = self.clone(); // Clone the current PathBuf to avoid mutating it directly

        // Check if the input string `s` contains `/` or `\` as a separator
        let do_s1 = s.find("/").is_some();
        let do_s2 = s.find(r"\").is_some();

        // Panic if both `/` and `\` are present in the input string, as this would cause conflicts
        if do_s1 && do_s2 {
            panic!(r"cannot have both / and \ in append");
        }

        // If `/` is used, split the string by `/` and append each part to the PathBuf
        if do_s1 {
            let s1 = s.split("/");
            s1.for_each(|x| { out.push(x) }); // Push each component separately
            // If `\` is used, split by `\` and append each part
        } else if do_s2 {
            let s2 = s.split(r"\");
            s2.for_each(|x| { out.push(x) }); // Push each component separately
            // Otherwise, append the entire string `s` directly to the PathBuf
        } else {
            out.push(s);
        }

        out // Return the modified PathBuf
    }

    // Appends a vector of strings `v` to the current PathBuf, creating a new PathBuf by appending each element.
    fn append_vec(self, v: &Vec<String>) -> Self {
        let mut out = PathBuf::new(); // Initialize an empty PathBuf
        for s in v {
            out = out.append(s); // Append each string from the vector to the PathBuf
        }
        out // Return the final PathBuf with all components appended
    }

    // Appends a string to the current PathBuf without inserting a path separator.
    // This function concatenates the string `s` directly to the existing path without adding any separators.
    fn append_without_separator(self, s: &str) -> Self {
        let mut ss = self.to_str().expect("error").to_string(); // Convert the current PathBuf to a string
        ss += s; // Concatenate the input string `s` without adding a separator
        return Self::from(ss) // Convert the string back into a PathBuf and return it
    }

    // Appends another path `s` to the current PathBuf.
    // This is similar to the `append` function, but the input is a generic type that can be converted into a `Path`.
    fn append_path<P: AsRef<Path>>(self, s: P) -> Self {
        let ss = s.as_ref().to_str().expect("error"); // Convert the input `s` to a string reference
        return self.append(ss); // Append the path string to the current PathBuf
    }

    // Splits the PathBuf into a vector of strings, representing each component of the path.
    // It handles both '/' (Unix) and '\' (Windows) path separators.
    fn split_into_strings(&self) -> Vec<String> {
        let mut out = vec![]; // Initialize an empty vector to store the path components
        let s = self.to_str().expect("error"); // Convert the PathBuf to a string reference

        // Check if the string contains '/' or '\' to determine the path separator used
        let do_s1 = s.contains("/");
        let do_s2 = s.contains(r"\");

        // Set the correct separator based on the system or path format
        let pat = if do_s1 { "/" } else if do_s2 { r"\" } else { return vec![s.to_string()] };

        // Split the string by the determined separator and collect components into the vector
        let ss = s.split(pat);
        ss.for_each(|x| out.push(x.to_string())); // Convert each component to a string and push it to the output vector

        out // Return the vector of path components as strings
    }

    // Splits the PathBuf into a vector of PathBufs, where each component is a separate PathBuf.
    // Similar to `split_into_strings`, but returns PathBuf objects instead of strings.
    fn split_into_path_bufs(&self) -> Vec<Self> {
        let mut out = vec![]; // Initialize an empty vector to store the path components as PathBufs
        let s = self.to_str().expect("error"); // Convert the PathBuf to a string reference

        // Check for path separators to determine whether the path uses '/' or '\'
        let do_s1 = s.contains("/");
        let do_s2 = s.contains(r"\");

        // Set the correct separator based on the format of the path string
        let pat = if do_s1 { "/" } else if do_s2 { r"\" } else { return vec![self.clone()] };

        // Split the path string by the determined separator and convert each part into a new PathBuf
        let ss = s.split(pat);
        ss.for_each(|x| out.push(PathBuf::new_from_append(x))); // For each part, create a PathBuf and push to the output vector

        out // Return the vector of PathBuf components
    }

    // Walks through the directory represented by the current PathBuf and returns the first match for the given path `s`.
    // This function uses the WalkDir crate to recursively search through the directory.
    fn walk_directory_and_find_first<P: AsRef<Path> + Debug>(self, s: P) -> Self {
        // Iterate through each entry in the directory using the WalkDir iterator
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res { // Ensure the directory entry is valid
                let entry_path = entry.into_path(); // Convert the directory entry into a PathBuf
                if entry_path.ends_with(&s) { return entry_path; } // Return the entry if it matches the search path
            }
        }

        // Panic if no matching path is found
        panic!("could not find {:?}", s);
    }
    // Similar to `walk_directory_and_find_first`, but returns a Result instead of panicking if the path is not found.
    // This allows for error handling instead of a forced program exit.
    fn walk_directory_and_find_first_result<P: AsRef<Path> + Debug>(self, s: P) -> Result<Self, String> {
        // Iterate through the directory using the WalkDir iterator
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res { // Ensure the directory entry is valid
                let entry_path = entry.into_path(); // Convert the directory entry into a PathBuf
                if entry_path.ends_with(&s) { return Ok(entry_path); } // Return the path if a match is found
            }
        }

        // Return an error message if no match is found
        Err(format!("could not find {:?}", s))
    }
    // Walks the directory and returns all matching paths as a vector of PathBufs.
    // This function searches for all occurrences of the given path `s`.
    fn walk_directory_and_find_all<P: AsRef<Path> + Debug>(self, s: P) -> Vec<Self> {
        let mut out = vec![]; // Initialize an empty vector to store all matching paths
        // Iterate through the directory using WalkDir to find all matching paths
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res { // Ensure the directory entry is valid
                let entry_path = entry.into_path(); // Convert the directory entry into a PathBuf
                if entry_path.ends_with(&s) { out.push(entry_path); } // Add the matching path to the output vector
            }
        }
        out // Return the vector containing all matching paths
    }

    // Creates a directory at the current PathBuf location.
    // This function ensures that the directory exists, creating it if necessary.
    fn create_directory(&self) {
        if self.exists() { return; } // If the directory already exists, do nothing
        fs::create_dir_all(self).expect("could not create directory"); // Create the directory and all necessary parent directories
    }

    // Deletes the file at the current PathBuf location.
    // This function will fail if the file doesn't exist or cannot be deleted.
    fn delete_file(&self) {
        fs::remove_file(self).expect("could not delete file"); // Delete the file and panic if the operation fails
    }
    // Deletes the directory represented by `self` and all of its contents.
    fn delete_directory(&self) {
        // `fs::remove_dir_all` attempts to remove the directory and its contents.
        // If it fails, the program will panic with the provided error message.
        fs::remove_dir_all(self).expect("could not delete directory");
    }

    // Attempts to delete the directory represented by `self` and returns a `Result` indicating success or failure.
    fn delete_directory_result(&self) -> Result<(), String> {
        // Try to remove the directory and capture the result.
        let res = fs::remove_dir_all(self);

        // Match on the result to return an appropriate `Result` type.
        return match res {
            Ok(_) => { Ok(()) } // Directory successfully deleted.
            Err(e) => { Err(e.to_string()) } // Return the error as a string.
        }
    }

    // Deletes all items in the directory represented by `self` and then recreates the directory.
    fn delete_all_items_in_directory(&self) {
        // Call `delete_directory` to remove the directory and its contents.
        self.delete_directory();
        // Call `create_directory` to recreate the directory.
        self.create_directory();
    }

    // Copies the file represented by `self` to a destination file path.
    fn copy_file_to_destination_file_path(&self, destination: &Self) {
        // Ensure `self` is a file before proceeding.
        assert!(self.is_file(), "must be a file");

        // Uncomment this line to assert that the destination is a file as well.
        // assert!(destination.is_file(), "destination must be file path");

        // Check if the destination path exists, if not, create the parent directory.
        if !destination.exists() {
            let par = destination.parent().expect("error").to_path_buf();
            par.create_directory(); // Ensure the parent directory exists.
        }

        // Attempt to copy the file to the destination and panic if it fails.
        fs::copy(self, destination).expect("could not copy");
    }

    // Copies the file represented by `self` to a destination directory, preserving the original file name.
    fn copy_file_to_destination_directory(&self, destination: &Self) {
        // Ensure `self` is a file before proceeding.
        assert!(self.is_file(), "must be a file");

        // Uncomment this line to assert that the destination is a directory.
        // assert!(destination.is_dir(), "destination must be directory");

        // Extract the file name from `self`.
        let f = self.file_name().unwrap().to_str().unwrap();
        // Construct the full destination path by appending the file name to the destination directory.
        let d = destination.clone().append(f);

        // Copy the file to the constructed destination file path.
        self.copy_file_to_destination_file_path(&d);
    }

    // Extracts the last `n` segments of the path represented by `self` and returns a new path with those segments.
    fn extract_last_n_segments(&self, n: usize) -> Self {
        // Ensure `n` is greater than 0.
        assert!(n > 0);

        // Split the path into segments (PathBuf objects).
        let s = self.split_into_path_bufs();

        // Use the smaller of `n` or the number of segments in the path.
        let n = n.min(s.len());

        // Create a new `PathBuf` to hold the result.
        let mut out = PathBuf::new();
        // Iterate over the last `n` segments and append them to `out`.
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
        // Create an empty vector to store the results.
        let mut out = vec![];

        // Try to read the directory contents.
        let res = self.read_dir();
        // If reading the directory was successful.
        if let Ok(read_dir) = res {
            // Iterate over the directory entries.
            for dir_entry_res in read_dir {
                // If reading the directory entry was successful.
                if let Ok(dir_entry) = dir_entry_res {
                    // Get the file name of the entry.
                    let filename = dir_entry.file_name();
                    // Convert the file name to a string.
                    let f = filename.to_str().unwrap().to_string();

                    // If directories are to be included and the entry is a directory.
                    if include_directories && dir_entry.path().is_dir() {
                        // If hidden directories are to be included.
                        if include_hidden_directories {
                            out.push(dir_entry.path());
                        } else {
                            // If not hidden, only include if the name does not start with a dot.
                            if !(f.chars().nth(0).unwrap().to_string() == ".") {
                                out.push(dir_entry.path());
                            }
                        }
                    }
                    // If files are to be included and the entry is a file.
                    else if include_files && dir_entry.path().is_file() {
                        // If hidden files are to be included.
                        if include_hidden_files {
                            out.push(dir_entry.path());
                        } else {
                            // If not hidden, only include if the name does not start with a dot.
                            if !(f.chars().nth(0).unwrap().to_string() == ".") {
                                out.push(dir_entry.path());
                            }
                        }
                    }
                }
            }
        }

        // Return the vector of paths that match the criteria.
        out
    }

    fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String> {
        // Create an empty vector to store the filenames.
        let mut out = vec![];
        // Get all items in the directory, including files based on the `include_hidden_files` flag.
        let items = self.get_all_items_in_directory(false, false, true, include_hidden_files);

        // Iterate over the items and collect their filenames into the `out` vector.
        items.iter().for_each(|x| {
            // Extract the file name, convert it to a string, and add it to the vector.
            out.push(x.file_name().expect("error").to_str().expect("error").to_string())
        });

        // Return the vector of filenames.
        out
    }

    fn read_file_contents_to_string(&self) -> String {
        // Attempt to open the file represented by `self`.
        let mut file_res = File::open(self);
        return match &mut file_res {
            // If file opening was successful.
            Ok(f) => {
                // Create a string to store the file contents.
                let mut contents = String::new();
                // Attempt to read the file contents into the string.
                let res = f.read_to_string(&mut contents);
                // If reading the file failed, panic with an error message.
                if res.is_err() {
                    panic!("could not load file with path {:?}.  Error: {:?}", self, res.err());
                }
                // Return the file contents.
                contents
            }
            // If file opening failed, panic with an error message.
            Err(e) => {
                panic!("could not load file with path {:?}.  Error: {}", self, e);
            }
        }
    }

    fn read_file_contents_to_string_result(&self) -> Result<String, String> {
        // Attempt to open the file represented by `self`.
        let mut file_res = File::open(self);
        return match &mut file_res {
            // If file opening was successful.
            Ok(f) => {
                // Create a string to store the file contents.
                let mut contents = String::new();
                // Attempt to read the file contents into the string.
                let res = f.read_to_string(&mut contents);
                // Return a `Result` based on whether reading the file was successful.
                match res {
                    Ok(_) => { Ok(contents) } // Return the file contents if successful.
                    Err(e) => { Err(format!("could not load file with path {:?}.  Error: {:?}", self, e)) } // Return an error message if reading failed.
                }
            }
            // If file opening failed, return an error message.
            Err(e) => {
                Err(format!("could not load file with path {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn write_string_to_file(&self, s: &String) {
        // Get the parent directory of the file path.
        let parent_option = self.parent();
        match parent_option {
            // If the parent directory is not found, panic with an error message.
            None => {
                panic!("Could not get parent of path in save_object_to_file_as_json.");
            }
            // If the parent directory is found, create it and any necessary parent directories.
            Some(parent) => {
                fs::create_dir_all(parent).expect("error");
            }
        }

        // If the file already exists, remove it.
        if self.exists() { fs::remove_file(self).expect("error"); }

        // Open the file for writing, creating it if it does not exist.
        let mut file_res = OpenOptions::new()
            .write(true)
            .create(true)
            .open(self);

        match &mut file_res {
            // If opening the file for writing was successful.
            Ok(f) => {
                // Write the string to the file.
                f.write(s.as_bytes()).expect("error");
            }
            // If opening the file for writing failed, panic with an error message.
            Err(e) => {
                panic!("could not write string to file.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_json_file<T: Serialize + DeserializeOwned>(&self) -> T {
        // Ensure the file extension is "json".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "json");

        // Read the file contents into a string.
        let json_str = self.read_file_contents_to_string();
        // Attempt to deserialize the JSON string into an object of type T.
        let o_res = serde_json::from_str(&json_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                o
            }
            // Panic with an error message if deserialization fails.
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_json_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        // Ensure the file extension is "json".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "json");

        // Read the file contents into a string and handle potential errors.
        let json_str = self.read_file_contents_to_string_result()?;
        // Attempt to deserialize the JSON string into an object of type T.
        let o_res = serde_json::from_str(&json_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                Ok(o)
            }
            // Return an error message if deserialization fails.
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_json_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        // Ensure the file extension is "json".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "json");

        // Serialize the object to a JSON string.
        let s = object.to_json_string();
        // Write the JSON string to the file.
        self.write_string_to_file(&s);
    }

    fn load_object_from_toml_file<T: Serialize + DeserializeOwned>(&self) -> T {
        // Ensure the file extension is "toml".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "toml");

        // Read the file contents into a string.
        let toml_str = self.read_file_contents_to_string();
        // Attempt to deserialize the TOML string into an object of type T.
        let o_res = toml::from_str(&toml_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                o
            }
            // Panic with an error message if deserialization fails.
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_toml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        // Ensure the file extension is "toml".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "toml");

        // Read the file contents into a string and handle potential errors.
        let toml_str = self.read_file_contents_to_string_result()?;
        // Attempt to deserialize the TOML string into an object of type T.
        let o_res = toml::from_str(&toml_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                Ok(o)
            }
            // Return an error message if deserialization fails.
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_toml_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        // Ensure the file extension is "toml".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "toml");

        // Serialize the object to a TOML string.
        let s = object.to_toml_string();
        // Write the TOML string to the file.
        self.write_string_to_file(&s);
    }

    fn load_object_from_ron_file<T: Serialize + DeserializeOwned>(&self) -> T {
        // Ensure the file extension is "ron".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "ron");

        // Read the file contents into a string.
        let ron_str = self.read_file_contents_to_string();
        // Attempt to deserialize the RON string into an object of type T.
        let o_res = ron::from_str(&ron_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                o
            }
            // Panic with an error message if deserialization fails.
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_ron_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        // Ensure the file extension is "ron".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "ron");

        // Read the file contents into a string and handle potential errors.
        let ron_str = self.read_file_contents_to_string_result()?;
        // Attempt to deserialize the RON string into an object of type T.
        let o_res = ron::from_str(&ron_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                Ok(o)
            }
            // Return an error message if deserialization fails.
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_ron_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        // Ensure the file extension is "ron".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "ron");

        // Serialize the object to a RON string.
        let s = object.to_ron_string();
        // Write the RON string to the file.
        self.write_string_to_file(&s);
    }

    fn load_object_from_yaml_file<T: Serialize + DeserializeOwned>(&self) -> T {
        // Ensure the file extension is "yaml".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "yaml");

        // Read the file contents into a string.
        let yaml_str = self.read_file_contents_to_string();
        // Attempt to deserialize the YAML string into an object of type T.
        let o_res = serde_yaml::from_str(&yaml_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                o
            }
            // Panic with an error message if deserialization fails.
            Err(e) => {
                panic!("could not load object.  Error: {:?}", e);
            }
        }
    }

    fn load_object_from_yaml_file_result<T: Serialize + DeserializeOwned>(&self) -> Result<T, String> {
        // Ensure the file extension is "yaml".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "yaml");

        // Read the file contents into a string and handle potential errors.
        let yaml_str = self.read_file_contents_to_string_result()?;
        // Attempt to deserialize the YAML string into an object of type T.
        let o_res = serde_yaml::from_str(&yaml_str);
        return match o_res {
            // Return the deserialized object if successful.
            Ok(o) => {
                Ok(o)
            }
            // Return an error message if deserialization fails.
            Err(e) => {
                Err(format!("Error loading object from file {:?}.  Error: {:?}", self, e))
            }
        }
    }

    fn save_object_to_yaml_file<T: Serialize + DeserializeOwned>(&self, object: &T) {
        // Ensure the file extension is "yaml".
        assert_eq!(self.extension().unwrap().to_str().unwrap(), "yaml");

        // Serialize the object to a YAML string.
        let s = object.to_yaml_string();
        // Write the YAML string to the file.
        self.write_string_to_file(&s);
    }

    fn get_file_for_writing(&self) -> File {
        // Get the parent directory of the file path.
        let prefix = self.parent().unwrap();
        // Create the directory and any necessary parent directories.
        fs::create_dir_all(prefix).unwrap();
        // If the file already exists, remove it.
        if self.exists() { fs::remove_file(self).expect("error"); }
        // Open the file for writing, creating it if it does not exist.
        let file = OpenOptions::new().write(true).create_new(true).open(self).expect("error");
        file
    }

    fn get_file_for_reading(&self) -> File {
        // Open the file for reading.
        let file = OpenOptions::new().read(true).open(self).expect("error");
        file
    }

    fn verify_extension(&self, extensions: &Vec<&str>) -> Result<(), String> {
        // Get the file extension of the path.
        let ext_option = self.extension();
        match ext_option {
            // If the path does not have an extension, return an error.
            None => {
                return Err(format!("Path {:?} does not have one of the following extensions: {:?}", self, extensions))
            }
            // If the path has an extension, check if it matches one of the allowed extensions.
            Some(ext) => {
                for e in extensions {
                    if e == &ext {
                        return Ok(()); // Return Ok if the extension matches.
                    }
                }
            }
        }
        // Return an error if the extension does not match.
        Err(format!("Path {:?} does not have one of the following extensions: {:?}", self, extensions))
    }

    fn get_a_to_b_path(&self, b: &PathBuf) -> PathBuf {
        // Clone the current path.
        let mut out = self.clone();

        // Split the current path into components.
        let s = self.split_into_path_bufs();
        let l = s.len();
        // Move up one directory level for each component except the last.
        for _ in 0..l-1 {
            out = out.clone().append("..");
        }

        // Append the target path to the base path and return it.
        return out.append_path(b);
    }
}