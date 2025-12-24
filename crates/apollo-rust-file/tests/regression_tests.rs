use apollo_rust_file::ApolloPathBufTrait;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
struct TestStruct {
    a: i32,
    b: String,
    c: Vec<f64>,
}

impl Default for TestStruct {
    fn default() -> Self {
        Self {
            a: 42,
            b: "hello".to_string(),
            c: vec![1.0, 2.0, 3.0],
        }
    }
}

// Implement traits required by ApolloPathBufTrait methods (ToJsonString etc are in apollo-rust-file traits)
// But wait, the trait `ApolloPathBufTrait` uses `ToJsonString` etc which are in `crate::traits`.
// I need strict consistency with how the library expects these.
// Let's check `lib.rs` imports again. `use crate::traits::{ToJsonString, ToRonString, ToTomlString, ToYamlString};`
// These traits must be implemented for `TestStruct`.
// Let's see `traits.rs` in `apollo-rust-file` to know if I need to impl them manually or if there are blanket impls.
// I'll assume for now I might need to derive or impl them but likely they have blanket impls for Serialize.
// Let's double check traits.rs first.

#[test]
fn test_path_manipulation() {
    let p = PathBuf::from("/tmp");
    let p2 = p.clone().append("subdir");
    assert_eq!(p2.to_str().unwrap(), "/tmp/subdir");

    let p3 = p2.append("file.txt");
    assert_eq!(p3.to_str().unwrap(), "/tmp/subdir/file.txt");

    let parts = p3.split_into_strings();
    // /tmp/subdir/file.txt -> ["", "tmp", "subdir", "file.txt"] or similar depending on platform.
    // Let's just check length and containment.
    assert!(parts.len() > 0);
    assert!(parts.contains(&"subdir".to_string()));
    assert!(parts.contains(&"file.txt".to_string()));
}

#[test]
fn test_file_io() {
    let temp_dir = std::env::temp_dir().join("apollo_rust_file_test_io");
    let p = PathBuf::from(&temp_dir);
    if p.exists() {
        p.delete_all_items_in_directory();
    } else {
        p.create_directory();
    }

    let file_path = p.clone().append("test_file.txt");
    let content = "Hello, Apollo!";
    file_path.write_string_to_file(&content.to_string());

    assert!(file_path.exists());
    let read_content = file_path.read_file_contents_to_string();
    assert_eq!(read_content, content);

    p.delete_all_items_in_directory();
    p.delete_directory();
    assert!(!p.exists());
}

#[test]
fn test_serialization() {
    let temp_dir = std::env::temp_dir().join("apollo_rust_file_test_serde");
    let p = PathBuf::from(&temp_dir);
    if p.exists() {
        p.delete_all_items_in_directory();
    } else {
        p.create_directory();
    }

    let test_obj = TestStruct::default();

    // JSON
    let json_path = p.clone().append("test.json");
    json_path.save_object_to_json_file(&test_obj);
    let loaded_json: TestStruct = json_path.load_object_from_json_file();
    assert_eq!(test_obj, loaded_json);

    // TOML
    let toml_path = p.clone().append("test.toml");
    toml_path.save_object_to_toml_file(&test_obj);
    let loaded_toml: TestStruct = toml_path.load_object_from_toml_file();
    assert_eq!(test_obj, loaded_toml);

    // YAML
    let yaml_path = p.clone().append("test.yaml");
    yaml_path.save_object_to_yaml_file(&test_obj);
    let loaded_yaml: TestStruct = yaml_path.load_object_from_yaml_file();
    assert_eq!(test_obj, loaded_yaml);

    // RON
    let ron_path = p.clone().append("test.ron");
    ron_path.save_object_to_ron_file(&test_obj);
    let loaded_ron: TestStruct = ron_path.load_object_from_ron_file();
    assert_eq!(test_obj, loaded_ron);

    p.delete_all_items_in_directory();
    p.delete_directory();
}

#[test]
fn test_filesystem_ops() {
    let temp_dir = std::env::temp_dir().join("apollo_rust_file_test_fs");
    let p = PathBuf::from(&temp_dir);

    // Ensure clean state
    if p.exists() {
        p.delete_all_items_in_directory();
        p.delete_directory();
    }

    p.create_directory();
    assert!(p.exists());
    assert!(p.is_dir());

    let subdir = p.clone().append("subdir");
    subdir.create_directory();
    assert!(subdir.exists());
    assert!(subdir.is_dir());

    let file_in_subdir = subdir.clone().append("file.txt");
    file_in_subdir.write_string_to_file(&"content".to_string());
    assert!(file_in_subdir.exists());

    // Test delete_all_items_in_directory
    p.delete_all_items_in_directory();
    assert!(p.exists()); // The directory itself should still exist
    assert!(!subdir.exists()); // Subdir should be gone

    p.delete_directory();
    assert!(!p.exists());
}
