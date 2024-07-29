pub mod utils;
pub mod robot_modules_preprocessor;
pub mod environment_modules_preprocessor;

use std::path::PathBuf;
use serde::de::DeserializeOwned;
use serde::Serialize;
use apollo_rust_file::ApolloPathBufTrait;
use crate::utils::progress_bar::ProgressBarWrapper;

pub trait ResourcesRootDirectoryTrait {
    type SubDirectoryType : ResourcesSubDirectoryTrait;

    fn new(directory: PathBuf) -> Self;
    fn new_default() -> Self;
    fn directory(&self) -> &PathBuf;
    fn get_subdirectory(&self, name: &str) -> Self::SubDirectoryType {
        let directory = self.directory().clone().append(name);
        assert!(directory.exists(), "{}", format!("directory {:?} does not exist", directory));
        <Self::SubDirectoryType as ResourcesSubDirectoryTrait>::new_raw(name.to_string(), self.directory().clone(), directory)
    }
    fn get_all_subdirectories(&self) -> Vec<Self::SubDirectoryType> {
        let mut out = vec![];

        let items = self.directory().get_all_items_in_directory(true, false, false, false);
        items.iter().for_each(|x| {
            let s = x.split_into_strings();
            let name = s.last().unwrap();
            out.push(self.get_subdirectory(name));
        });

        out
    }
}

pub trait ResourcesSubDirectoryTrait {
    fn new_raw(name: String, root_directory: PathBuf, directory: PathBuf) -> Self;
    fn name(&self) -> String;
    fn root_directory(&self) -> &PathBuf;
    fn directory(&self) -> &PathBuf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ResourcesRootDirectoryPreprocessorTrait {
    fn preprocess_all(&self, force_build_on_all: bool);
}

pub trait ResourcesSubDirectoryPreprocessorTrait {
    fn preprocess(&self, force_build_on_all: bool);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait PreprocessorModule : Serialize + DeserializeOwned {
    type SubDirectoryType : ResourcesSubDirectoryTrait;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String;

    fn relative_file_path_from_root_dir_to_module_dir(s: &Self::SubDirectoryType) -> PathBuf {
        PathBuf::new().append(&s.name()).append(&Self::relative_file_path_str_from_sub_dir_to_module_dir())
    }

    fn full_path_to_module_dir(s: &Self::SubDirectoryType) -> PathBuf {
        s.directory().clone().append(&Self::relative_file_path_str_from_sub_dir_to_module_dir())
    }

    fn full_path_to_module_version(s: &Self::SubDirectoryType) -> PathBuf {
        Self::full_path_to_module_dir(s).append("VERSION")
    }

    fn full_path_module_json(s: &Self::SubDirectoryType) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.json")
    }

    fn full_path_module_ron(s: &Self::SubDirectoryType) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.ron")
    }

    fn full_path_module_yaml(s: &Self::SubDirectoryType) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.yaml")
    }

    /// should be in the format "0.0.1"
    fn current_version() -> String;

    fn build_raw(s: &Self::SubDirectoryType, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;

    fn build(s: &Self::SubDirectoryType) -> Result<Self, String> {
        let mut pb = ProgressBarWrapper::new(&s.name(), &Self::relative_file_path_str_from_sub_dir_to_module_dir());
        let o = Self::build_raw(s, &mut pb);
        return match o {
            Ok(o) => {
                o.save(&s);
                Ok(o)
            }
            Err(e) => {
                Err(e)
            }
        }
    }

    fn load_from_json(s: &Self::SubDirectoryType) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_json(s).load_object_from_json_file_result()
    }

    fn load_from_ron(s: &Self::SubDirectoryType) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_ron(s).load_object_from_ron_file_result()
    }

    fn load_from_yaml(s: &Self::SubDirectoryType) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_yaml(s).load_object_from_yaml_file_result()
    }

    fn save(&self, s: &Self::SubDirectoryType) {
        Self::full_path_to_module_version(s).write_string_to_file(&Self::current_version());
        Self::full_path_module_json(s).save_object_to_json_file(self);
        Self::full_path_module_ron(s).save_object_to_ron_file(self);
        Self::full_path_module_yaml(s).save_object_to_yaml_file(self);
    }

    fn load_or_build(s: &Self::SubDirectoryType, force_build: bool) -> Result<Self, String> {
        if !force_build {
            let fp = Self::full_path_to_module_version(s);
            match fp.exists() {
                true => {
                    let saved_version = fp.read_file_contents_to_string();
                    if saved_version == Self::current_version() {
                        let loaded = Self::load_from_json(s);
                        match loaded {
                            Ok(loaded) => { return Ok(loaded); }
                            Err(e) => {
                                println!("Unable to load module in {:?} because of this reason: {:?}.  Will rebuild.", Self::full_path_module_json(s), e);
                            }
                        }
                    } else {
                        println!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}.  I will rebuild this module.", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version());
                    }
                }
                false => { }
            }
        }

        return Self::build(s);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

