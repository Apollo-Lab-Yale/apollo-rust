use std::path::PathBuf;
use serde::de::DeserializeOwned;
use serde::Serialize;
use apollo_rust_file::ApolloPathBufTrait;
use crate::utils::progress_bar::ProgressBarWrapper;

pub mod utils;

#[derive(Clone, Debug)]
pub struct RobotPreprocessorRobotsDirectory {
    directory: PathBuf
}
impl RobotPreprocessorRobotsDirectory {
    pub fn new(directory: PathBuf) -> Self {
        assert!(directory.exists());

        RobotPreprocessorRobotsDirectory {
            directory
        }
    }
    pub fn directory(&self) -> &PathBuf {
        &self.directory
    }
    pub fn get_robot_subdirectory(&self, robot_name: &str) -> RobotPreprocessorSingleRobotDirectory {
        let directory = self.directory.clone().append(robot_name);
        assert!(directory.exists(), "{}", format!("directory {:?} does not exist", directory));
        RobotPreprocessorSingleRobotDirectory { robot_name: robot_name.to_string(), robots_directory: self.directory.clone(), directory }
    }
    pub fn get_all_robot_subdirectories(&self) -> Vec<RobotPreprocessorSingleRobotDirectory> {
        let mut out = vec![];

        let items = self.directory().get_all_items_in_directory(true, false, false, false);
        items.iter().for_each(|x| {
            let robot_name = x.iter().last().unwrap().to_str().unwrap().to_string();
            out.push(RobotPreprocessorSingleRobotDirectory { robot_name, robots_directory: self.directory.clone(), directory: x.clone() })
        });

        out
    }

    /*
    pub fn to_robot(&self, robot_name: &str) -> Robot {
        Robot::new_from_root(self, robot_name)
    }
    */

    /*
    pub fn preprocess_all(&self, force_build_on_all: bool) {
        for x in self.get_all_robot_subdirectories() {
            x.preprocess(force_build_on_all);
        }
    }
    */
}

#[derive(Clone, Debug)]
pub struct RobotPreprocessorSingleRobotDirectory {
    robot_name: String,
    robots_directory: PathBuf,
    directory: PathBuf
}
impl RobotPreprocessorSingleRobotDirectory {
    /*
    pub fn preprocess(&self, force_build_on_all: bool) {
        ApolloURDFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloDOFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloChainModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConnectionsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloOriginalMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloPlainMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexHullMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexDecompositionMeshesModule::load_or_build(self, force_build_on_all).expect("error");
    }
    */

    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }

    pub fn robots_directory(&self) -> &PathBuf {
        &self.robots_directory
    }

    pub fn directory(&self) -> &PathBuf {
        &self.directory
    }
}

pub trait RobotPreprocessorModule: Serialize + DeserializeOwned {

    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String;

    fn relative_file_path_from_root_dir_to_module_dir(s: &RobotPreprocessorSingleRobotDirectory) -> PathBuf {
        PathBuf::new().append(&s.robot_name).append(&Self::relative_file_path_str_from_robot_sub_dir_to_module_dir())
    }

    fn full_path_to_module_dir(s: &RobotPreprocessorSingleRobotDirectory) -> PathBuf {
        s.directory.clone().append(&Self::relative_file_path_str_from_robot_sub_dir_to_module_dir())
    }

    fn full_path_to_module_version(s: &RobotPreprocessorSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("VERSION")
    }

    fn full_path_module_json(s: &RobotPreprocessorSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.json")
    }

    fn full_path_module_ron(s: &RobotPreprocessorSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.ron")
    }

    fn full_path_module_yaml(s: &RobotPreprocessorSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.yaml")
    }

    /// should be in the format "0.0.1"
    fn current_version() -> String;

    fn build_raw(s: &RobotPreprocessorSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;

    fn build(s: &RobotPreprocessorSingleRobotDirectory) -> Result<Self, String> {
        let mut pb = ProgressBarWrapper::new(&s.robot_name, &Self::relative_file_path_str_from_robot_sub_dir_to_module_dir());
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

    fn load_from_json(s: &RobotPreprocessorSingleRobotDirectory) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_json(s).load_object_from_json_file_result()
    }

    fn load_from_ron(s: &RobotPreprocessorSingleRobotDirectory) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_ron(s).load_object_from_ron_file_result()
    }

    fn load_from_yaml(s: &RobotPreprocessorSingleRobotDirectory) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_yaml(s).load_object_from_yaml_file_result()
    }

    fn save(&self, s: &RobotPreprocessorSingleRobotDirectory) {
        Self::full_path_to_module_version(s).write_string_to_file(&Self::current_version());
        Self::full_path_module_json(s).save_object_to_json_file(self);
        Self::full_path_module_ron(s).save_object_to_ron_file(self);
        Self::full_path_module_yaml(s).save_object_to_yaml_file(self);
    }

    fn load_or_build(s: &RobotPreprocessorSingleRobotDirectory, force_build: bool) -> Result<Self, String> {
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
                        println!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}.  I will rebuild this module.", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version());
                    }
                }
                false => { }
            }
        }

        return Self::build(s);
    }
}
