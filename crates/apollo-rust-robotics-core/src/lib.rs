use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;

/// The apollo-rust-robotics-core module contains robotics functions and structs that depend
/// only on robot modules, but without initializing them.  Structs in this crate are initialized
/// using separate traits in the apollo-rust-robotics crate.  In practice, you should be using
/// that crate for all robotics needs.  This crate structure helps avoid circular dependencies.

pub mod modules_runtime;
pub mod robot_functions;

#[derive(Clone, Debug)]
pub struct RobotPreprocessorRobotsDirectory {
    pub directory: PathBuf
}
impl RobotPreprocessorRobotsDirectory {
    pub fn new(directory: PathBuf) -> Self {
        assert!(directory.exists());

        RobotPreprocessorRobotsDirectory {
            directory
        }
    }
    pub fn new_default() -> Self {
        Self::new(PathBuf::new_from_default_apollo_robots_dir())
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
    pub robot_name: String,
    pub robots_directory: PathBuf,
    pub directory: PathBuf
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



