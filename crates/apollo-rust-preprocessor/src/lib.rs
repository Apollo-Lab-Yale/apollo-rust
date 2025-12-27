pub mod process_functions;
pub mod robot_modules_preprocessor;
pub mod standalone_preprocessor;
pub mod utils;

use std::path::PathBuf;
use serde::de::DeserializeOwned;
use serde::Serialize;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_modules::{ResourcesRootDirectory, ResourcesSubDirectory};
use apollo_rust_modules::robot_modules::bevy_modules::first_look_vis_module::ApolloFirstLookVisModule;
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::ApolloLinkShapesDistanceStatisticsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use crate::utils::progress_bar::ProgressBarWrapper;

/*
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
 */

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait PreprocessorModule<P = PathBuf>: Serialize + DeserializeOwned
where
    P: ApolloPathBufTrait + Clone,
{
    // type SubDirectoryType : ResourcesSubDirectoryTrait;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String;

    fn relative_file_path_from_root_dir_to_module_dir(s: &ResourcesSubDirectory<P>) -> P {
        P::new_from_str(&s.name).append(&Self::relative_file_path_str_from_sub_dir_to_module_dir())
    }

    fn full_path_to_module_dir(s: &ResourcesSubDirectory<P>) -> P {
        s.directory
            .clone()
            .append(&Self::relative_file_path_str_from_sub_dir_to_module_dir())
    }

    fn full_path_to_module_version(s: &ResourcesSubDirectory<P>) -> P {
        Self::full_path_to_module_dir(s).append("VERSION")
    }

    fn full_path_module_json(s: &ResourcesSubDirectory<P>) -> P {
        Self::full_path_to_module_dir(s).append("module.json")
    }

    fn full_path_module_ron(s: &ResourcesSubDirectory<P>) -> P {
        Self::full_path_to_module_dir(s).append("module.ron")
    }

    fn full_path_module_yaml(s: &ResourcesSubDirectory<P>) -> P {
        Self::full_path_to_module_dir(s).append("module.yaml")
    }

    /// should be in the format "0.0.1"
    fn current_version() -> String;

    fn build_raw(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;

    fn build(s: &ResourcesSubDirectory<P>) -> Result<Self, String> {
        let mut pb = ProgressBarWrapper::new(
            &s.name(),
            &Self::relative_file_path_str_from_sub_dir_to_module_dir(),
        );
        let o = Self::build_raw(s, &mut pb);
        return match o {
            Ok(o) => {
                o.save(&s);
                Ok(o)
            }
            Err(e) => Err(e),
        };
    }

    fn load_from_json(s: &ResourcesSubDirectory<P>) -> Result<Self, String> {
        if !Self::full_path_to_module_version(s).path_exists() {
            return Err("Module version does not exist".to_string());
        }
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() {
            return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version()));
        }
        Self::full_path_module_json(s).load_object_from_json_file_result()
    }

    fn load_from_ron(s: &ResourcesSubDirectory<P>) -> Result<Self, String> {
        if !Self::full_path_to_module_version(s).path_exists() {
            return Err("Module version does not exist".to_string());
        }
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() {
            return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version()));
        }
        Self::full_path_module_ron(s).load_object_from_ron_file_result()
    }

    fn load_from_yaml(s: &ResourcesSubDirectory<P>) -> Result<Self, String> {
        if !Self::full_path_to_module_version(s).path_exists() {
            return Err("Module version does not exist".to_string());
        }
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() {
            return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version()));
        }
        Self::full_path_module_yaml(s).load_object_from_yaml_file_result()
    }

    fn save(&self, s: &ResourcesSubDirectory<P>) {
        Self::full_path_to_module_version(s).write_string_to_file(&Self::current_version());
        Self::full_path_module_json(s).save_object_to_json_file(self);
        Self::full_path_module_ron(s).save_object_to_ron_file(self);
        Self::full_path_module_yaml(s).save_object_to_yaml_file(self);
    }

    fn load_or_build(s: &ResourcesSubDirectory<P>, force_build: bool) -> Result<Self, String> {
        if !force_build {
            let fp = Self::full_path_to_module_version(s);
            match fp.path_exists() {
                true => {
                    let saved_version = fp.read_file_contents_to_string();
                    if saved_version == Self::current_version() {
                        let loaded = Self::load_from_json(s);
                        match loaded {
                            Ok(loaded) => {
                                return Ok(loaded);
                            }
                            Err(e) => {
                                println!("Unable to load module in {:?} because of this reason: {:?}.  Will rebuild.", Self::full_path_module_json(s), e);
                            }
                        }
                    } else {
                        println!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}.  I will rebuild this module.", Self::relative_file_path_str_from_sub_dir_to_module_dir(), saved_version, Self::current_version());
                    }
                }
                false => {}
            }
        }

        return Self::build(s);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ResourcesRootDirectoryTrait<P = PathBuf>
where
    P: ApolloPathBufTrait + Clone,
{
    fn preprocess_all_robots(&self, force_build_on_all: bool);
    fn preprocess_all_environments(&self, force_build_on_all: bool);
}
impl<P: ApolloPathBufTrait + Clone> ResourcesRootDirectoryTrait<P> for ResourcesRootDirectory<P> {
    fn preprocess_all_robots(&self, force_build_on_all: bool) {
        self.get_all_subdirectories().iter().for_each(|x| {
            x.preprocess_robot(force_build_on_all, true);
        });
    }

    fn preprocess_all_environments(&self, force_build_on_all: bool) {
        self.get_all_subdirectories().iter().for_each(|x| {
            x.preprocess_environment(force_build_on_all);
        });
    }
}

pub trait ResourcesSubDirectoryTrait<P = PathBuf>
where
    P: ApolloPathBufTrait + Clone,
{
    fn preprocess_robot(&self, force_build_on_all: bool, include_original_meshes_module: bool);
    fn preprocess_environment(&self, force_build_on_all: bool);
}
impl<P: ApolloPathBufTrait + Clone> ResourcesSubDirectoryTrait<P> for ResourcesSubDirectory<P> {
    fn preprocess_robot(&self, force_build_on_all: bool, include_original_meshes_module: bool) {
        ApolloURDFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloDOFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloChainModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConnectionsModule::load_or_build(self, force_build_on_all).expect("error");
        if include_original_meshes_module {
            ApolloOriginalMeshesModule::<P>::load_or_build(self, force_build_on_all)
                .expect("error");
        }
        ApolloPlainMeshesModule::<P>::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexHullMeshesModule::<P>::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexDecompositionMeshesModule::<P>::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloFirstLookVisModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloBoundsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesDistanceStatisticsModule::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloLinkShapesSimpleSkipsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesApproximationsModule::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloLinkShapesSkipsModule::load_or_build(self, force_build_on_all).expect("error");
    }

    fn preprocess_environment(&self, force_build_on_all: bool) {
        ApolloURDFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloDOFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloChainModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConnectionsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloOriginalMeshesModule::<P>::load_or_build(self, force_build_on_all).expect("error");
        ApolloPlainMeshesModule::<P>::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexHullMeshesModule::<P>::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexDecompositionMeshesModule::<P>::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloFirstLookVisModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloBoundsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesDistanceStatisticsModule::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloLinkShapesSimpleSkipsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesApproximationsModule::load_or_build(self, force_build_on_all)
            .expect("error");
        ApolloLinkShapesSkipsModule::load_or_build(self, force_build_on_all).expect("error");
    }
}
