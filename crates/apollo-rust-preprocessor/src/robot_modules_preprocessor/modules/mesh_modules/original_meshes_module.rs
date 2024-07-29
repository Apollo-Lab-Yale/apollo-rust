use std::path::PathBuf;
use apollo_rust_robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use crate::{create_generic_build_from_adjusted_robot, create_generic_build_from_combined_robot, create_generic_build_raw, PreprocessorModule};
use crate::utils::progress_bar::ProgressBarWrapper;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesSingleRobotDirectory;
use apollo_rust_robot_modules::urdf_module::{ApolloURDFGeometry, ApolloURDFModule};
use crate::robot_modules_preprocessor::modules::mesh_modules::recover_full_paths_from_relative_paths;
use crate::robot_modules_preprocessor::CombinedRobot;
use crate::robot_modules_preprocessor::ResourcesRobotsDirectory;
use crate::robot_modules_preprocessor::AdjustedRobot;
use crate::ResourcesRootDirectoryTrait;

pub trait OriginalMeshesModuleBuilders: Sized {
    fn build_from_urdf_module(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
    fn build_from_combined_robot(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
    fn build_from_adjusted_robot(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
}
impl OriginalMeshesModuleBuilders for ApolloOriginalMeshesModule {
    fn build_from_urdf_module(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false);

        return if let Ok(urdf_module) = urdf_module {
            let mut out = vec![];

            let num_links = urdf_module.links.len();
            urdf_module.links.iter().enumerate().for_each(|(i, x)| {
                let progress = i as f64 / num_links as f64;
                progress_bar.update_with_percentage_preset(progress * 100.0);
                if x.visual.len() > 0 {
                    let visual = &x.visual[0];
                    match &visual.geometry {
                        ApolloURDFGeometry::Mesh { filename, scale: _ } => {
                            let fp = PathBuf::new().append_without_separator(filename);
                            let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                            let relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes").append(&filename);
                            let target = Self::full_path_to_module_dir(s).append("meshes").append(&filename);
                            // if !target.exists() {
                            let ff = fp.extract_last_n_segments(3);
                            println!("searching directory for file that ends with {:?}", ff);
                            let find = PathBuf::new_from_documents_dir().walk_directory_and_find_first(ff);
                            println!("found!  Copying file.");
                            find.copy_file_to_destination_file_path(&target);
                            // }
                            out.push(Some(relative_path));
                        }
                        _ => { out.push(None); }
                    }
                } else {
                    out.push(None);
                }
            });

            progress_bar.done_preset();
            Ok(ApolloOriginalMeshesModule {
                link_mesh_relative_paths: out,
            })
        } else {
            Err("could not build from urdf module".to_string())
        }
    }

    create_generic_build_from_combined_robot!(ApolloOriginalMeshesModule, None);

    create_generic_build_from_adjusted_robot!(ApolloOriginalMeshesModule);
}

impl PreprocessorModule for ApolloOriginalMeshesModule {
    type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "mesh_modules/original_meshes_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    create_generic_build_raw!(Self, build_from_urdf_module);
}

pub trait OriginalMeshesModuleGetFullPaths {
    fn get_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>>;
}
impl OriginalMeshesModuleGetFullPaths for ApolloOriginalMeshesModule {
    fn get_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.link_mesh_relative_paths)
    }
}