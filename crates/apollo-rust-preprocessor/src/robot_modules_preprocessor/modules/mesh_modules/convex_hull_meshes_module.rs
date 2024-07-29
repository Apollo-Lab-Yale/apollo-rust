use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use crate::{create_generic_build_from_adjusted_robot2, create_generic_build_from_combined_robot2, create_generic_build_raw, PreprocessorModule, ResourcesSubDirectoryTrait};
use crate::utils::progress_bar::ProgressBarWrapper;
use apollo_rust_mesh_utils::stl::load_stl_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;
use apollo_rust_robot_modules::ResourcesSingleRobotDirectory;
use crate::robot_modules_preprocessor::CombinedRobot;
use apollo_rust_robot_modules::ResourcesRobotsDirectory;
use crate::robot_modules_preprocessor::AdjustedRobot;
use crate::robot_modules_preprocessor::modules::mesh_modules::recover_full_paths_from_relative_paths;
use crate::ResourcesRootDirectoryTrait;

pub trait ConvexHullMeshesModuleBuilders: Sized {
    fn build_from_plain_meshes_module(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
    fn build_from_combined_robot(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
    fn build_from_adjusted_robot(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
}
impl ConvexHullMeshesModuleBuilders for ApolloConvexHullMeshesModule {
    fn build_from_plain_meshes_module(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let plain_meshes_module = ApolloPlainMeshesModule::load_or_build(s, false);

        return if let Ok(plain_meshes_module) = plain_meshes_module {
            let mut stl_link_mesh_relative_paths = vec![];
            let mut obj_link_mesh_relative_paths = vec![];
            let mut glb_link_mesh_relative_paths = vec![];

            let num_paths = plain_meshes_module.stl_link_mesh_relative_paths.len();

            for (i, rel_path) in plain_meshes_module.stl_link_mesh_relative_paths.iter().enumerate() {
                let progress = i as f64 / num_paths as f64;
                progress_bar.update_with_percentage_preset(progress * 100.0);

                match rel_path {
                    None => {
                        stl_link_mesh_relative_paths.push(None);
                        obj_link_mesh_relative_paths.push(None);
                        glb_link_mesh_relative_paths.push(None);
                    }
                    Some(rel_path) => {
                        let full_path = s.root_directory().clone().append_path(rel_path);
                        let stl = load_stl_file(&full_path).expect("error");
                        let trimesh = stl.to_trimesh();
                        let filestem = rel_path.file_stem().unwrap().to_str().unwrap().to_string();

                        let stl_relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes/stl").append(&filestem).append_without_separator(".stl");
                        let obj_relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes/obj").append(&filestem).append_without_separator(".obj");
                        let glb_relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes/glb").append(&filestem).append_without_separator(".glb");

                        let stl_target = Self::full_path_to_module_dir(s).append("meshes/stl").append(&filestem).append_without_separator(".stl");
                        let obj_target = Self::full_path_to_module_dir(s).append("meshes/obj").append(&filestem).append_without_separator(".obj");
                        let glb_target = Self::full_path_to_module_dir(s).append("meshes/glb").append(&filestem).append_without_separator(".glb");

                        let convex_hull = trimesh.to_convex_hull();

                        convex_hull.save_to_stl(&stl_target);
                        stl_link_mesh_relative_paths.push(Some(stl_relative_path));

                        convex_hull.save_to_obj(&obj_target);
                        obj_link_mesh_relative_paths.push(Some(obj_relative_path));

                        convex_hull.save_to_glb(&glb_target);
                        glb_link_mesh_relative_paths.push(Some(glb_relative_path));
                    }
                }
            }

            progress_bar.done_preset();
            Ok(Self {
                stl_link_mesh_relative_paths,
                obj_link_mesh_relative_paths,
                glb_link_mesh_relative_paths,
            })
        } else {
            Err(format!("could not build ConvexHullMeshesModule in {:?} because PlainMeshesModule could not be loaded or built.", s.directory()))
        }
    }

    create_generic_build_from_combined_robot2!(ApolloConvexHullMeshesModule, None);

    create_generic_build_from_adjusted_robot2!(ApolloConvexHullMeshesModule);
}

impl PreprocessorModule for ApolloConvexHullMeshesModule {
    type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String { "mesh_modules/convex_hull_meshes_module".to_string() }

    fn current_version() -> String { "0.0.1".to_string() }

    create_generic_build_raw!(Self, build_from_plain_meshes_module);
}

pub trait ConvexHullMeshesModuleGetFullPaths {
    fn get_stl_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>>;

    fn get_obj_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>>;

    fn get_glb_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>>;
}
impl ConvexHullMeshesModuleGetFullPaths for ApolloConvexHullMeshesModule {
    fn get_stl_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.stl_link_mesh_relative_paths)
    }

    fn get_obj_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.obj_link_mesh_relative_paths)
    }

    fn get_glb_full_paths(&self, s: &ResourcesSingleRobotDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.glb_link_mesh_relative_paths)
    }
}