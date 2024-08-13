use apollo_rust_file::ApolloPathBufTrait;
use crate::{create_generic_build_from_adjusted_robot2, create_generic_build_from_combined_robot2, create_generic_build_raw, PreprocessorModule};
use apollo_rust_mesh_utils::collada::load_dae_file;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::obj::load_obj_file;
use crate::utils::progress_bar::ProgressBarWrapper;
use apollo_rust_mesh_utils::stl::load_stl_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;
use apollo_rust_robot_modules::ResourcesSubDirectory;
use crate::robot_modules_preprocessor::CombinedRobot;
use crate::robot_modules_preprocessor::AdjustedRobot;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robot_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;

pub trait PlainMeshesModuleBuilders: Sized {
    fn build_from_original_meshes_module(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
    fn build_from_combined_robot(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
    fn build_from_adjusted_robot(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
}
impl PlainMeshesModuleBuilders for ApolloPlainMeshesModule {
    fn build_from_original_meshes_module(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let original_meshes_module = ApolloOriginalMeshesModule::load_or_build(s, false);

        if let Ok(original_meshes_module) = original_meshes_module {
            let mut stl_link_mesh_relative_paths = vec![];
            let mut obj_link_mesh_relative_paths = vec![];
            let mut glb_link_mesh_relative_paths = vec![];

            let paths = &original_meshes_module.link_mesh_relative_paths;
            let num_paths = paths.len();
            for (i, path_option) in paths.iter().enumerate() {
                let progress = i as f64 / num_paths as f64;
                progress_bar.update_with_percentage_preset(progress * 100.0);
                match path_option {
                    None => {
                        stl_link_mesh_relative_paths.push(None);
                        obj_link_mesh_relative_paths.push(None);
                        glb_link_mesh_relative_paths.push(None);
                    }
                    Some(path) => {
                        let file_stem = path.file_stem().expect("error").to_str().unwrap().to_string();
                        let stl_filename = file_stem.clone() + ".stl";
                        let obj_filename = file_stem.clone() + ".obj";
                        let glb_filename = file_stem + ".glb";

                        let stl_relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes/stl").append(&stl_filename);
                        let obj_relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes/obj").append(&obj_filename);
                        let glb_relative_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes/glb").append(&glb_filename);

                        let stl_target = Self::full_path_to_module_dir(s).append("meshes/stl").append(&stl_filename);
                        let obj_target = Self::full_path_to_module_dir(s).append("meshes/obj").append(&obj_filename);
                        let glb_target = Self::full_path_to_module_dir(s).append("meshes/glb").append(&glb_filename);

                        // if stl_target.exists() && obj_target.exists() && glb_target.exists() {
                        //     stl_link_mesh_relative_paths.push(Some(stl_relative_path));
                        //     obj_link_mesh_relative_paths.push(Some(obj_relative_path));
                        //     glb_link_mesh_relative_paths.push(Some(glb_relative_path));
                        //     continue 'l;
                        // }

                        let full_path = s.root_directory().clone().append_path(path);
                        let ext = path.extension().expect("must have extension").to_str().unwrap().to_string();
                        let trimesh_option = if ext == "stl" || ext == "STL" {
                            Some(load_stl_file(&full_path).expect("error").to_trimesh())
                        } else if ext == "dae" || ext == "DAE" {
                            Some(load_dae_file(&full_path).expect("error").to_trimesh())
                        } else if ext == "obj" || ext == "OBJ" {
                            Some(load_obj_file(&full_path).expect("error").to_trimesh())
                        } else if ext == "glb" || ext == "GLB" || ext == "gltf" || ext == "GLTF" {
                            Some(load_gltf_file(&full_path).expect("error").to_trimesh())
                        } else {
                            None
                        };

                        match trimesh_option {
                            None => {
                                println!("WARNING: mesh file with extension {:?} is not handled yet.  Not generating an stl.", ext);
                                stl_link_mesh_relative_paths.push(None);
                                obj_link_mesh_relative_paths.push(None);
                                glb_link_mesh_relative_paths.push(None);
                            }
                            Some(trimesh) => {
                                trimesh.save_to_stl(&stl_target);
                                stl_link_mesh_relative_paths.push(Some(stl_relative_path));

                                trimesh.save_to_obj(&obj_target);
                                obj_link_mesh_relative_paths.push(Some(obj_relative_path));

                                trimesh.save_to_glb(&glb_target);
                                glb_link_mesh_relative_paths.push(Some(glb_relative_path));
                            }
                        }
                    }
                }
            }

            progress_bar.done_preset();

            Ok(Self {
                stl_link_mesh_relative_paths,
                obj_link_mesh_relative_paths,
                glb_link_mesh_relative_paths
            })
        } else {
            Err(format!("could not build StlMeshesModule in {:?} because OriginalMeshesModule could not be loaded or built.", s.directory()))
        }
    }

    create_generic_build_from_combined_robot2!(ApolloPlainMeshesModule, None);

    create_generic_build_from_adjusted_robot2!(ApolloPlainMeshesModule);
}

impl PreprocessorModule for ApolloPlainMeshesModule {
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "mesh_modules/plain_meshes_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    create_generic_build_raw!(Self, build_from_original_meshes_module);
}

