pub mod original_meshes_module;
pub mod plain_meshes_module;
pub mod convex_hull_meshes_module;
pub mod convex_decomposition_meshes_module;

use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesSingleRobotDirectory;
use crate::ResourcesSubDirectoryTrait;

pub fn recover_full_paths_from_relative_paths(s: &ResourcesSingleRobotDirectory, paths: &Vec<Option<PathBuf>>) -> Vec<Option<PathBuf>> {
    let root = s.root_directory().clone();

    let out = paths.iter().map(|x| {
        match x {
            None => { None }
            Some(y) => { Some(root.clone().append_path(y)) }
        }
    }).collect();

    out
}

pub fn recover_full_paths_from_double_vec_of_relative_paths(s: &ResourcesSingleRobotDirectory, paths: &Vec<Vec<PathBuf>>) -> Vec<Vec<PathBuf>> {
    let root = s.root_directory().clone();
    let out: Vec<Vec<PathBuf>> = paths.iter().map(|x| {
        let tmp: Vec<PathBuf> = x.iter().map(|y| {
            root.clone().append_path(y)
        }).collect();
        tmp
    }).collect();

    out
}

#[macro_export]
macro_rules! create_generic_build_from_combined_robot {
    ($module:ty, $initial_push:expr) => {
        fn build_from_combined_robot(
            s: &ResourcesSingleRobotDirectory,
            progress_bar: &mut ProgressBarWrapper,
        ) -> Result<Self, String> {
            let fp = s.directory.clone().append("combined_robot_module/module.json");
            let combined_robot = fp.load_object_from_json_file_result::<CombinedRobot>();
            return if let Ok(combined_robot) = combined_robot {
                let mut link_mesh_relative_paths = vec![];

                link_mesh_relative_paths.push($initial_push);
                combined_robot.attached_robots.iter().for_each(|x| {
                    let robot_name = x.robot_name.clone();
                    let ss = ResourcesRobotsDirectory::new(s.robots_directory.clone())
                        .get_subdirectory(&robot_name);
                    let module = <$module>::load_or_build(&ss, false).expect("error");
                    link_mesh_relative_paths.extend(module.link_mesh_relative_paths);
                });

                progress_bar.done_preset();
                Ok(Self {
                    link_mesh_relative_paths,
                })
            } else {
                Err("could not build from combined robot".to_string())
            }
        }
    };
}

#[macro_export]
macro_rules! create_generic_build_from_combined_robot2 {
    ($module:ty, $initial_push:expr) => {
        fn build_from_combined_robot(
            s: &ResourcesSingleRobotDirectory,
            progress_bar: &mut ProgressBarWrapper,
        ) -> Result<Self, String> {
            let fp = s.directory.clone().append("combined_robot_module/module.json");
            let combined_robot = fp.load_object_from_json_file_result::<CombinedRobot>();
            return if let Ok(combined_robot) = combined_robot {
                let mut stl_link_mesh_relative_paths = vec![];
                let mut obj_link_mesh_relative_paths = vec![];
                let mut glb_link_mesh_relative_paths = vec![];

                stl_link_mesh_relative_paths.push($initial_push);
                obj_link_mesh_relative_paths.push($initial_push);
                glb_link_mesh_relative_paths.push($initial_push);

                combined_robot.attached_robots.iter().for_each(|x| {
                    let robot_name = x.robot_name.clone();
                    let ss = ResourcesRobotsDirectory::new(s.robots_directory.clone())
                        .get_subdirectory(&robot_name);
                    let module = <$module>::load_or_build(&ss, false).expect("error");

                    stl_link_mesh_relative_paths.extend(module.stl_link_mesh_relative_paths);
                    obj_link_mesh_relative_paths.extend(module.obj_link_mesh_relative_paths);
                    glb_link_mesh_relative_paths.extend(module.glb_link_mesh_relative_paths);
                });

                progress_bar.done_preset();
                Ok(Self {
                    stl_link_mesh_relative_paths,
                    obj_link_mesh_relative_paths,
                    glb_link_mesh_relative_paths
                })
            } else {
                Err("could not build from combined robot".to_string())
            }
        }
    };
}

#[macro_export]
macro_rules! create_generic_build_from_adjusted_robot {
    ($module:ty) => {
        fn build_from_adjusted_robot(
            s: &ResourcesSingleRobotDirectory,
            progress_bar: &mut ProgressBarWrapper,
        ) -> Result<Self, String> {
            let fp = s.directory.clone().append("adjusted_robot_module/module.json");
            let adjusted_robot = fp.load_object_from_json_file_result::<AdjustedRobot>();
            return if let Ok(adjusted_robot) = adjusted_robot {
                let mut link_mesh_relative_paths = vec![];

                let ss = ResourcesRobotsDirectory::new(s.robots_directory.clone())
                    .get_subdirectory(&adjusted_robot.base_robot_name);
                let module = <$module>::load_or_build(&ss, false).expect("error");

                let adjusted_link_idx_to_base_link_idx_mapping = &adjusted_robot.adjusted_link_idx_to_base_link_idx_mapping;

                adjusted_link_idx_to_base_link_idx_mapping.iter().for_each(|idx| {
                    link_mesh_relative_paths.push(module.link_mesh_relative_paths[*idx].clone());
                    // stl_link_mesh_relative_paths.push(module.stl_link_mesh_relative_paths[*idx]);
                    // obj_link_mesh_relative_paths.push(module.obj_link_mesh_relative_paths[*idx]);
                    // glb_link_mesh_relative_paths.push(module.glb_link_mesh_relative_paths[*idx]);
                });

                // link_mesh_relative_paths.push($initial_push);
                /*
                combined_robot.attached_robots.iter().for_each(|x| {
                    let robot_name = x.robot_name.clone();
                    let ss = RobotPreprocessorRobotsDirectory::new(s.robots_directory.clone())
                        .get_robot_subdirectory(&robot_name);
                    let module = <$module>::load_or_build(&ss, false).expect("error");
                    link_mesh_relative_paths.extend(module.link_mesh_relative_paths);
                });
                */

                progress_bar.done_preset();
                Ok(Self {
                    link_mesh_relative_paths,
                })
            } else {
                Err("could not build from adjusted robot".to_string())
            }
        }
    };
}

#[macro_export]
macro_rules! create_generic_build_from_adjusted_robot2 {
        ($module:ty) => {
        fn build_from_adjusted_robot(
            s: &ResourcesSingleRobotDirectory,
            progress_bar: &mut ProgressBarWrapper,
        ) -> Result<Self, String> {
            let fp = s.directory.clone().append("adjusted_robot_module/module.json");
            let adjusted_robot = fp.load_object_from_json_file_result::<AdjustedRobot>();
            return if let Ok(adjusted_robot) = adjusted_robot {
                let mut stl_link_mesh_relative_paths = vec![];
                let mut obj_link_mesh_relative_paths = vec![];
                let mut glb_link_mesh_relative_paths = vec![];

                // stl_link_mesh_relative_paths.push($initial_push);
                // obj_link_mesh_relative_paths.push($initial_push);
                // glb_link_mesh_relative_paths.push($initial_push);

                let ss = ResourcesRobotsDirectory::new(s.robots_directory.clone())
                    .get_subdirectory(&adjusted_robot.base_robot_name);
                let module = <$module>::load_or_build(&ss, false).expect("error");

                let adjusted_link_idx_to_base_link_idx_mapping = &adjusted_robot.adjusted_link_idx_to_base_link_idx_mapping;

                adjusted_link_idx_to_base_link_idx_mapping.iter().for_each(|idx| {
                    stl_link_mesh_relative_paths.push(module.stl_link_mesh_relative_paths[*idx].clone());
                    obj_link_mesh_relative_paths.push(module.obj_link_mesh_relative_paths[*idx].clone());
                    glb_link_mesh_relative_paths.push(module.glb_link_mesh_relative_paths[*idx].clone());
                });

                /*
                combined_robot.attached_robots.iter().for_each(|x| {
                    let robot_name = x.robot_name.clone();
                    let ss = RobotPreprocessorRobotsDirectory::new(s.robots_directory.clone())
                        .get_robot_subdirectory(&robot_name);
                    let module = <$module>::load_or_build(&ss, false).expect("error");

                    stl_link_mesh_relative_paths.extend(module.stl_link_mesh_relative_paths);
                    obj_link_mesh_relative_paths.extend(module.obj_link_mesh_relative_paths);
                    glb_link_mesh_relative_paths.extend(module.glb_link_mesh_relative_paths);
                });
                */

                progress_bar.done_preset();
                Ok(Self {
                    stl_link_mesh_relative_paths,
                    obj_link_mesh_relative_paths,
                    glb_link_mesh_relative_paths
                })
            } else {
                Err("could not build from adjusted robot".to_string())
            }
        }
    };
}

#[macro_export]
macro_rules! create_generic_build_raw {
    ($self_ty:ty, $default_fn:ident) => {
        fn build_raw(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<$self_ty, String> {
            let res = <$self_ty>::build_from_combined_robot(s, progress_bar);
            if let Ok(res) = res {
                return Ok(res);
            }

            let res = <$self_ty>::build_from_adjusted_robot(s, progress_bar);
            if let Ok(res) = res {
                return Ok(res);
            }

            return <$self_ty>::$default_fn(s, progress_bar);
        }
    };
}