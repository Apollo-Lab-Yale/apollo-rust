use crate::robot_modules_preprocessor::AdjustedRobot;
use crate::robot_modules_preprocessor::CombinedRobot;
use crate::utils::progress_bar::ProgressBarWrapper;
use crate::{
    create_generic_build_from_adjusted_robot2, create_generic_build_from_combined_robot2,
    create_generic_build_raw, PreprocessorModule,
};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::stl::load_stl_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_modules::ResourcesSubDirectory;

pub trait ConvexDecompositionMeshesModuleBuilders<P: ApolloPathBufTrait + Clone>: Sized {
    fn build_from_plain_meshes_module(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
    fn build_from_combined_robot(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
    fn build_from_adjusted_robot(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
}
impl<P: ApolloPathBufTrait + Clone> ConvexDecompositionMeshesModuleBuilders<P>
    for ApolloConvexDecompositionMeshesModule<P>
{
    fn build_from_plain_meshes_module(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let plain_meshes_module = ApolloPlainMeshesModule::<P>::load_or_build(s, false);

        return if let Ok(plain_meshes_module) = plain_meshes_module {
            let mut stl_link_mesh_relative_paths = vec![];
            let mut obj_link_mesh_relative_paths = vec![];
            let mut glb_link_mesh_relative_paths = vec![];

            let num_paths = plain_meshes_module.stl_link_mesh_relative_paths.len();
            for (i, rel_path) in plain_meshes_module
                .stl_link_mesh_relative_paths
                .iter()
                .enumerate()
            {
                let progress = i as f64 / num_paths as f64;
                progress_bar.update_with_percentage_preset(progress * 100.0);

                match rel_path {
                    None => {
                        stl_link_mesh_relative_paths.push(vec![]);
                        obj_link_mesh_relative_paths.push(vec![]);
                        glb_link_mesh_relative_paths.push(vec![]);
                    }
                    Some(rel_path) => {
                        let full_path = s.root_directory.clone().append_another(rel_path);
                        let stl = load_stl_file(&full_path.to_path_buf()).expect("error");
                        let trimesh = stl.to_trimesh();
                        let filestem = rel_path
                            .extract_last_n_segments(1)
                            .split_into_strings()
                            .pop()
                            .unwrap();
                        let filestem = filestem.split('.').next().unwrap().to_string();
                        let trimeshes = trimesh.to_convex_decomposition(5);
                        let dir = Self::full_path_to_module_dir(s)
                            .append("meshes")
                            .append(&filestem);
                        if dir.path_exists() {
                            dir.delete_all_items_in_directory();
                        }

                        let mut stl_curr = vec![];
                        let mut obj_curr = vec![];
                        let mut glb_curr = vec![];

                        for (j, trimesh) in trimeshes.iter().enumerate() {
                            let stl_relative_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes/stl")
                                    .append(&filestem)
                                    .append(&format!("{}.stl", j));
                            let obj_relative_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes/obj")
                                    .append(&filestem)
                                    .append(&format!("{}.obj", j));
                            let glb_relative_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes/glb")
                                    .append(&filestem)
                                    .append(&format!("{}.glb", j));

                            let stl_target = Self::full_path_to_module_dir(s)
                                .append("meshes/stl")
                                .append(&filestem)
                                .append(&format!("{}.stl", j));
                            let obj_target = Self::full_path_to_module_dir(s)
                                .append("meshes/obj")
                                .append(&filestem)
                                .append(&format!("{}.obj", j));
                            let glb_target = Self::full_path_to_module_dir(s)
                                .append("meshes/glb")
                                .append(&filestem)
                                .append(&format!("{}.glb", j));

                            trimesh.save_to_stl(&stl_target.to_path_buf());
                            trimesh.save_to_obj(&obj_target.to_path_buf());
                            trimesh.save_to_glb(&glb_target.to_path_buf());

                            stl_curr.push(stl_relative_path);
                            obj_curr.push(obj_relative_path);
                            glb_curr.push(glb_relative_path);
                        }
                        stl_link_mesh_relative_paths.push(stl_curr);
                        obj_link_mesh_relative_paths.push(obj_curr);
                        glb_link_mesh_relative_paths.push(glb_curr);
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
            Err(format!("could not build ConvexDecompositionMeshesModule in {:?} because PlainMeshesModule could not be loaded or built.", s.directory))
        };
    }

    create_generic_build_from_combined_robot2!(ApolloConvexDecompositionMeshesModule, P, vec![]);

    create_generic_build_from_adjusted_robot2!(ApolloConvexDecompositionMeshesModule, P);
}

impl<P: ApolloPathBufTrait + Clone> PreprocessorModule<P>
    for ApolloConvexDecompositionMeshesModule<P>
{
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "mesh_modules/convex_decomposition_meshes_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    create_generic_build_raw!(
        ApolloConvexDecompositionMeshesModule,
        P,
        build_from_plain_meshes_module
    );
}
