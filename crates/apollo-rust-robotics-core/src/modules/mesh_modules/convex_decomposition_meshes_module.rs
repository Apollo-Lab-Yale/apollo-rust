use std::path::PathBuf;
use apollo_rust_robot_modules::ResourcesSubDirectory;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use crate::modules::mesh_modules::recover_full_paths_from_double_vec_of_relative_paths;

pub trait ConvexDecompositionMeshesModuleGetFullPaths {
    fn get_stl_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>>;

    fn get_obj_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>>;

    fn get_glb_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>>;
}
impl ConvexDecompositionMeshesModuleGetFullPaths for ApolloConvexDecompositionMeshesModule {
    fn get_stl_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>> {
        recover_full_paths_from_double_vec_of_relative_paths(s, &self.stl_link_mesh_relative_paths)
    }

    fn get_obj_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>> {
        recover_full_paths_from_double_vec_of_relative_paths(s, &self.obj_link_mesh_relative_paths)
    }

    fn get_glb_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>> {
        recover_full_paths_from_double_vec_of_relative_paths(s, &self.glb_link_mesh_relative_paths)
    }
}