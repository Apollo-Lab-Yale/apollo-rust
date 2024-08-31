use std::path::PathBuf;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use crate::modules::mesh_modules::recover_full_paths_from_relative_paths;

pub trait ConvexHullMeshesModuleGetFullPaths {
    fn get_stl_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;

    fn get_obj_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;

    fn get_glb_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;
}
impl ConvexHullMeshesModuleGetFullPaths for ApolloConvexHullMeshesModule {
    fn get_stl_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.stl_link_mesh_relative_paths)
    }

    fn get_obj_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.obj_link_mesh_relative_paths)
    }

    fn get_glb_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.glb_link_mesh_relative_paths)
    }
}