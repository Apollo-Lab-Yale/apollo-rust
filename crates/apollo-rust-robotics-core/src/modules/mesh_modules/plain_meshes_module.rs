use std::path::PathBuf;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use crate::modules::mesh_modules::recover_full_paths_from_relative_paths;

pub trait PlainMeshesModuleGetFullPaths {
    fn get_stl_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;

    fn get_obj_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;

    fn get_glb_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;
}
impl PlainMeshesModuleGetFullPaths for ApolloPlainMeshesModule {
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