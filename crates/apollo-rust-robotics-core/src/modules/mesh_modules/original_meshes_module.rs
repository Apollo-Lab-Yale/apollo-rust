use std::path::PathBuf;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use crate::modules::mesh_modules::recover_full_paths_from_relative_paths;

pub trait OriginalMeshesModuleGetFullPaths {
    fn get_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;
}
impl OriginalMeshesModuleGetFullPaths for ApolloOriginalMeshesModule {
    fn get_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>> {
        recover_full_paths_from_relative_paths(s, &self.link_mesh_relative_paths)
    }
}