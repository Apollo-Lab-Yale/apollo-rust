use std::path::PathBuf;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use crate::modules::mesh_modules::{recover_full_paths_from_double_vec_of_relative_paths};

/// The `OriginalMeshesModuleGetFullPaths` trait provides a method to retrieve the full file paths
/// for mesh files based on relative paths stored in the `ApolloOriginalMeshesModule`.
pub trait OriginalMeshesModuleGetFullPaths {
    /// Retrieves the full file paths by recovering paths from a vector of relative paths.
    ///
    /// # Arguments
    /// - `s`: A reference to the `ResourcesSubDirectory` containing the root directory for the resources.
    ///
    /// # Returns
    /// A `Vec<Option<PathBuf>>` containing the full paths to the mesh files.
    fn get_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>>;
}
impl OriginalMeshesModuleGetFullPaths for ApolloOriginalMeshesModule {

    fn get_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Vec<PathBuf>> {
        recover_full_paths_from_double_vec_of_relative_paths(s, &self.link_mesh_relative_paths)
    }
}