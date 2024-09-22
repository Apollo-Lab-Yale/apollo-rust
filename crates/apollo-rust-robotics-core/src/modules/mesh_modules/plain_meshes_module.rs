use std::path::PathBuf;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use crate::modules::mesh_modules::recover_full_paths_from_relative_paths;

/// The `PlainMeshesModuleGetFullPaths` trait provides methods to retrieve the full file paths
/// for STL, OBJ, and GLB mesh files based on relative paths stored in the `ApolloPlainMeshesModule`.
pub trait PlainMeshesModuleGetFullPaths {
    /// Retrieves the full file paths for STL files by recovering paths from a vector of relative paths.
    ///
    /// # Arguments
    /// - `s`: A reference to the `ResourcesSubDirectory` containing the root directory for the resources.
    ///
    /// # Returns
    /// A `Vec<Option<PathBuf>>` containing the full paths to the STL files.
    fn get_stl_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;

    /// Retrieves the full file paths for OBJ files by recovering paths from a vector of relative paths.
    ///
    /// # Arguments
    /// - `s`: A reference to the `ResourcesSubDirectory` containing the root directory for the resources.
    ///
    /// # Returns
    /// A `Vec<Option<PathBuf>>` containing the full paths to the OBJ files.
    fn get_obj_full_paths(&self, s: &ResourcesSubDirectory) -> Vec<Option<PathBuf>>;

    /// Retrieves the full file paths for GLB files by recovering paths from a vector of relative paths.
    ///
    /// # Arguments
    /// - `s`: A reference to the `ResourcesSubDirectory` containing the root directory for the resources.
    ///
    /// # Returns
    /// A `Vec<Option<PathBuf>>` containing the full paths to the GLB files.
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