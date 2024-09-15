use std::path::PathBuf;
use serde::{Deserialize, Serialize};

/// # ApolloPlainMeshesModule
///
/// This struct stores optional relative paths for different types of plain 3D mesh files.
/// The paths are stored as vectors of `Option<PathBuf>`, allowing for potential missing paths.
///
/// ## Fields:
///
/// - `stl_link_mesh_relative_paths`: A vector of optional `PathBuf` for STL mesh files.
/// - `obj_link_mesh_relative_paths`: A vector of optional `PathBuf` for OBJ mesh files.
/// - `glb_link_mesh_relative_paths`: A vector of optional `PathBuf` for GLB mesh files.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloPlainMeshesModule {
    pub stl_link_mesh_relative_paths: Vec<Option<PathBuf>>,
    pub obj_link_mesh_relative_paths: Vec<Option<PathBuf>>,
    pub glb_link_mesh_relative_paths: Vec<Option<PathBuf>>
}
