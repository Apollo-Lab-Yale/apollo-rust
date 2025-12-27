use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// # ApolloConvexHullMeshesModule
///
/// This struct holds relative paths to various 3D mesh files used for convex hull meshes.
/// The file paths are stored as optional `PathBuf` types to account for potential missing paths.
///
/// ## Fields:
///
/// - `stl_link_mesh_relative_paths`: A vector of optional `PathBuf` for STL mesh files.
/// - `obj_link_mesh_relative_paths`: A vector of optional `PathBuf` for OBJ mesh files.
/// - `glb_link_mesh_relative_paths`: A vector of optional `PathBuf` for GLB mesh files.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ApolloConvexHullMeshesModule<P = PathBuf> {
    pub stl_link_mesh_relative_paths: Vec<Option<P>>,
    pub obj_link_mesh_relative_paths: Vec<Option<P>>,
    pub glb_link_mesh_relative_paths: Vec<Option<P>>,
}
