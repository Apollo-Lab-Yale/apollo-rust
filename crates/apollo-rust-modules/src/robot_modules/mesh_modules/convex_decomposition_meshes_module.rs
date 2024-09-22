use std::path::PathBuf;
use serde::{Deserialize, Serialize};

/// # ApolloConvexDecompositionMeshesModule
///
/// This struct represents a module for handling different 3D mesh formats
/// and their respective file paths for convex decomposition in the Apollo
/// Toolbox. It supports STL, OBJ, and GLB formats, each represented by
/// relative paths organized in a two-dimensional vector.
///
/// The fields store the relative paths of the mesh files, categorized by
/// format.
///
/// ## Fields:
///
/// - `stl_link_mesh_relative_paths`: A 2D vector of `PathBuf` storing the
///    relative paths to STL mesh files.
/// - `obj_link_mesh_relative_paths`: A 2D vector of `PathBuf` storing the
///    relative paths to OBJ mesh files.
/// - `glb_link_mesh_relative_paths`: A 2D vector of `PathBuf` storing the
///    relative paths to GLB mesh files.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ApolloConvexDecompositionMeshesModule {
    pub stl_link_mesh_relative_paths: Vec<Vec<PathBuf>>,
    pub obj_link_mesh_relative_paths: Vec<Vec<PathBuf>>,
    pub glb_link_mesh_relative_paths: Vec<Vec<PathBuf>>
}
