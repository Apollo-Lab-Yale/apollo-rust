use std::path::PathBuf;
use serde::{Deserialize, Serialize};

/// # ApolloOriginalMeshesModule
///
/// This struct holds optional relative paths for original mesh files.
/// The paths are stored as a vector of `Option<PathBuf>`, allowing for the possibility of missing paths.
///
/// ## Fields:
///
/// - `link_mesh_relative_paths`: A vector of optional `PathBuf` representing relative paths to original mesh files.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ApolloOriginalMeshesModule {
    pub link_mesh_relative_paths: Vec<Vec<PathBuf>>
}
