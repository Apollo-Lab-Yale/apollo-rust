use std::path::PathBuf;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ApolloOriginalMeshesModule {
    pub link_mesh_relative_paths: Vec<Option<PathBuf>>
}