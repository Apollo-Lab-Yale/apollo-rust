use std::path::PathBuf;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloPlainMeshesModule {
    pub stl_link_mesh_relative_paths: Vec<Option<PathBuf>>,
    pub obj_link_mesh_relative_paths: Vec<Option<PathBuf>>,
    pub glb_link_mesh_relative_paths: Vec<Option<PathBuf>>
}