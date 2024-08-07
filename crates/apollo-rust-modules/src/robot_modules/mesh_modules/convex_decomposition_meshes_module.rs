use std::path::PathBuf;
use serde::{Deserialize, Serialize};


#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ApolloConvexDecompositionMeshesModule {
    pub stl_link_mesh_relative_paths: Vec<Vec<PathBuf>>,
    pub obj_link_mesh_relative_paths: Vec<Vec<PathBuf>>,
    pub glb_link_mesh_relative_paths: Vec<Vec<PathBuf>>
}