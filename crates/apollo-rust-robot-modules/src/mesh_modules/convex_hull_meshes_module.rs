use std::path::PathBuf;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ApolloConvexHullMeshesModule {
    pub stl_link_mesh_relative_paths: Vec<Option<PathBuf>>,
    pub obj_link_mesh_relative_paths: Vec<Option<PathBuf>>,
    pub glb_link_mesh_relative_paths: Vec<Option<PathBuf>>
}