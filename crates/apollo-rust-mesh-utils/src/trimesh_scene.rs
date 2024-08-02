use nalgebra::Isometry3;
use crate::trimesh::TriMesh;

pub trait ToTrimeshScene {
    fn to_trimesh_scene(&self) -> TrimeshScene;
}

#[derive(Clone, Debug)]
pub struct TrimeshScene {
    pub nodes: Vec<TrimeshNode>
}

#[derive(Clone, Debug)]
pub struct TrimeshNode {
    pub name: String,
    pub parent_node: Option<String>,
    pub local_space_trimesh: TriMesh,
    pub offset_from_parent: Isometry3<f64>
}