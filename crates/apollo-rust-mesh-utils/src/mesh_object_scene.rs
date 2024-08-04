use serde::{Deserialize, Serialize};
use crate::mesh_object::MeshObject;

pub trait ToMeshObjectScene {
    fn to_mesh_object_scene(&self) -> MeshObjectScene;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshObjectScene {
    pub nodes: Vec<MeshObjectNode>
}
impl MeshObjectScene {
    pub fn to_mesh_objects(&self) -> Vec<MeshObject> {
        self.nodes.iter().map(|x| { x.local_space_mesh_object.clone() }).collect()
    }

    pub fn print_all_mesh_object_names(&self) {
        self.nodes.iter().enumerate().for_each(|(i, x)| {
            println!("{}: {:?}", i, x.name);
        });
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshObjectNode {
    pub name: String,
    pub parent_node: Option<String>,
    pub local_space_mesh_object: MeshObject,
}