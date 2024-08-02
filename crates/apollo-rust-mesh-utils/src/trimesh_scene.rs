use nalgebra::{Isometry3, Vector3};
use crate::trimesh::TriMesh;

pub trait ToTrimeshScene {
    fn to_trimesh_scene(&self) -> TrimeshScene;
}

#[derive(Clone, Debug)]
pub struct TrimeshScene {
    pub nodes: Vec<TrimeshNode>
}
impl TrimeshScene {
    pub fn to_resolved_trimeshes(&self) -> Vec<TriMesh> {
        let mut out = vec![];

        let mut done = vec![false; self.nodes.len()];
        let names: Vec<String> = self.nodes.iter().map(|x| x.name.clone()).collect();
        let mut transforms = vec![Isometry3::identity(); self.nodes.len()];

        while !done.iter().all(|x| *x) {
            self.nodes.iter().enumerate().for_each(|(i, x)| {
                if (x.parent_node.is_none() && !done[i]) || (x.parent_node.is_some() && !done[i] && done[names.iter().position(|y| y == x.parent_node.as_ref().unwrap()).unwrap()]) {
                    let parent_transform: Isometry3<f64> = match &x.parent_node {
                        None => { Isometry3::identity() }
                        Some(parent_node) => { transforms[names.iter().position(|y| y == parent_node).unwrap()] }
                    };

                    let scale = x.scale;

                    let curr_transform = parent_transform * x.offset_from_parent;

                    let new_points = x.local_space_trimesh.points.iter().map(|x| {
                        let mut v = Vector3::new(x[0], x[1], x[2]);
                        v[0] *= scale[0];
                        v[1] *= scale[1];
                        v[2] *= scale[2];

                        let new_v = curr_transform.rotation * v + curr_transform.translation.vector;

                        [new_v[0], new_v[1], new_v[2]]
                    }).collect();

                    let new_indices = x.local_space_trimesh.indices.clone();

                    let new_trimesh = TriMesh {
                        points: new_points,
                        indices: new_indices,
                    };

                    out.push(new_trimesh);

                    transforms[i] = curr_transform.clone();
                    done[i] = true;
                }
            });
        }

        out
    }
}

#[derive(Clone, Debug)]
pub struct TrimeshNode {
    pub name: String,
    pub parent_node: Option<String>,
    pub local_space_trimesh: TriMesh,
    pub offset_from_parent: Isometry3<f64>,
    pub scale: [f64; 3]
}