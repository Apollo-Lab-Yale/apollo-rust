use std::path::PathBuf;
use gltf::{Document};
use gltf::buffer::Data as Data1;
use gltf::image::Data as Data2;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use crate::trimesh::{ToTriMesh, TriMesh};
use crate::trimesh_scene::{ToTrimeshScene, TrimeshNode, TrimeshScene};

impl ToTriMesh for GltfInfo {
    fn to_trimesh(&self) -> TriMesh {
        let trimesh_scene = self.to_trimesh_scene();

        let mut out = TriMesh::new_empty();

        let mut done = vec![false; trimesh_scene.nodes.len()];
        let names: Vec<String> = trimesh_scene.nodes.iter().map(|x| x.name.clone()).collect();
        let mut transforms = vec![Isometry3::identity(); trimesh_scene.nodes.len()];

        while !done.iter().all(|x| *x) {
            trimesh_scene.nodes.iter().enumerate().for_each(|(i, x)| {
                if (x.parent_node.is_none() && !done[i]) || (x.parent_node.is_some() && !done[i] && done[names.iter().position(|y| y == x.parent_node.as_ref().unwrap()).unwrap()]) {
                    let parent_transform: Isometry3<f64> = match &x.parent_node {
                        None => { Isometry3::identity() }
                        Some(parent_node) => { transforms[names.iter().position(|y| y == parent_node).unwrap()] }
                    };

                    let curr_transform = parent_transform * x.offset_from_parent;

                    let new_points = x.local_space_trimesh.points.iter().map(|x| {
                        let v = Vector3::new(x[0], x[1], x[2]);

                        let new_v = curr_transform.rotation * v + curr_transform.translation.vector;
                        [new_v[0], new_v[1], new_v[2]]
                    }).collect();

                    let new_indices = x.local_space_trimesh.indices.clone();

                    let new_trimesh = TriMesh {
                        points: new_points,
                        indices: new_indices,
                    };

                    out.extend(&new_trimesh);

                    transforms[i] = curr_transform.clone();
                    done[i] = true;
                }
            });
        }

        out
    }
}

impl ToTrimeshScene for GltfInfo {
    fn to_trimesh_scene(&self) -> TrimeshScene {
        let mut out = TrimeshScene {
            nodes: vec![],
        };

        self.document.scenes().for_each(|scene| {
            let mut node_stack = vec![];
            scene.nodes().for_each(|node| node_stack.push( (node, None::<String>)));

            while !node_stack.is_empty() {
                let (node, parent_name) = node_stack.pop().unwrap();

                let name = match node.name() {
                    None => { todo!() }
                    Some(s) => { s.to_string() }
                };
                let ( xyz, q, scale ) = node.transform().decomposed();
                let t = Translation3::new(xyz[0] as f64, xyz[1] as f64, xyz[2] as f64);
                let uq = UnitQuaternion::from_quaternion(Quaternion::new(q[3] as f64, q[0] as f64, q[1] as f64, q[2] as f64));
                let offset_from_parent = Isometry3::from_parts(t, uq);

                let mesh = node.mesh().unwrap();
                let mut points = vec![];
                let mut indices = vec![];

                mesh.primitives().for_each(|primitive| {
                    let reader = primitive.reader(|buffer| {
                        Some(&self.buffers[buffer.index()])
                    });

                    let positions = reader.read_positions().unwrap();
                    for position in positions {
                        points.push([ scale[0] as f64 * position[0] as f64, scale[1] as f64 * position[1] as f64, scale[2] as f64 * position[2] as f64]);
                    }

                    let mut i = Vec::new();
                    if let Some(gltf::mesh::util::ReadIndices::U16(gltf::accessor::Iter::Standard(iter))) = reader.read_indices(){
                        for v in iter{
                            i.push(v as usize);
                        }
                    }

                    i.chunks(3).for_each(|a| {
                        indices.push(  [a[0], a[1], a[2]] );
                    });

                });

                out.nodes.push(TrimeshNode {
                    name: name.clone(),
                    parent_node: match &parent_name {
                        None => { None }
                        Some(s) => { Some(s.to_string()) }
                    },
                    local_space_trimesh: TriMesh {
                        points,
                        indices,
                    },
                    offset_from_parent,
                });

                node.children().for_each(|child| {
                    match child.mesh() {
                        None => {}
                        Some(_) => { node_stack.push( (child, Some(name.clone())) ) }
                    }
                });
            }
        });

        out
    }
}

pub fn load_gltf_file(path: &PathBuf) -> Result<GltfInfo, String> {
    let (document, buffers, images) = gltf::import(path.clone()).expect("error");
    Ok(GltfInfo {
        document,
        buffers,
        images,
    })
}

#[derive(Clone, Debug)]
pub struct GltfInfo {
    pub document: Document,
    pub buffers: Vec<Data1>,
    pub images: Vec<Data2>
}