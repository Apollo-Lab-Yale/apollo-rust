use std::path::PathBuf;
use gltf::{Document, Material,};
use gltf::buffer::Data as Data1;
use gltf::image::Data as Data2;
use gltf::texture::Info;
use gltf_json::{Index};
use gltf_json::extras::Void;
use gltf_json::material::{AlphaCutoff, PbrBaseColorFactor, PbrMetallicRoughness, StrengthFactor};
use gltf_json::validation::Checked;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use crate::mesh_object::{ExtraInfo, MeshObject};
use crate::mesh_object_scene::{MeshObjectNode, MeshObjectScene, ToMeshObjectScene};
use crate::trimesh::{ToTriMesh, TriMesh};
use crate::trimesh_scene::{ToTrimeshScene, TrimeshNode, TrimeshScene};

impl ToTriMesh for GltfInfo {
    fn to_trimesh(&self) -> TriMesh {
        let trimesh_scene = self.to_trimesh_scene();
        // trimesh_scene.to_resolved_trimeshes().to_trimesh()
        let mut out = TriMesh::new_empty();
        trimesh_scene.nodes.iter().for_each(|x| {
            out.extend(&x.local_space_trimesh);
        });
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

            let mut count = 0;
            while !node_stack.is_empty() {
                let (node, parent_name) = node_stack.pop().unwrap();

                let name = match node.name() {
                    None => { format!("object_{}", count) }
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
                        points.push([ position[0] as f64, position[1] as f64, position[2] as f64]);
                    }

                    let mut i = Vec::new();
                    if let Some(gltf::mesh::util::ReadIndices::U16(gltf::accessor::Iter::Standard(iter))) = reader.read_indices() {
                        for v in iter{
                            i.push(v as usize);
                        }
                    } else if let Some(gltf::mesh::util::ReadIndices::U32(gltf::accessor::Iter::Standard(iter))) = reader.read_indices() {
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
                    scale: [scale[0] as f64, scale[1] as f64, scale[2] as f64],
                });

                node.children().for_each(|child| {
                    match child.mesh() {
                        None => {}
                        Some(_) => { node_stack.push( (child, Some(name.clone())) ) }
                    }
                });

                count += 1;
            }
        });

        out
    }
}

impl ToMeshObjectScene for GltfInfo {
    fn to_mesh_object_scene(&self) -> MeshObjectScene {
        let mut out = MeshObjectScene {
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
               let mut materials = vec![];

               mesh.primitives().for_each(|primitive| {
                   let mut curr_points = vec![];
                   let mut curr_indices = vec![];

                   let reader = primitive.reader(|buffer| {
                       Some(&self.buffers[buffer.index()])
                   });

                   let positions = reader.read_positions().unwrap();
                   for position in positions {
                       curr_points.push([ position[0] as f64, position[1] as f64, position[2] as f64]);
                   }

                   let mut i = Vec::new();
                   if let Some(gltf::mesh::util::ReadIndices::U16(gltf::accessor::Iter::Standard(iter))) = reader.read_indices(){
                       for v in iter{
                           i.push(v as usize);
                       }
                   }

                   i.chunks(3).for_each(|a| {
                       curr_indices.push(  [a[0], a[1], a[2]] );
                   });

                   let mat = primitive.material();
                   materials.push(mat.to_json_material());

                   points.push(curr_points);
                   indices.push(curr_indices);
               });

               let trimeshes: Vec<TriMesh> = points.iter().zip(indices.iter()).map(|(x, y)| {
                   TriMesh {
                       points: x.clone(),
                       indices: y.clone(),
                   }
               }).collect();

               out.nodes.push(MeshObjectNode {
                   name: name.clone(),
                   parent_node: parent_name.clone(),
                   local_space_mesh_object: MeshObject {
                       local_space_trimeshes: trimeshes,
                       offset_from_parent,
                       scale: [scale[0] as f64, scale[1] as f64, scale[2] as f64],
                       extra_info: ExtraInfo::GLB { materials },
                   },
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

pub trait ToGlbJsonMaterial {
    fn to_json_material(&self) -> gltf_json::Material;
}
impl<'a> ToGlbJsonMaterial for Material<'a> {
    fn to_json_material(&self) -> gltf_json::Material {
        gltf_json::Material {
            alpha_cutoff: match self.alpha_cutoff() {
                None => { None }
                Some(a) => { Some(AlphaCutoff(a)) }
            },
            alpha_mode: Checked::Valid(self.alpha_mode().clone()),
            double_sided: self.double_sided(),
            name: match self.name() {
                None => { None }
                Some(s) => { Some(s.to_string()) }
            },
            pbr_metallic_roughness: PbrMetallicRoughness {
                base_color_factor: PbrBaseColorFactor(self.pbr_metallic_roughness().base_color_factor()),
                base_color_texture: match self.pbr_metallic_roughness().base_color_texture() {
                    None => { None }
                    Some(a) => { Some(texture_info_conversion(&a))}
                },
                metallic_factor: StrengthFactor(self.pbr_metallic_roughness().metallic_factor()),
                roughness_factor: StrengthFactor(self.pbr_metallic_roughness().roughness_factor()),
                metallic_roughness_texture: None,
                extensions: None,
                extras: Void::default(),
            },
            normal_texture: None,
            occlusion_texture: None,
            emissive_texture: None,
            emissive_factor: Default::default(),
            extensions: None,
            extras: Void::default(),
        }
    }
}

fn texture_info_conversion(info: &Info) -> gltf_json::texture::Info {
    gltf_json::texture::Info {
        index: Index::new(info.texture().index() as u32),
        tex_coord: info.tex_coord(),
        extensions: None,
        extras: Void::default(),
    }
}

