use std::path::PathBuf;
use std::str::FromStr;
use apollo_rust_file::ApolloPathBufTrait;
use dae_parser::{ArrayElement, Document, Geometry, LocalMap, Node, Primitive, Semantic, Source, Transform, Vertices};
use nalgebra::{Isometry3, Matrix3, Matrix4, Rotation3, Unit, UnitQuaternion, Vector3};
use crate::mesh_object::{ExtraInfo, MeshObject};
use crate::mesh_object_scene::{MeshObjectNode, MeshObjectScene, ToMeshObjectScene};
use crate::trimesh::{ToTriMesh, TriMesh};

impl ToTriMesh for Document {
    fn to_trimesh(&self) -> TriMesh {
        let mut out_trimesh = TriMesh::new_empty();

        let visual_scene = self.get_visual_scene();
        if let Some(visual_scene) = visual_scene {
            let geometry_map = self.local_map::<Geometry>().unwrap();
            let vertices_map = self.local_map::<Vertices>().unwrap();
            let sources_map = self.local_map::<Source>().unwrap();

            let mut node_and_transforms_stack = vec![ ];

            visual_scene.nodes.iter().for_each(|x| {
                node_and_transforms_stack.push( (x.clone(), vec![ ]) );
            });

            while !node_and_transforms_stack.is_empty() {
                let (curr_node, parent_transforms) = node_and_transforms_stack.pop().unwrap();
                let mut curr_transforms = parent_transforms.clone();
                curr_transforms.extend(curr_node.transforms.clone());

                let curr_trimesh = get_trimesh_from_collada_node(&curr_node, &curr_transforms, &geometry_map, &vertices_map, &sources_map);
                out_trimesh.extend(&curr_trimesh);

                curr_node.children.iter().for_each(|x| {
                    node_and_transforms_stack.push( (x.clone(), curr_transforms.clone()) )
                });
            }
        }

        let unit = &self.asset.unit;
        let meter = unit.meter as f64;

        if meter != 1.0 {
            out_trimesh.points.iter_mut().for_each(|x| {
                x.iter_mut().for_each(|y| *y *= meter);
            })
        }

        out_trimesh
    }
}

impl ToMeshObjectScene for Document {
    fn to_mesh_object_scene(&self) -> MeshObjectScene {
        MeshObjectScene {
            nodes: vec![MeshObjectNode {
                name: "".to_string(),
                parent_node: None,
                local_space_mesh_object: MeshObject {
                    local_space_trimeshes: vec![self.to_trimesh()],
                    offset_from_parent: Isometry3::identity(),
                    scale: [1.,1.,1.],
                    extra_info: ExtraInfo::None,
                },
            }],
        }
    }
}

pub fn load_dae_file(path: &PathBuf) -> Result<Document, String> {
    path.verify_extension(&vec!["dae", "DAE"])?;
    let contents = path.read_file_contents_to_string();
    Ok( Document::from_str(&contents).expect(&format!("there was an error loading dae. {:?}", path)) )
}

fn get_trimesh_from_collada_node(node: &Node, curr_transforms: &Vec<Transform>, geometry_map: &LocalMap<Geometry>, vertices_map: &LocalMap<Vertices>, sources_map: &LocalMap<Source>) -> TriMesh {
    let mut trimesh = TriMesh::new_empty();

    node.instance_geometry.iter().for_each(|instance_geometry| {
        let url = instance_geometry.url.clone();
        let geometry = geometry_map.get_raw(&url);
        if let Some(geometry) = geometry {
            let mesh = geometry.element.as_mesh();
            if let Some(mesh) = mesh {
                mesh.elements.iter().for_each(|primitive| {
                    match primitive {
                        Primitive::Triangles(triangles) => {
                            let mut curr_points = vec![];
                            let mut curr_indices = vec![];

                            let mut num_offsets = 0;

                            let mut vertex_offset = None;

                            triangles.inputs.iter().for_each(|input| {
                                num_offsets += 1;
                                match &input.semantic {
                                    Semantic::Vertex => {
                                        vertex_offset = Some(input.offset.clone() as usize);
                                        let url = input.source.clone();
                                        let vertices = vertices_map.get_raw(&url).unwrap();
                                        let url = vertices.position_input().source.clone();
                                        let positions = sources_map.get_raw(&url).unwrap();
                                        let arr = positions.array.as_ref().expect("error");
                                        match arr {
                                            ArrayElement::Float(arr) => {
                                                arr.val.chunks(3).for_each(|x| {
                                                    curr_points.push( [x[0] as f64, x[1] as f64, x[2] as f64] );
                                                });
                                            }
                                            _ => { }
                                        }
                                    }
                                    _ => {  }
                                }
                            });
                            transform_points(&mut curr_points, curr_transforms);

                            let indices = triangles.data.prim.as_ref().expect("error");

                            indices.chunks(num_offsets * 3).for_each(|chunk| {
                                if let Some(vertex_offset ) = vertex_offset {
                                    let i1 = chunk[num_offsets * 0 + vertex_offset] as usize;
                                    let i2 = chunk[num_offsets * 1 + vertex_offset] as usize;
                                    let i3 = chunk[num_offsets * 2 + vertex_offset] as usize;
                                    // points.push( [ all_points[i1], all_points[i2], all_points[i3] ] );
                                    curr_indices.push([i1, i2, i3]);
                                }
                            });

                            trimesh.extend_from_points_and_indices(&curr_points, &curr_indices);
                        }
                        Primitive::Lines(_) => {  }
                        Primitive::LineStrips(_) => { unimplemented!("I just saw a primitive of type LineStrips when parsing a collada file.  Maybe it's time to figure this out.") }
                        Primitive::Polygons(_) => { unimplemented!("I just saw a primitive of type Polygons when parsing a collada file.  Maybe it's time to figure this out.") }
                        Primitive::PolyList(_) => { unimplemented!("I just saw a primitive of type PolyList when parsing a collada file.  Maybe it's time to figure this out.") }
                        Primitive::TriFans(_) => { unimplemented!("I just saw a primitive of type TriFans when parsing a collada file.  Maybe it's time to figure this out.") }
                        Primitive::TriStrips(_) => { unimplemented!("I just saw a primitive of type TriStrips when parsing a collada file.  Maybe it's time to figure this out.") }
                    }
                });
            }
        }
    });

    trimesh
}

fn transform_points(points: &mut Vec<[f64; 3]>, transforms: &Vec<Transform>) {
    transforms.iter().for_each(|t| {
        match t {
            Transform::LookAt(_) => { unimplemented!("lookat is not handled.") }
            Transform::Matrix(matrix) => {
                let mm = &matrix.0;
                let mm: Vec<f64> = mm.iter().map(|x| *x as f64).collect();
                let mat = Matrix4::from_row_slice(mm.as_slice());
                let translation = Vector3::new(mat[(0, 3)], mat[(1, 3)], mat[(2, 3)]);
                let m = mat.fixed_view::<3, 3>(0, 0);
                let m: Matrix3<f64> = m.into();
                let rotation = UnitQuaternion::from_matrix(&m);
                let iso = Isometry3::from_parts(translation.into(), rotation);
                points.iter_mut().for_each(|p| {
                    // *p = iso.mul_by_point_generic(&p);
                    let mut pp = Vector3::new(p[0], p[1], p[2]);
                    pp = iso*pp;
                    p[0] = pp[0];
                    p[1] = pp[1];
                    p[2] = pp[2];
                });
            }
            Transform::Rotate(rotate) => {
                let rr = &rotate.0;
                let rotation = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::new(rr[0] as f64, rr[1] as f64, rr[2] as f64)), (rr[3] as f64).to_degrees());
                points.iter_mut().for_each(|p| {
                    // *p = rotation.mul_by_point_generic(p)
                    let mut pp = Vector3::new(p[0], p[1], p[2]);
                    pp = rotation*pp;
                    p[0] = pp[0];
                    p[1] = pp[1];
                    p[2] = pp[2];
                });
            }
            Transform::Scale(scale) => {
                let ss = &scale.0;
                points.iter_mut().for_each(|p| {
                    p[0] *= ss[0] as f64;
                    p[1] *= ss[1] as f64;
                    p[2] *= ss[2] as f64;
                });
            }
            Transform::Skew(_) => { unimplemented!("skew is not handled.") }
            Transform::Translate(translate) => {
                let tt = &translate.0;
                points.iter_mut().for_each(|p| {
                    p[0] += tt[0] as f64;
                    p[1] += tt[1] as f64;
                    p[2] += tt[2] as f64;
                });
            }
        }
    });
}

////////////////////////////////////////////////////////////////////////////////////////////////////



