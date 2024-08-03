use std::io::BufReader;
use std::path::PathBuf;
use nalgebra::Isometry3;
use apollo_rust_file::ApolloPathBufTrait;
use obj::{Obj, load_obj};
use crate::mesh_object::{ExtraInfo, MeshObject};
use crate::mesh_object_scene::{MeshObjectNode, MeshObjectScene, ToMeshObjectScene};
use crate::trimesh::{ToTriMesh, TriMesh};

impl ToTriMesh for Obj {
    fn to_trimesh(&self) -> TriMesh {
        let points: Vec<[f64; 3]> = self.vertices.iter()
            .map(|v| [v.position[0] as f64, v.position[1] as f64, v.position[2] as f64])
            .collect();

        let mut indices = Vec::new();
        for i in (0..self.indices.len()).step_by(3) {
            let tri = [
                self.indices[i] as usize,
                self.indices[i + 1] as usize,
                self.indices[i + 2] as usize,
            ];
            indices.push(tri);
        }

        TriMesh { points, indices }
    }
}

impl ToMeshObjectScene for Obj {
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

pub fn load_obj_file(path: &PathBuf) -> Result<Obj, String> {
    path.verify_extension(&vec!["obj", "OBJ"])?;

    let file = path.get_file_for_reading();
    let reader = BufReader::new(file);

    let obj = load_obj(reader);
    if obj.is_err() { return Err("could not load obj".to_string()) }

    let obj = obj.expect("error");

    Ok(obj)
}