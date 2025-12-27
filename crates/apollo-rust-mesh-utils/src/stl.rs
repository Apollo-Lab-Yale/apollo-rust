use crate::mesh_object::{ExtraInfo, MeshObject};
use crate::mesh_object_scene::{MeshObjectNode, MeshObjectScene, ToMeshObjectScene};
use crate::trimesh::{ToTriMesh, TriMesh};
use apollo_rust_file::ApolloPathBufTrait;
use nalgebra::Isometry3;
use stl_io::IndexedMesh;

impl ToTriMesh for IndexedMesh {
    fn to_trimesh(&self) -> TriMesh {
        let mut points = vec![];

        self.vertices.iter().for_each(|x| {
            points.push([x[0] as f64, x[1] as f64, x[2] as f64]);
        });

        let mut indices = vec![];

        self.faces.iter().for_each(|x| {
            indices.push(x.vertices.clone());
        });

        TriMesh { points, indices }
    }
}

impl ToMeshObjectScene for IndexedMesh {
    fn to_mesh_object_scene(&self) -> MeshObjectScene {
        MeshObjectScene {
            nodes: vec![MeshObjectNode {
                name: "".to_string(),
                parent_node: None,
                local_space_mesh_object: MeshObject {
                    local_space_trimeshes: vec![self.to_trimesh()],
                    offset_from_parent: Isometry3::identity(),
                    scale: [1., 1., 1.],
                    extra_info: ExtraInfo::None,
                },
            }],
        }
    }
}

pub fn load_stl_file<P: ApolloPathBufTrait>(path: &P) -> Result<IndexedMesh, String> {
    path.verify_extension(&vec!["stl", "STL"])?;
    let bytes = path.read_file_contents_to_bytes();
    load_stl_from_bytes(&bytes)
}

pub fn load_stl_from_bytes(bytes: &[u8]) -> Result<IndexedMesh, String> {
    let mut reader = std::io::Cursor::new(bytes);
    let read_res = stl_io::read_stl(&mut reader);
    match read_res {
        Ok(read) => Ok(read),
        Err(e) => Err(e.to_string()),
    }
}
