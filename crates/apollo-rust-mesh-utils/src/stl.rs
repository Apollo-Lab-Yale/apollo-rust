use std::fs::File;
use std::path::PathBuf;
use nalgebra::Isometry3;
use apollo_rust_file::ApolloPathBufTrait;
use stl_io::IndexedMesh;
use crate::mesh_object::{ExtraInfo, MeshObject};
use crate::mesh_object_scene::{MeshObjectNode, MeshObjectScene, ToMeshObjectScene};
use crate::trimesh::{ToTriMesh, TriMesh};

impl ToTriMesh for IndexedMesh {
    fn to_trimesh(&self) -> TriMesh {
        let mut points = vec![];

        self.vertices.iter().for_each(|x| {
            points.push( [x[0] as f64, x[1] as f64, x[2] as f64] );
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
                    scale: [1.,1.,1.],
                    extra_info: ExtraInfo::None,
                },
            }],
        }
    }
}

pub fn load_stl_file(path: &PathBuf) -> Result<IndexedMesh, String> {
    path.verify_extension(&vec!["stl", "STL"])?;
    let mut file = File::open(path);
    match &mut file {
        Ok(f) => {
            let read_res = stl_io::read_stl(f);
            match read_res {
                Ok(read) => { Ok(read) }
                Err(e) => { Err(e.to_string()) }
            }
        }
        Err(e) => { Err(e.to_string()) }
    }
}