use std::fs::File;
use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use stl_io::IndexedMesh;
use crate::utils::trimesh::{ToTriMesh, TriMesh};

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