use std::io::BufReader;
use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use obj::{Obj, load_obj};
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

pub fn load_obj_file(path: &PathBuf) -> Result<Obj, String> {
    path.verify_extension(&vec!["obj", "OBJ"])?;

    let file = path.get_file_for_reading();
    let reader = BufReader::new(file);

    let obj = load_obj(reader);
    if obj.is_err() { return Err("could not load obj".to_string()) }

    let obj = obj.expect("error");

    Ok(obj)
}