use std::io::Read;
use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use gltf::Gltf;
use crate::trimesh::{ToTriMesh, TriMesh};

impl ToTriMesh for Gltf {
    fn to_trimesh(&self) -> TriMesh {
        todo!()
    }
}

pub fn load_gltf_file(path: &PathBuf) -> Result<Gltf, String> {
    let mut file = path.get_file_for_reading();
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer).expect("Failed to read GLTF file");
    let gltf = Gltf::from_slice(&buffer).expect("Failed to parse GLTF file");

    return Ok(gltf);
}

pub fn calculate_min_max(points: &[f32]) -> (Vec<f64>, Vec<f64>) {
    let mut min = vec![f64::INFINITY; 3];
    let mut max = vec![f64::NEG_INFINITY; 3];

    for chunk in points.chunks(3) {
        for i in 0..3 {
            if (chunk[i] as f64) < min[i] {
                min[i] = chunk[i] as f64;
            }
            if (chunk[i] as f64) > max[i] {
                max[i] = chunk[i] as f64;
            }
        }
    }

    (min, max)
}