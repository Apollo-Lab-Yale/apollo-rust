use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::collada::load_dae_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;

fn main() {
    let fp = PathBuf::new_from_default_apollo_robots_dir().append("go2/mesh_modules/original_meshes_module/meshes/base.dae");
    let d = load_dae_file(&fp).expect("error");

    let f = d.to_trimesh();
    println!("{:?}", f.points[0]);
}