use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;
use apollo_rust_mesh_utils::trimesh_scene::ToTrimeshScene;

fn main() {
    let fp = PathBuf::new_from_desktop_dir().append("untitled.glb");
    let g = load_gltf_file(&fp).unwrap();

    let tms = g.to_trimesh_scene();
    println!("{:?}", tms);

    let tm = g.to_trimesh();
    println!("{:?}", tm);

    tm.save_to_stl(&PathBuf::new_from_desktop_dir().append("test.stl"));
}