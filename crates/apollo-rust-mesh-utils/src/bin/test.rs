use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::mesh_object_scene::ToMeshObjectScene;

fn main() {
    let fp = PathBuf::new_from_desktop_dir().append("untitled.glb");
    let g = load_gltf_file(&fp).unwrap();

    g.to_mesh_object_scene().nodes[0].local_space_mesh_object.save_to_glb(&PathBuf::new_from_desktop_dir().append("woah3.glb"));
}