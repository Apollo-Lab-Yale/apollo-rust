use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_preprocessor::{PreprocessorModule, ResourcesSubDirectoryTrait};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("d1");

    // s.preprocess_robot(false);

    ApolloPlainMeshesModule::load_or_build(&s, true);
    // ApolloLinkShapesSkipsModule::load_or_build(&s, true).expect("error");
}