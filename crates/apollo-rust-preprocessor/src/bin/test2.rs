use apollo_rust_preprocessor::{PreprocessorModule, ResourcesRootDirectoryTrait};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("b1");
    ApolloLinkShapesSkipsModule::load_or_build(&s, true).expect("error");
    // r.preprocess_all_environments(false);
}