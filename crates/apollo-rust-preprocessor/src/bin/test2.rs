use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_preprocessor::{PreprocessorModule, ResourcesSubDirectoryTrait};
use apollo_rust_robotics_core::ChainNalgebra;
// use apollo_rust_robot_modules::ResourcesRootDirectory;
// use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_lie_alg_error_models_module::ApolloLinkShapesLieAlgErrorModelsModule;
// use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("go1_test");
    s.preprocess_robot(false);

    // r.preprocess_all_environments(false);
}