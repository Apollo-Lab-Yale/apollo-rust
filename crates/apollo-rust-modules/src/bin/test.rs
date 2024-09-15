use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_robot_modules::robot_modules::urdf_module::ApolloURDFModule;

fn main() {
    let p = PathBuf::new_from_default_apollo_robots_dir();
    let q = p.walk_directory_and_find_first("aliengo").append("urdf_module/module.json");
    let urdf_module: ApolloURDFModule = q.load_object_from_json_file_result().expect("error reading: ");
    println!("{:?}", urdf_module);
}