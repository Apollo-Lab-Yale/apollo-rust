use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChain;

fn main() {
    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_robots_dir());
    let s = r.get_subdirectory("xarm7_with_gripper_and_rail");
    let robot = s.to_chain();
    let fk_res = robot.fk(&V::new(&[-8.5; 14]));
    let d = robot.reverse_of_fk(&fk_res);
    println!("{:?}", d);
}