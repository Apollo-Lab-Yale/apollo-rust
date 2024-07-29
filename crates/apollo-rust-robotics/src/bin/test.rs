use apollo_rust_preprocessor::{ResourcesRootDirectoryPreprocessorTrait, ResourcesRootDirectoryTrait};
use apollo_rust_robot_modules::ResourcesRobotsDirectory;
use apollo_rust_robotics::ToRobotFromName;

fn main() {
    let r = ResourcesRobotsDirectory::new_default();
    r.preprocess_all(false);

    let rr = r.to_robot("ur5");
    println!("{:?}", rr.num_dofs());
}