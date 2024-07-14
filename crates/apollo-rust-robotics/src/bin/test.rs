use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules_builders::RobotPreprocessorRobotsDirectory;
use apollo_rust_robotics::Robot;

fn main() {
    let r = RobotPreprocessorRobotsDirectory::new(PathBuf::new_from_default_apollo_robots_dir());
    let s = r.get_robot_subdirectory("ur5");
    let robot = Robot::new_from_single_robot_directory(&s);
    println!("{:?}", robot.num_dofs());
}