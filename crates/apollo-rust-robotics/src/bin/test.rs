use apollo_rust_robot_modules_preprocessor::RobotPreprocessorRobotsDirectoryTrait;
use apollo_rust_robotics_core::RobotPreprocessorRobotsDirectory;

fn main() {
    let r = RobotPreprocessorRobotsDirectory::new_default();
    r.preprocess_all(false);
}