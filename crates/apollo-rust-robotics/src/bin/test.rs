use apollo_rust_robot_modules_builders::RobotPreprocessorRobotsDirectory;

fn main() {
    let r = RobotPreprocessorRobotsDirectory::new_default();
    r.preprocess_all(false);
}