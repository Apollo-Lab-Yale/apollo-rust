use apollo_rust_robotics::Robot;
use apollo_rust_robotics_core::RobotPreprocessorRobotsDirectory;

fn main() {
    let r = RobotPreprocessorRobotsDirectory::new_default();
    // r.preprocess_all(false);

    let robot = Robot::new_from_root(&r, "ur5");
    println!("{:?}", robot.link_shapes_distance_statistics_module().full_convex_hulls.maximums);
}