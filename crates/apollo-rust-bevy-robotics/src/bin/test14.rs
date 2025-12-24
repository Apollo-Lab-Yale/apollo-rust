use apollo_rust_linalg::V;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};
use apollo_rust_robotics_core::robot_functions::robot_proximity_functions::RobotProximityFunctions;
use std::time::Instant;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1");
    let c = s.to_chain_nalgebra();

    let link_shape_mode = LinkShapeMode::Decomposition;
    let link_shape_rep = LinkShapeRep::ConvexHull;

    let state = V::new_random(12);
    let link_poses = c.fk(&state);

    let start = Instant::now();
    // c.self_intersect_from_state(&state, LinkShapeMode::Full, LinkShapeRep::BoundingSphere, false);.
    RobotProximityFunctions::self_distance(
        c.link_shapes_module(),
        &link_poses,
        link_shape_mode,
        link_shape_rep,
        None,
        false,
    );
    let stop1 = start.elapsed();
    println!("{:?}", stop1);

    let start = Instant::now();
    // c.self_intersect_from_state(&state, LinkShapeMode::Full, LinkShapeRep::BoundingSphere, false);.
    RobotProximityFunctions::self_distance(
        c.link_shapes_module(),
        &link_poses,
        link_shape_mode,
        link_shape_rep,
        Some(
            c.link_shapes_skips_nalgebra_module
                .get_skips(link_shape_mode, link_shape_rep),
        ),
        false,
    );
    let stop2 = start.elapsed();
    println!("{:?}", stop2);

    println!("{:?}", stop2.as_secs_f64() / stop1.as_secs_f64())
}
