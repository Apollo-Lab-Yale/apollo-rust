use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_proximity::proxima::proxima_core::ProximaBudget;
use apollo_rust_proximity::{ProximityLossFunction, ToProximityValue};
use apollo_rust_proximity::proxima::proxima2::LieAlgMode;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();

    let state = c.zeros_state();
    let mut p1 = c.get_self_proxima1(0.0, &state, LinkShapeMode::Full, LinkShapeRep::ConvexHull);
    let mut p2 = c.get_self_proxima2(&state, LinkShapeMode::Full, LinkShapeRep::ConvexHull, LieAlgMode::Pseudo);

    let proxima_budget = ProximaBudget::AccuracyBudget(0.3);
    let loss = ProximityLossFunction::Hinge { threshold: 1.0 };

    let state = 0.005*c.bounds_module.sample_random_state();
    let link_poses = c.fk(&state);

    let res = c.self_proximity_proxima(&mut p1, &proxima_budget, &loss, 10.0, 1.0, &link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false);
    println!("{:?}", res.result);
    println!("{:?}", res.ground_truth_checks.len());

    let res = c.self_proximity_proxima(&mut p2, &proxima_budget, &loss, 10.0, 1.0, &link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false);
    println!("{:?}", res.result);
    println!("{:?}", res.ground_truth_checks.len());

    let res = c.self_contact(&link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false, 1000000.0, true);
    println!("{:?}", res.to_proximity_value(&loss, 10.0));
}