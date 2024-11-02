use apollo_rust_continuous_planning::feasibility_checkers::FeasibilityCheckerTrait;
use apollo_rust_continuous_planning::feasibility_checkers::robot_feasibility_checkers::{RobotBVHFeasibilityChecker, RobotNaiveFeasibilityChecker};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_proximity::bvh::BvhShapeAABB;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra().to_arc_chain();

    let f = RobotNaiveFeasibilityChecker::new(c.clone(), LinkShapeMode::Full, LinkShapeRep::ConvexHull);

    let res = f.is_feasible_state(&V::new(&[3.0; 6]));
    println!("{:?}", res);

    let f2 = RobotBVHFeasibilityChecker::<BvhShapeAABB>::new(c.clone(), LinkShapeMode::Full, LinkShapeRep::ConvexHull, 2);
    let res = f2.is_feasible_state(&V::new(&[3.0; 6]));
    println!("{:?}", res);
}