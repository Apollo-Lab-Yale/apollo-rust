use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_proximity::bvh::BvhShapeAABB;
use apollo_rust_proximity::{ProximityLossFunction, ToProximityValue};
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();
    let mut bvh = c.get_bvh::<BvhShapeAABB>(&V::new(&[0.0; 6]), LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, 2);

    let fk_res = c.fk(&V::new(&[2.0; 6]));
    let res = c.self_contact_bvh(&mut bvh, &fk_res, LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, false, 1.0);
    println!("{:?}", res.to_proximity_value(&ProximityLossFunction::Hinge { threshold: 1.0 }, 10.0));

    let res = c.self_contact(&fk_res, LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, false, 1.0);
    println!("{:?}", res.to_proximity_value(&ProximityLossFunction::Hinge { threshold: 1.0 }, 10.0));

    println!("{:?}", res.num_ground_truth_checks);
}