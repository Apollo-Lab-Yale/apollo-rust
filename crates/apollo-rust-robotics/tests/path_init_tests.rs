use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_proximity_parry::ToIntersectionResult;
use apollo_rust_robotics::{ChainBuildersTrait, ChainNalgebra, ResourcesType, ToChainFromPath};
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};
use std::path::PathBuf;

#[test]
fn test_path_init_relative() {
    // Relative path from crates/apollo-rust-robotics to ur5_urdd
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = path.to_chain(ResourcesType::Robot);

    assert_eq!(chain.resources_sub_directory.name, "ur5");
    assert!(chain.chain_module.links_in_chain.len() > 0);
    println!(
        "Successfully initialized chain from relative path: {:?}",
        path
    );
}

#[test]
fn test_path_init_with_name_relative() {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = ChainNalgebra::new_from_path_with_name(&path, "ur5", ResourcesType::Robot);

    assert_eq!(chain.resources_sub_directory.name, "ur5");
    assert!(chain.chain_module.links_in_chain.len() > 0);
    println!(
        "Successfully initialized chain with explicit name from relative path: {:?}",
        path
    );
}

#[test]
fn test_chain_builders_trait_relative() {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = ChainNalgebra::new_from_path(&path, ResourcesType::Robot);

    assert_eq!(chain.resources_sub_directory.name, "ur5");
    assert!(chain.chain_module.links_in_chain.len() > 0);
    println!(
        "Successfully initialized chain using ChainBuildersTrait from relative path: {:?}",
        path
    );
}

#[test]
fn test_ur5_fk_relative() {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = path.to_chain(ResourcesType::Robot);

    // UR5 has 6 degrees of freedom
    assert_eq!(chain.dof_module.num_dofs, 6);

    let q = V::new(&[0.0; 6]);
    let poses = chain.fk(&q);

    // Verify we got poses for all links
    assert_eq!(poses.len(), chain.chain_module.links_in_chain.len());

    // Verify some ground truth translations for UR5 at zero configuration
    // Link 2: shoulder_link
    let p2 = poses[2].0.translation.vector;
    assert!((p2[0] - 0.0).abs() < 1e-6);
    assert!((p2[1] - 0.13585).abs() < 1e-6);
    assert!((p2[2] - 0.089159).abs() < 1e-6);

    // Link 3: upper_arm_link
    let p3 = poses[3].0.translation.vector;
    assert!((p3[0] - 0.0).abs() < 1e-6);
    assert!((p3[1] - 0.01615).abs() < 1e-4);
    assert!((p3[2] - 0.514159).abs() < 1e-4);

    // Link 4: forearm_link
    let p4 = poses[4].0.translation.vector;
    assert!((p4[0] - 0.0).abs() < 1e-6);
    assert!((p4[1] - 0.01615).abs() < 1e-4);
    assert!((p4[2] - 0.906409).abs() < 1e-4);

    println!("Successfully verified FK with ground truth for UR5 from relative path");
}

#[test]
fn test_ur5_self_collision_relative() {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = path.to_chain(ResourcesType::Robot);

    // Zero config should be collision-free
    let q_zero = V::new(&[0.0; 6]);
    let poses_zero = chain.fk(&q_zero);
    let res_zero = chain.self_contact(
        &poses_zero,
        LinkShapeMode::Full,
        LinkShapeRep::ConvexHull,
        false,
        0.0,
        false,
    );
    assert!(!res_zero.to_intersection_result());

    // Known colliding config
    let q_colliding = V::new(&[0.0, -1.5, 3.0, 0.0, 0.0, 0.0]);
    let poses_colliding = chain.fk(&q_colliding);
    let res_colliding = chain.self_contact(
        &poses_colliding,
        LinkShapeMode::Full,
        LinkShapeRep::ConvexHull,
        false,
        0.0,
        false,
    );
    assert!(res_colliding.to_intersection_result());

    println!("Successfully verified self-collision logic for UR5 from relative path");
}

#[test]
fn test_ur5_proximity_queries_relative() {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = path.to_chain(ResourcesType::Robot);

    // Zero config
    let q_zero = V::new(&[0.0; 6]);
    let poses_zero = chain.fk(&q_zero);

    // test self_distance
    let res_dist = chain.self_distance(
        &poses_zero,
        LinkShapeMode::Full,
        LinkShapeRep::ConvexHull,
        false,
    );
    let min_dist = res_dist
        .outputs
        .iter()
        .fold(f64::INFINITY, |a, &b| a.min(b));
    assert!((min_dist - 0.2757).abs() < 1e-3);

    // test self_contact
    let res_contact = chain.self_contact(
        &poses_zero,
        LinkShapeMode::Full,
        LinkShapeRep::ConvexHull,
        false,
        0.0,
        false,
    );
    assert_eq!(res_contact.outputs.iter().flatten().count(), 0);

    // Colliding config
    let q_colliding = V::new(&[0.0, -1.5, 3.0, 0.0, 0.0, 0.0]);
    let poses_colliding = chain.fk(&q_colliding);

    let res_contact_col = chain.self_contact(
        &poses_colliding,
        LinkShapeMode::Full,
        LinkShapeRep::ConvexHull,
        false,
        0.0,
        false,
    );
    let min_dist_col = res_contact_col
        .outputs
        .iter()
        .flatten()
        .map(|c| c.dist)
        .fold(f64::INFINITY, |a, b| a.min(b));
    assert!(min_dist_col < 0.0);
    assert!((min_dist_col - (-0.0467)).abs() < 1e-3);
    assert!(res_contact_col.outputs.iter().flatten().count() > 0);

    println!("Successfully verified self_contact and self_distance for UR5 from relative path");
}
