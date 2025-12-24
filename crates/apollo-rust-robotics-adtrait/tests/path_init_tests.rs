use apollo_rust_robotics_adtrait::{ChainBuildersTrait, ResourcesType, ToChainFromPath};
use std::path::PathBuf;

#[test]
fn test_path_init_relative() {
    // Relative path from crates/apollo-rust-robotics-adtrait to ur5_urdd
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("../../ur5_urdd");

    let chain = path.to_chain::<f64>(ResourcesType::Robot);

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

    let chain =
        apollo_rust_robotics_core_adtrait::ChainNalgebraADTrait::<f64>::new_from_path_with_name(
            &path,
            "ur5",
            ResourcesType::Robot,
        );

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

    let chain = apollo_rust_robotics_core_adtrait::ChainNalgebraADTrait::<f64>::new_from_path(
        &path,
        ResourcesType::Robot,
    );

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

    let chain = path.to_chain::<f64>(ResourcesType::Robot);

    // UR5 has 6 degrees of freedom
    assert_eq!(chain.num_dofs(), 6);

    let q = chain.zeros_state();
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

    println!("Successfully verified FK for UR5 from relative path in ADTrait");
}
