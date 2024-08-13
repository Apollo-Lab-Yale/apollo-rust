use std::path::PathBuf;
use std::sync::Arc;
use bevy::app::App;
use bevy::prelude::Update;
use apollo_rust_bevy::{ApolloBevyTrait};
use apollo_rust_bevy::apollo_bevy_utils::chain::{BevyChainSlidersEgui, ChainMeshesRepresentation};
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChain;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_robots_dir());
    let s = r.get_subdirectory("ur5");
    let chain = s.to_chain();
    let chain_arc = Arc::new(chain.clone());

    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_robotics_scene_visuals_start()
        .apollo_bevy_starter_lights();

    app = app.apollo_bevy_spawn_robot(&chain, 0, ISE3q::new(I3::from_slices_euler_angles(&[1.,0.,0.], &[1.,0.,0.])), vec![(ChainMeshesRepresentation::Plain, MeshType::GLB), (ChainMeshesRepresentation::ConvexDecomposition, MeshType::OBJ)], &PathBuf::new_from_default_apollo_bevy_assets_dir());
    app = app.apollo_bevy_spawn_robot(&chain, 1, ISE3q::new(I3::from_slices_euler_angles(&[0.,1.,0.], &[1.,0.,0.])), vec![(ChainMeshesRepresentation::Plain, MeshType::GLB), (ChainMeshesRepresentation::ConvexDecomposition, MeshType::OBJ)], &PathBuf::new_from_default_apollo_bevy_assets_dir());

    app.add_systems(Update, BevyChainSlidersEgui {
        chain_instance_idx: 0,
        chain: chain_arc.clone(),
    }.get_system_side_panel_left());

    app.add_systems(Update, BevyChainSlidersEgui {
        chain_instance_idx: 1,
        chain: chain_arc.clone(),
    }.get_system_side_panel_left());

    app.run();
}