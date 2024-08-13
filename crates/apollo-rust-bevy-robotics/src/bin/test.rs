use std::path::PathBuf;
use bevy::app::App;
use apollo_rust_bevy::{ApolloBevyTrait, BevyChainCounter};
use apollo_rust_bevy::apollo_bevy_utils::chain::ChainMeshesRepresentation;
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChain;

fn main() {
    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_robots_dir());
    let s = r.get_subdirectory("ur5");
    let chain = s.to_chain();

    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_robotics_scene_visuals_start()
        .apollo_bevy_starter_lights();

    let mut chain_counter = BevyChainCounter::new();
    app = app.apollo_bevy_spawn_robot(&mut chain_counter, &chain, vec![(ChainMeshesRepresentation::Plain, MeshType::GLB)], &PathBuf::new_from_default_apollo_bevy_assets_dir());

    app.run();
}