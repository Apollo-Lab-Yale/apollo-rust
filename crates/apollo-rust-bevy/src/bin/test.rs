use std::path::PathBuf;
use bevy::app::{App, Startup};
use bevy::prelude::{AssetServer, Commands, Res};
use apollo_rust_bevy::apollo_bevy_utils::robotics::spawn_robot_meshes;
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robotics::ToRobotFromName;
use apollo_rust_robotics_core::RobotPreprocessorRobotsDirectory;

fn main () {
    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_starter_lights()
        .apollo_bevy_robotics_scene_visuals_start();

    let robot = RobotPreprocessorRobotsDirectory::new_default().to_robot("ur5");
    let path_to_assets = PathBuf::new_from_default_apollo_bevy_assets_dir();
    app.add_systems(Startup, move |mut commands: Commands, asset_server: Res<AssetServer>| {
        spawn_robot_meshes(0, &robot, &V::new(&[0.0; 6]), &path_to_assets, &mut commands, &asset_server);
    });

    app.run();
}