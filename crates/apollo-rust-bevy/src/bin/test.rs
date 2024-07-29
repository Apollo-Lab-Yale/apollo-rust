use std::path::PathBuf;
use bevy::app::{App, Startup};
use bevy::asset::Assets;
use bevy::pbr::StandardMaterial;
use bevy::prelude::{AssetServer, Commands, Res, ResMut};
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_bevy::apollo_bevy_utils::robotics::{RobotMeshesRepresentation, spawn_robot_meshes};
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_preprocessor::ResourcesRootDirectoryTrait;
use apollo_rust_robot_modules::ResourcesRobotsDirectory;
use apollo_rust_robotics::ToRobotFromName;

fn main () {
    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_starter_lights()
        .apollo_bevy_robotics_scene_visuals_start();

    let robot = ResourcesRobotsDirectory::new_default().to_robot("ur5");
    let path_to_assets = PathBuf::new_from_default_apollo_bevy_assets_dir();
    app.add_systems(Startup, move |mut commands: Commands, asset_server: Res<AssetServer>, mut materials: ResMut<Assets<StandardMaterial>>| {
        spawn_robot_meshes(0, RobotMeshesRepresentation::ConvexHull, MeshType::OBJ, &robot, &V::new(&[0.0; 6]), &path_to_assets, &mut commands, &asset_server, &mut materials);
        // spawn_robot_meshes(0, RobotMeshesRepresentation::Plain, MeshType::GLB, &robot, &V::new(&[0.0; 6]), &path_to_assets, &mut commands, &asset_server, &mut materials);
    });

    app.run();
}