use std::path::PathBuf;
use std::sync::Arc;
use bevy::app::{App, Startup};
use bevy::asset::Assets;
use bevy::pbr::StandardMaterial;
use bevy::prelude::{AssetServer, Commands, Query, Res, ResMut, Transform, Update};
use bevy_egui::{EguiContexts};
use bevy_egui::egui::panel::Side;
use bevy_egui::egui::{SidePanel, Window};
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_bevy::apollo_bevy_utils::robotics::{RobotMeshesRepresentation, spawn_robot_meshes, RobotStates, RobotLinkMesh, robot_state_updater_loop, robot_sliders_egui};
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

    app.insert_resource(RobotStates { states: vec![V::new(&[0.0; 6])] });

    let robot = Arc::new(ResourcesRobotsDirectory::new_default().to_robot("ur5"));
    let robot1 = robot.clone();
    let robot2 = robot.clone();
    let robot3 = robot.clone();

    let path_to_assets = PathBuf::new_from_default_apollo_bevy_assets_dir();
    app.add_systems(Startup,  move |mut commands: Commands, asset_server: Res<AssetServer>, mut materials: ResMut<Assets<StandardMaterial>>| {
        spawn_robot_meshes(0, RobotMeshesRepresentation::Plain, MeshType::OBJ, &robot1, &V::new(&[0.0; 6]), &path_to_assets, &mut commands, &asset_server, &mut materials);
        // spawn_robot_meshes(0, RobotMeshesRepresentation::ConvexDecomposition, MeshType::GLB, &robot1, &V::new(&[0.0; 7]), &path_to_assets, &mut commands, &asset_server, &mut materials);
    });

    app.add_systems(Update, move |mut query: Query<(&mut Transform, &RobotLinkMesh)>, robot_states: Res<RobotStates>| {
        robot_state_updater_loop(&robot2, &mut query, &robot_states);
    });

    app.add_systems(Update, move |mut robot_states: ResMut<RobotStates>, mut egui_contexts: EguiContexts| {
        Window::new("hello").show(egui_contexts.ctx_mut(), |ui| {
            robot_sliders_egui(0, &robot3, ui, &mut robot_states);
            if ui.ui_contains_pointer() { println!("yep"); }
        });

        SidePanel::new(Side::Left, "").show(egui_contexts.ctx_mut(), |_ui| {

        });
    });

    app.run();
}