use std::path::PathBuf;
use std::sync::Arc;
use bevy::app::{App, Startup};
use bevy::asset::Assets;
use bevy::pbr::StandardMaterial;
use bevy::prelude::{AssetServer, Commands, Query, Res, ResMut, Transform, Update, Window as Window1, With};
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts};
use bevy_egui::egui::panel::Side;
use bevy_egui::egui::{SidePanel, Window as Window2};
use apollo_rust_bevy::apollo_bevy_utils::egui::{CursorIsOverEgui, set_cursor_is_over_egui_default};
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_bevy::apollo_bevy_utils::chain::{ChainMeshesRepresentation, spawn_chain_meshes, ChainStates, ChainLinkMesh, chain_state_updater_loop, chain_sliders_egui, BevySpawnChainMeshes};
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainFromName;

fn main () {
    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_starter_lights()
        .apollo_bevy_robotics_scene_visuals_start();

    app.insert_resource(ChainStates { states: vec![V::new(&[0.0; 6])] });

    let robot = Arc::new(ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_robots_dir()).to_chain("ur5"));
    let robot1 = robot.clone();
    let robot2 = robot.clone();
    let robot3 = robot.clone();

    // let path_to_assets = PathBuf::new_from_default_apollo_bevy_assets_dir();
    // app.add_systems(Startup,  move |mut commands: Commands, asset_server: Res<AssetServer>, mut materials: ResMut<Assets<StandardMaterial>>| {
    //     spawn_chain_meshes(0, ChainMeshesRepresentation::Plain, MeshType::OBJ, &robot1, &V::new(&[0.0; 6]), &path_to_assets, &mut commands, &asset_server, &mut materials);
    //     // spawn_robot_meshes(0, RobotMeshesRepresentation::ConvexDecomposition, MeshType::GLB, &robot1, &V::new(&[0.0; 7]), &path_to_assets, &mut commands, &asset_server, &mut materials);
    // });
    let c = BevySpawnChainMeshes {
        chain_instance_idx: 0,
        chain_meshes_representation: ChainMeshesRepresentation::Plain,
        mesh_type: MeshType::OBJ,
        chain: robot1,
        path_to_bevy_assets: PathBuf::new_from_default_apollo_bevy_assets_dir(),
        state: V::new(&[0.0; 6])
    };
    app.add_systems(Startup, c.get_system());

    app.add_systems(Update, move |mut query: Query<(&mut Transform, &ChainLinkMesh)>, robot_states: Res<ChainStates>| {
        chain_state_updater_loop(&robot2, &mut query, &robot_states);
    });

    app.add_systems(Update, move |mut robot_states: ResMut<ChainStates>, mut egui_contexts: EguiContexts, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, window_query: Query<&Window1, With<PrimaryWindow>>| {
        Window2::new("hello").show(egui_contexts.ctx_mut(), |ui| {
            chain_sliders_egui(0, &robot3, ui, &mut robot_states);
            set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &window_query);
        });

        SidePanel::new(Side::Left, "").show(egui_contexts.ctx_mut(), |_ui| {

        });
    });

    app.run();
}