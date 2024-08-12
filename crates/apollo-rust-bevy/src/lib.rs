pub mod apollo_bevy_utils;

use std::path::PathBuf;
use std::sync::Arc;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_egui::egui::panel::Side;
use bevy_egui::egui::SidePanel;
use bevy_mod_outline::{AsyncSceneInheritOutlinePlugin, AutoGenerateOutlineNormalsPlugin, OutlinePlugin};
use bevy_obj::ObjPlugin;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robotics::Chain;
use crate::apollo_bevy_utils::camera::CameraSystems;
use crate::apollo_bevy_utils::chain::{BevySpawnChainMeshes, chain_sliders_egui, chain_state_updater_loop, ChainLinkMesh, ChainMeshesRepresentation, ChainStates};
use crate::apollo_bevy_utils::egui::{CursorIsOverEgui, reset_cursor_is_over_egui, set_cursor_is_over_egui_default};
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::viewport_visuals::ViewportVisualsActions;

pub trait ApolloBevyTrait {
    fn apollo_bevy_base(self) -> Self;
    fn apollo_bevy_pan_orbit_camera(self) -> Self;
    fn apollo_bevy_pan_orbit_three_style_camera(self) -> Self;
    fn apollo_bevy_starter_lights(self) -> Self;
    fn apollo_bevy_robotics_scene_visuals_start(self) -> Self;
}
impl ApolloBevyTrait for App {
    fn apollo_bevy_base(self) -> Self {
        let mut out = App::from(self);

        out
            .insert_resource(ClearColor(Color::srgb(0.9, 0.9, 0.92)))
            .insert_resource(CursorIsOverEgui(false))
            .insert_resource(Msaa::default())
            .add_plugins(DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "APOLLO TOOLBOX".to_string(),
                        ..Default::default()
                    }),
                    ..Default::default()
                })
            )
            .add_plugins(EguiPlugin)
            .add_plugins(ObjPlugin)
            .add_plugins((OutlinePlugin, AutoGenerateOutlineNormalsPlugin, AsyncSceneInheritOutlinePlugin))
            .add_systems(Last, reset_cursor_is_over_egui);

        out
    }

    fn apollo_bevy_pan_orbit_camera(self) -> Self {
        let mut out = App::from(self);

        out
            .add_systems(Startup, CameraSystems::system_spawn_pan_orbit_camera)
            .add_systems(PostUpdate, CameraSystems::system_pan_orbit_camera);

        out
    }

    fn apollo_bevy_pan_orbit_three_style_camera(self) -> Self {
        let mut out = App::from(self);

        out
            .add_systems(Startup, CameraSystems::system_spawn_pan_orbit_three_style_camera)
            .add_systems(PostUpdate, CameraSystems::system_pan_orbit_three_style_camera);

        out
    }

    fn apollo_bevy_starter_lights(self) -> Self {
        let mut out = App::from(self);

        out.add_systems(Startup, |mut commands: Commands| {
            commands.spawn(PointLightBundle {
                point_light: PointLight {
                    ..default()
                },
                transform: Transform::from_xyz(4.0, 4.0, 4.0),
                ..default()
            });
            commands.spawn(PointLightBundle {
                point_light: PointLight {
                    ..default()
                },
                transform: Transform::from_xyz(1.0, 2.0, -4.0),
                ..default()
            });
        });

        out
    }

    fn apollo_bevy_robotics_scene_visuals_start(self) -> Self {
        let mut out = App::from(self);

        out
            .add_systems(Startup, |mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
                ViewportVisualsActions::action_draw_robotics_grid(&mut commands, &mut meshes, &mut materials);
            });

        out
    }
}


pub trait ApolloChainBevyTrait {
    fn bevy_display(&self, path_to_bevy_assets: &PathBuf) {
        self.bevy_display_app(path_to_bevy_assets).run();
    }

    fn bevy_display_app(&self, path_to_bevy_assets: &PathBuf) -> App;
}
impl ApolloChainBevyTrait for Chain {
    fn bevy_display_app(&self, path_to_bevy_assets: &PathBuf) -> App {
        let mut app = App::new()
            .apollo_bevy_base()
            .apollo_bevy_starter_lights()
            .apollo_bevy_robotics_scene_visuals_start()
            .apollo_bevy_pan_orbit_three_style_camera();

        let chain1 = Arc::new(self.clone());
        let chain2 = chain1.clone();
        let chain3 = chain1.clone();
        let num_dofs = chain1.dof_module().num_dofs;
        let zeros_state = V::new(&vec![0.0; num_dofs]);
        app.insert_resource(ChainStates { states: vec![zeros_state.clone()] });

        let c = BevySpawnChainMeshes {
            chain_instance_idx: 0,
            chain_meshes_representation: ChainMeshesRepresentation::Plain,
            mesh_type: MeshType::GLB,
            chain: chain1,
            path_to_bevy_assets: path_to_bevy_assets.clone(),
            state: zeros_state.clone(),
        };

        app.add_systems(Startup, c.get_system());

        app.add_systems(PostUpdate, move |mut query: Query<(&mut Transform, &ChainLinkMesh)>, chain_states: Res<ChainStates>| {
            chain_state_updater_loop(&chain2, &mut query, &chain_states);
        });

        app.add_systems(Update, move |mut robot_states: ResMut<ChainStates>, mut egui_contexts: EguiContexts, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, window_query: Query<&Window, With<PrimaryWindow>>| {
            SidePanel::new(Side::Left, "side").show(egui_contexts.ctx_mut(), |ui| {
                chain_sliders_egui(0, &chain3, ui, &mut robot_states);
                set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &window_query);
            });
        });

        app
    }
}