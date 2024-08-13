pub mod apollo_bevy_utils;

use std::path::PathBuf;
use std::sync::Arc;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin};
use bevy_mod_outline::{AsyncSceneInheritOutlinePlugin, AutoGenerateOutlineNormalsPlugin, OutlinePlugin};
use bevy_obj::ObjPlugin;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robotics_core::Chain;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::camera::CameraSystems;
use crate::apollo_bevy_utils::chain::{BevyChainStateUpdaterLoop, BevySpawnChainMeshes, ChainMeshesRepresentation, ChainState};
use crate::apollo_bevy_utils::egui::{CursorIsOverEgui, reset_cursor_is_over_egui};
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::viewport_visuals::ViewportVisualsActions;

pub trait ApolloBevyTrait {
    fn apollo_bevy_base(self) -> Self;
    fn apollo_bevy_pan_orbit_camera(self) -> Self;
    fn apollo_bevy_pan_orbit_three_style_camera(self) -> Self;
    fn apollo_bevy_starter_lights(self) -> Self;
    fn apollo_bevy_robotics_scene_visuals_start(self) -> Self;
    fn apollo_bevy_spawn_robot(self, chain: &Chain, chain_instance_idx: usize, global_offset: ISE3q, mesh_specs: Vec<(ChainMeshesRepresentation, MeshType)>, path_to_bevy_assets: &PathBuf) -> Self;
}
impl ApolloBevyTrait for App {
    fn apollo_bevy_base(self) -> Self {
        let mut out = App::from(self);

        out
            .insert_resource(ClearColor(Color::srgb(0.9, 0.9, 0.92)))
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
            .insert_resource(CursorIsOverEgui(false))
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

    fn apollo_bevy_spawn_robot(self, chain: &Chain, chain_instance_idx: usize, global_offset: ISE3q, mesh_specs: Vec<(ChainMeshesRepresentation, MeshType)>, path_to_bevy_assets: &PathBuf) -> Self {
        let mut out = App::from(self);

        let chain_arc = Arc::new(chain.clone());
        let zero_state = V::new(&vec![0.0; chain_arc.num_dofs()]);
        let zero_state_clone = zero_state.clone();
        let global_offset_clone = global_offset.clone();

        out.add_systems(Startup, move |mut commands: Commands| {
            commands.spawn(ChainState {
                chain_instance_idx,
                state: zero_state_clone.clone(),
                global_offset: global_offset_clone.clone(),
            });
        });
        
        mesh_specs.iter().for_each(|(x, y)| {
            let c = BevySpawnChainMeshes {
                chain_instance_idx,
                chain_meshes_representation: x.clone(),
                mesh_type: y.clone(),
                chain: chain_arc.clone(),
                path_to_bevy_assets: path_to_bevy_assets.clone(),
                state: zero_state.clone(),
            };
            out.add_systems(Startup, c.get_system());
        });

        let c = BevyChainStateUpdaterLoop {
            chain_instance_idx,
            chain: chain_arc.clone(),
        };
        out.add_systems(PostUpdate, c.get_system());

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
    fn bevy_display_app(&self, _path_to_bevy_assets: &PathBuf) -> App {
        todo!()
    }
    /*
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

        let c = BevyChainStateUpdaterLoop {
            chain: chain2.clone()
        };
        app.add_systems(PostUpdate, c.get_system());

        app.add_systems(Update, move |mut robot_states: ResMut<ChainStates>, mut egui_contexts: EguiContexts| {
            SidePanel::new(Side::Left, "side").show(egui_contexts.ctx_mut(), |ui| {
                let mut b = BevyChainSlidersEgui {
                    chain_instance_idx: 0,
                    chain: chain3.clone(),
                    ui,
                };
                b.action_chain_sliders_egui(&mut robot_states);
            });
        });

        app
    }
    */
}