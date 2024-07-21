pub mod apollo_bevy_utils;

use bevy::prelude::*;
use crate::apollo_bevy_utils::camera::CameraSystems;
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
            .insert_resource(ClearColor(Color::srgb(0.45, 0.45, 0.5)))
            .insert_resource(Msaa::default())
            .add_plugins(DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "APOLLO TOOLBOX".to_string(),
                        ..Default::default()
                    }),
                    ..Default::default()
                })
            );

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
                    intensity: 1500.0,
                    ..default()
                },
                transform: Transform::from_xyz(4.0, 4.0, 4.0),
                ..default()
            });
            commands.spawn(PointLightBundle {
                point_light: PointLight {
                    intensity: 1500.0,
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