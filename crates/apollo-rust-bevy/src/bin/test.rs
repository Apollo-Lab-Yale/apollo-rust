use apollo_rust_bevy::ApolloBevyTrait;
use bevy::prelude::*;

fn main() {
    let _app = App::new()
        .apollo_bevy_base(true, false)
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_starter_lights()
        .apollo_bevy_robotics_scene_visuals_start();

    // println!("Apollo Bevy Base App Setup Check");
}
