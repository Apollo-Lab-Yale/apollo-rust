use bevy::app::App;
use apollo_rust_bevy::ApolloBevyTrait;



fn main () {
    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_starter_lights()
        .apollo_bevy_robotics_scene_visuals_start();

    app.run();
}