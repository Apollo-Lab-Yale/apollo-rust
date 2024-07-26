use std::path::PathBuf;
use bevy::app::{App, Startup};
use bevy::asset::AssetServer;
use bevy::prelude::{Commands, Query, Res, Transform, Update, With};
use apollo_rust_bevy::apollo_bevy_utils::gltf::{GLTF, spawn_gltf};
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main () {
    let mut app = App::new()
        .apollo_bevy_base()
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_starter_lights()
        .apollo_bevy_robotics_scene_visuals_start();

    let t = ISE3q::new(I3::from_slices_euler_angles(&[0.,0.,0.], &[0.0,0.,0.]));
    let assets_path = PathBuf::new_from_documents_dir().append("apollo-rust/crates/apollo-rust-bevy/assets");
    let mesh_path = PathBuf::new_from_default_apollo_robots_dir().append("ur5/mesh_modules/plain_meshes_module/meshes/glb/forearm.glb");
    let a = assets_path.get_a_to_b_path(&mesh_path);

    app.add_systems(Startup, move |mut commands: Commands, asset_server: Res<AssetServer>| {
        let e = spawn_gltf(a.clone(), Some(&t), &mut commands, &asset_server);
        commands.get_entity(e).unwrap().insert(GLTF);
    });
    app.add_systems(Update, move |mut query: Query<&mut Transform, With<GLTF>>| {
        query.iter_mut().for_each(|mut x| {
            // x.translation.y = 2.0;
        });
    });

    app.run();
}