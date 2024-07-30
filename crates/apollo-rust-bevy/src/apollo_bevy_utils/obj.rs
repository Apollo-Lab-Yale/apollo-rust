use std::path::PathBuf;
use bevy::asset::AssetServer;
use bevy::color::Color;
use bevy::pbr::{PbrBundle, StandardMaterial};
use bevy::prelude::{Assets, Commands, Component, Entity, Res, ResMut, Transform};
use bevy::utils::default;
use bevy_mod_outline::{OutlineBundle, OutlineMode, OutlineVolume};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::transform::TransformUtils;

pub fn spawn_obj(file_path_relative_to_assets: PathBuf, color: Color, pose: Option<&ISE3q>, commands: &mut Commands, asset_server: &Res<AssetServer>, materials: &mut ResMut<Assets<StandardMaterial>>) -> Entity {
    file_path_relative_to_assets.verify_extension(&vec!["obj", "OBJ"]).expect("error");

    let mut res = commands.spawn(PbrBundle {
        mesh: asset_server.load(&file_path_relative_to_assets.to_str().expect("error").to_string()),
        material: materials.add(StandardMaterial::from_color(color)),
        transform: match pose {
            None => { Transform::default() }
            Some(pose) => { TransformUtils::util_convert_pose_to_y_up_bevy_transform(&pose.add_tiny_bit_of_noise()) }
        },
        ..default()
    });

    res.insert(OutlineBundle {
        outline: OutlineVolume {
            visible: true,
            width: 1.0,
            colour: Color::srgb(0.,0.,0.),
        },
        mode: OutlineMode::RealVertex,
        ..default()
    });

    res.insert(OBJ);

    res.id()
}

#[derive(Component)]
pub struct OBJ;