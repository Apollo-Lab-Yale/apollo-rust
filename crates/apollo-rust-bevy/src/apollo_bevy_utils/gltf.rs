use std::path::PathBuf;
use bevy::gltf::GltfAssetLabel;
use bevy::prelude::{AssetServer, Commands, Component, Entity, Res, Transform};
use bevy::scene::SceneBundle;
use bevy::utils::default;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::transform::TransformUtils;

pub fn spawn_gltf(file_path_relative_to_assets: PathBuf, pose: Option<&ISE3q>, commands: &mut Commands, asset_server: &Res<AssetServer>) -> Entity {
    file_path_relative_to_assets.verify_extension(&vec!["gltf", "GLTF", "glb", "GLB"]).expect("error");

    let mut res = commands.spawn(SceneBundle {
        scene:asset_server.load(GltfAssetLabel::Scene(0).from_asset(file_path_relative_to_assets)),
        transform: match pose {
            None => { Transform::default() }
            Some(pose) => { TransformUtils::util_convert_pose_to_y_up_bevy_transform(&pose.add_tiny_bit_of_noise()) }
        },
        ..default()
    });

    res.insert(GLTF);

    res.id()
}

#[derive(Component)]
pub struct GLTF;