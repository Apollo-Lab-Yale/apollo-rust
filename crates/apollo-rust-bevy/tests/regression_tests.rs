use apollo_rust_bevy::apollo_bevy_utils::transform::TransformUtils;
use apollo_rust_bevy::apollo_bevy_utils::visibility::VisibilityChangeEngine;
use apollo_rust_bevy::{get_default_mesh_specs, ApolloBevyTrait};
use apollo_rust_lie::LieGroupElement;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use bevy::prelude::*;
use transform_gizmo_bevy::GizmoOptions;

#[test]
fn test_transform_utils_vec3_conversions() {
    let z_up = Vec3::new(1.0, 2.0, 3.0);
    // util_convert_z_up_vec3_to_y_up_bevy_vec3: (x, z, -y)
    let y_up = TransformUtils::util_convert_z_up_vec3_to_y_up_bevy_vec3(z_up);
    assert_eq!(y_up, Vec3::new(1.0, 3.0, -2.0));

    let z_up_back = TransformUtils::util_convert_bevy_y_up_vec3_to_z_up_vec3(y_up);
    assert_eq!(z_up, z_up_back);
}

#[test]
fn test_transform_utils_pose_conversions() {
    let original_pose = ISE3q::new(I3::from_slices_euler_angles(
        &[1.0, 2.0, 3.0],
        &[0.1, 0.2, 0.3],
    ));

    let bevy_transform = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&original_pose);
    let back_to_pose = TransformUtils::util_convert_y_up_bevy_transform_to_pose(&bevy_transform);

    // Check translation
    assert!(
        (original_pose.0.translation.vector.x - back_to_pose.0.translation.vector.x).abs() < 1e-6
    );
    assert!(
        (original_pose.0.translation.vector.y - back_to_pose.0.translation.vector.y).abs() < 1e-6
    );
    assert!(
        (original_pose.0.translation.vector.z - back_to_pose.0.translation.vector.z).abs() < 1e-6
    );

    // Check rotation
    let diff = original_pose.group_operator(&back_to_pose.inverse());
    let angle = diff.0.rotation.to_rotation_matrix().angle();
    assert!(angle.abs() < 1e-6);
}

#[test]
fn test_apollo_bevy_base_setup() {
    let app = App::new().apollo_bevy_base(true, true);

    // Check if expected resources are inserted
    assert!(app.world().get_resource::<ClearColor>().is_some());
    assert!(app.world().get_resource::<GizmoOptions>().is_some());
    assert!(app
        .world()
        .get_resource::<VisibilityChangeEngine>()
        .is_some());
}

#[test]
fn test_apollo_bevy_robotics_base_setup() {
    let app = App::new()
        .apollo_bevy_base(true, true)
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_robotics_scene_visuals_start()
        .apollo_bevy_starter_lights();

    // robotics_base includes apollo_bevy_base, camera systems, etc.
    assert!(app.world().get_resource::<ClearColor>().is_some());
}

#[test]
fn test_get_default_mesh_specs() {
    let specs = get_default_mesh_specs();
    assert!(!specs.is_empty());
}
