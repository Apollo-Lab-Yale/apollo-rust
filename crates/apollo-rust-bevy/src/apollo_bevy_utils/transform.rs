use bevy::math::{Quat, Vec3};
use bevy::prelude::Transform;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;

pub struct TransformUtils;
impl TransformUtils {
    #[inline(always)]
    pub fn util_convert_pose_to_y_up_bevy_transform(pose: &ISE3q) -> Transform {
        let pose_new = ISE3q::new(I3::from_slices_euler_angles(&[0.0; 3], &[-std::f64::consts::FRAC_PI_2, 0.0, 0.0])).group_operator(pose);
        let t = pose_new.0.translation.vector.xyz();
        let r = pose_new.0.rotation;

        return Transform {
            translation: Vec3::new( t.x as f32, t.y as f32, t.z as f32 ),
            rotation: Quat::from_xyzw( r.i as f32, r.j as f32, r.k as f32, r.w as f32 ),
            ..Default::default()
        }
    }

    pub fn util_convert_y_up_bevy_transform_to_pose(t: &Transform) -> ISE3q {
        let x = t.translation.x as f64;
        let y = t.translation.y as f64;
        let z = t.translation.z as f64;
        let qw = t.rotation.w as f64;
        let qx = t.rotation.x as f64;
        let qy = t.rotation.y as f64;
        let qz = t.rotation.z as f64;

        let pose = ISE3q::new(I3::from_slices_quaternion(&[x, y, z], &[qw, qx, qy, qz]));
        let pose_new = ISE3q::new(I3::from_slices_euler_angles(&[0.0; 3], &[std::f64::consts::FRAC_PI_2, 0.0, 0.0])).group_operator(&pose);

        return pose_new;
    }

    #[inline(always)]
    pub fn util_convert_z_up_vec3_to_y_up_bevy_vec3(vec: Vec3) -> Vec3 {
        return Vec3::new(vec.x, vec.z, -vec.y);
    }

    #[inline(always)]
    pub fn util_convert_bevy_y_up_vec3_to_z_up_vec3(vec: Vec3) -> Vec3 {
        return Vec3::new(vec.x, -vec.z, vec.y);
    }

    #[inline(always)]
    pub fn util_convert_z_up_v3_to_z_up_vec3(v3: V3) -> Vec3 {
        return Vec3::new( v3.x as f32, v3.y as f32, v3.z as f32 )
    }

    #[inline(always)]
    pub fn util_convert_z_up_v3_to_y_up_vec3(v3: V3) -> Vec3 {
        return Self::util_convert_z_up_vec3_to_y_up_bevy_vec3(Self::util_convert_z_up_v3_to_z_up_vec3(v3));
    }
}