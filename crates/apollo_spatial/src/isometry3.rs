use nalgebra::{Isometry3, IsometryMatrix3};
use crate::quaternions::{ApolloUnitQuaternionTrait, UQ};
use crate::rotation_matrices::{ApolloRotation3Trait, R3};
use crate::translations::{ApolloTranslation3, T3};
use crate::vectors::V3;

pub type I3 = Isometry3<f64>;
pub type I3M = IsometryMatrix3<f64>;


pub trait ApolloIsometry3Trait {
    /// range will just be for translation part
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self;
    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self;
    fn from_slices_quaternion(translation: &[f64], quaternion: &[f64]) -> Self;
    fn from_slice_scaled_axis(slice: &[f64]) -> Self;
    fn from_slice_euler_angles(slice: &[f64]) -> Self;
    fn from_slice_quaternion(slice: &[f64]) -> Self;
    fn to_isometry_matrix_3(&self) -> I3M;
}
impl ApolloIsometry3Trait for I3 {
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let t = T3::new_random_with_range(min, max);
        let uq = UQ::new_random();
        Self::from_parts(t, uq)
    }

    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self {
        Self::new(V3::from_column_slice(translation), V3::from_column_slice(scaled_axis))
    }

    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self {
        Self::from_parts(T3::from_slice(translation), UQ::from_slice_euler_angles(euler_angles))
    }

    /// quaternion should be [w, x, y, z]
    fn from_slices_quaternion(translation: &[f64], quaternion: &[f64]) -> Self {
        assert_eq!(quaternion.len(), 4);
        let t = T3::from_slice(translation);
        let uq = UQ::from_slice_quaternion(quaternion);
        Self::from_parts(t, uq)
    }

    fn from_slice_scaled_axis(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = [slice[0], slice[1], slice[2]];
        let scaled_axis = [slice[3], slice[4], slice[5]];

        Self::from_slices_scaled_axis(&translation, &scaled_axis)
    }

    fn from_slice_euler_angles(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = [slice[0], slice[1], slice[2]];
        let euler_angles = [slice[3], slice[4], slice[5]];

        Self::from_slices_euler_angles(&translation, &euler_angles)
    }

    fn from_slice_quaternion(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 7);
        let translation = [slice[0], slice[1], slice[2]];
        let quaternion = [slice[3], slice[4], slice[5], slice[6]];

        Self::from_slices_quaternion(&translation, &quaternion)
    }

    #[inline(always)]
    fn to_isometry_matrix_3(&self) -> I3M {
        let new_rot = self.rotation.to_rotation_matrix();
        I3M::from_parts(self.translation, new_rot)
    }
}


pub trait ApolloIsometryMatrix3Trait {
    /// range will just be for translation part
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self;
    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self;
    fn from_slices_quaternion(translation: &[f64], quaternion: &[f64]) -> Self;
    fn from_slice_scaled_axis(slice: &[f64]) -> Self;
    fn from_slice_euler_angles(slice: &[f64]) -> Self;
    fn from_slice_quaternion(slice: &[f64]) -> Self;
    fn to_isometry_3(&self) -> I3;
}
impl ApolloIsometryMatrix3Trait for I3M {
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let t = T3::new_random_with_range(min, max);
        let r3 = R3::new_random();
        Self::from_parts(t, r3)
    }

    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self {
        Self::new(V3::from_column_slice(translation), V3::from_column_slice(scaled_axis))
    }

    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self {
        Self::from_parts(T3::from_slice(translation), R3::from_slice_euler_angles(euler_angles))
    }

    fn from_slices_quaternion(translation: &[f64], quaternion: &[f64]) -> Self {
        assert_eq!(quaternion.len(), 4);
        let t = T3::from_slice(translation);
        let r3 = R3::from_slice_quaternion(quaternion);
        Self::from_parts(t, r3)
    }

    fn from_slice_scaled_axis(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = [slice[0], slice[1], slice[2]];
        let scaled_axis = [slice[3], slice[4], slice[5]];

        Self::from_slices_scaled_axis(&translation, &scaled_axis)
    }

    fn from_slice_euler_angles(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = [slice[0], slice[1], slice[2]];
        let euler_angles = [slice[3], slice[4], slice[5]];

        Self::from_slices_euler_angles(&translation, &euler_angles)
    }

    fn from_slice_quaternion(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 7);
        let translation = [slice[0], slice[1], slice[2]];
        let quaternion = [slice[3], slice[4], slice[5], slice[6]];

        Self::from_slices_quaternion(&translation, &quaternion)
    }

    #[inline(always)]
    fn to_isometry_3(&self) -> I3 {
        let new_rot = self.rotation.to_unit_quaternion();
        I3::from_parts(self.translation, new_rot)
    }
}
