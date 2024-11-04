use ad_trait::AD;
use nalgebra::{Isometry3, IsometryMatrix3};
use crate::ad::quaternions::{ApolloUnitQuaternionADTrait, UQAD};
use crate::ad::rotation_matrices::{ApolloRotation3ADTrait, R3AD};
use crate::ad::translations::{ApolloTranslation3AD, T3AD};
use crate::ad::vectors::V3AD;

/// Alias for `Isometry3<A>`, representing a 3D isometry with AD-compatible elements.
pub type I3AD<A> = Isometry3<A>;

/// Alias for `IsometryMatrix3<A>`, representing a 3D isometry matrix with AD-compatible elements.
pub type I3MAD<A> = IsometryMatrix3<A>;

/// Trait for operations on 3D isometries (`I3AD<A>`).
pub trait ApolloIsometry3ADTrait<A: AD> {
    fn new_random_with_range_ad(min: f64, max: f64) -> Self;
    fn from_slices_scaled_axis_ad(translation: &[A], scaled_axis: &[A]) -> Self;
    fn from_slices_euler_angles_ad(translation: &[A], euler_angles: &[A]) -> Self;
    fn from_slices_quaternion_ad(translation: &[A], quaternion: &[A]) -> Self;
    fn from_slice_scaled_axis_ad(slice: &[A]) -> Self;
    fn from_slice_euler_angles_ad(slice: &[A]) -> Self;
    fn from_slice_quaternion_ad(slice: &[A]) -> Self;
    fn to_isometry_matrix_3_ad(&self) -> I3MAD<A>;
    fn is_identity_ad(&self) -> bool;
    fn add_tiny_bit_of_noise_ad(&self) -> Self;
}

impl<A: AD> ApolloIsometry3ADTrait<A> for I3AD<A> {
    fn new_random_with_range_ad(min: f64, max: f64) -> Self {
        let t = T3AD::new_ad_random_with_range(min, max);
        let uq = UQAD::new_ad_random_with_range(min, max);
        Self::from_parts(t, uq)
    }

    fn from_slices_scaled_axis_ad(translation: &[A], scaled_axis: &[A]) -> Self {
        Self::new(V3AD::from_column_slice(translation), V3AD::from_column_slice(scaled_axis))
    }

    fn from_slices_euler_angles_ad(translation: &[A], euler_angles: &[A]) -> Self {
        Self::from_parts(T3AD::from_slice_ad(translation), UQAD::from_slice_euler_angles_ad(euler_angles))
    }

    fn from_slices_quaternion_ad(translation: &[A], quaternion: &[A]) -> Self {
        assert_eq!(quaternion.len(), 4);
        let t = T3AD::from_slice_ad(translation);
        let uq = UQAD::from_slice_quaternion_ad(quaternion);
        Self::from_parts(t, uq)
    }

    fn from_slice_scaled_axis_ad(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let scaled_axis = &slice[3..6];
        Self::from_slices_scaled_axis_ad(translation, scaled_axis)
    }

    fn from_slice_euler_angles_ad(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let euler_angles = &slice[3..6];
        Self::from_slices_euler_angles_ad(translation, euler_angles)
    }

    fn from_slice_quaternion_ad(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 7);
        let translation = &slice[0..3];
        let quaternion = &slice[3..7];
        Self::from_slices_quaternion_ad(translation, quaternion)
    }

    fn to_isometry_matrix_3_ad(&self) -> I3MAD<A> {
        let new_rot = self.rotation.to_rotation_matrix();
        I3MAD::from_parts(self.translation, new_rot)
    }

    fn is_identity_ad(&self) -> bool {
        self.translation.vector.is_identity(A::constant(0.0000001)) && self.rotation == UQAD::identity()
    }

    fn add_tiny_bit_of_noise_ad(&self) -> Self {
        let r = Self::new_random_with_range_ad(-0.00005, 0.00005);
        *self * r
    }
}

/// Trait for operations on 3D isometry matrices (`I3MAD<A>`).
pub trait ApolloIsometryMatrix3ADTrait<A: AD> {
    fn new_random_with_range_ad(min: f64, max: f64) -> Self;
    fn from_slices_scaled_axis_ad(translation: &[A], scaled_axis: &[A]) -> Self;
    fn from_slices_euler_angles_ad(translation: &[A], euler_angles: &[A]) -> Self;
    fn from_slices_quaternion_ad(translation: &[A], quaternion: &[A]) -> Self;
    fn from_slice_scaled_axis_ad(slice: &[A]) -> Self;
    fn from_slice_euler_angles_ad(slice: &[A]) -> Self;
    fn from_slice_quaternion_ad(slice: &[A]) -> Self;
    fn to_isometry_3_ad(&self) -> I3AD<A>;
    fn is_identity_ad(&self) -> bool;
}

impl<A: AD> ApolloIsometryMatrix3ADTrait<A> for I3MAD<A> {
    fn new_random_with_range_ad(min: f64, max: f64) -> Self {
        let t = T3AD::new_ad_random_with_range(min, max);
        let r3 = R3AD::new_ad_random();
        Self::from_parts(t, r3)
    }

    fn from_slices_scaled_axis_ad(translation: &[A], scaled_axis: &[A]) -> Self {
        Self::new(V3AD::from_column_slice(translation), V3AD::from_column_slice(scaled_axis))
    }

    fn from_slices_euler_angles_ad(translation: &[A], euler_angles: &[A]) -> Self {
        Self::from_parts(T3AD::from_slice_ad(translation), R3AD::from_slice_euler_angles_ad(euler_angles))
    }

    fn from_slices_quaternion_ad(translation: &[A], quaternion: &[A]) -> Self {
        assert_eq!(quaternion.len(), 4);
        let t = T3AD::from_slice_ad(translation);
        let r3 = R3AD::from_slice_quaternion_ad(quaternion);
        Self::from_parts(t, r3)
    }

    fn from_slice_scaled_axis_ad(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let scaled_axis = &slice[3..6];
        Self::from_slices_scaled_axis_ad(translation, scaled_axis)
    }

    fn from_slice_euler_angles_ad(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let euler_angles = &slice[3..6];
        Self::from_slices_euler_angles_ad(translation, euler_angles)
    }

    fn from_slice_quaternion_ad(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 7);
        let translation = &slice[0..3];
        let quaternion = &slice[3..7];
        Self::from_slices_quaternion_ad(translation, quaternion)
    }

    fn to_isometry_3_ad(&self) -> I3AD<A> {
        let new_rot = self.rotation.to_unit_quaternion_ad();
        I3AD::from_parts(self.translation, new_rot)
    }

    fn is_identity_ad(&self) -> bool {
        self.translation.vector.is_identity(A::constant(0.0000001)) && self.rotation == R3AD::identity()
    }
}
