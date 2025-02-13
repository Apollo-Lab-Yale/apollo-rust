use ad_trait::AD;
use nalgebra::{Isometry3, IsometryMatrix3};
use crate::quaternions::{ApolloUnitQuaternionADTrait, UQ};
use crate::rotation_matrices::{ApolloRotation3Trait, R3};
use crate::translations::{ApolloTranslation3AD, T3};
use crate::vectors::V3;

/// Alias for `Isometry3<A>`, representing a 3D isometry with AD-compatible elements.
pub type I3<A> = Isometry3<A>;

/// Alias for `IsometryMatrix3<A>`, representing a 3D isometry matrix with AD-compatible elements.
pub type I3M<A> = IsometryMatrix3<A>;

/// Trait for operations on 3D isometries (`I3AD<A>`).
pub trait ApolloIsometry3Trait<A: AD> {
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn from_slices_scaled_axis(translation: &[A], scaled_axis: &[A]) -> Self;
    fn from_slices_euler_angles(translation: &[A], euler_angles: &[A]) -> Self;
    fn from_slices_quaternion(translation: &[A], quaternion: &[A]) -> Self;
    fn from_slice_scaled_axis(slice: &[A]) -> Self;
    fn from_slice_euler_angles(slice: &[A]) -> Self;
    fn from_slice_quaternion(slice: &[A]) -> Self;
    fn to_isometry_matrix_3(&self) -> I3M<A>;
    fn is_identity(&self) -> bool;
    fn add_tiny_bit_of_noise(&self) -> Self;
    fn to_other_ad_type<A2: AD>(&self) -> I3<A2>;
    fn to_constant_ad(&self) -> Self;
}
impl<A: AD> ApolloIsometry3Trait<A> for I3<A> {
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let t = T3::new_random_with_range(min, max);
        let uq = UQ::new_random_with_range(min, max);
        Self::from_parts(t, uq)
    }

    fn from_slices_scaled_axis(translation: &[A], scaled_axis: &[A]) -> Self {
        Self::new(V3::from_column_slice(translation), V3::from_column_slice(scaled_axis))
    }

    fn from_slices_euler_angles(translation: &[A], euler_angles: &[A]) -> Self {
        Self::from_parts(T3::from_slice(translation), UQ::from_slice_euler_angles(euler_angles))
    }

    fn from_slices_quaternion(translation: &[A], quaternion: &[A]) -> Self {
        assert_eq!(quaternion.len(), 4);
        let t = T3::from_slice(translation);
        let uq = UQ::from_slice_quaternion(quaternion);
        Self::from_parts(t, uq)
    }

    fn from_slice_scaled_axis(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let scaled_axis = &slice[3..6];
        Self::from_slices_scaled_axis(translation, scaled_axis)
    }

    fn from_slice_euler_angles(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let euler_angles = &slice[3..6];
        Self::from_slices_euler_angles(translation, euler_angles)
    }

    fn from_slice_quaternion(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 7);
        let translation = &slice[0..3];
        let quaternion = &slice[3..7];
        Self::from_slices_quaternion(translation, quaternion)
    }

    fn to_isometry_matrix_3(&self) -> I3M<A> {
        let new_rot = self.rotation.to_rotation_matrix();
        I3M::from_parts(self.translation, new_rot)
    }

    fn is_identity(&self) -> bool {
        self.translation.vector.is_identity(A::constant(0.0000001)) && self.rotation == UQ::identity()
    }

    fn add_tiny_bit_of_noise(&self) -> Self {
        let r = Self::new_random_with_range(-0.00005, 0.00005);
        *self * r
    }

    fn to_other_ad_type<A2: AD>(&self) -> I3<A2> {
        I3::from_slices_quaternion(self.translation.to_other_ad_type::<A2>().vector.as_slice(), self.rotation.to_other_ad_type::<A2>().coords.as_slice())
    }

    fn to_constant_ad(&self) -> Self {
        I3::from_slices_quaternion(self.translation.to_constant_ad().vector.as_slice(), self.rotation.to_constant_ad().coords.as_slice())
    }
}

/// Trait for operations on 3D isometry matrices (`I3MAD<A>`).
pub trait ApolloIsometryMatrix3Trait<A: AD> {
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn from_slices_scaled_axis(translation: &[A], scaled_axis: &[A]) -> Self;
    fn from_slices_euler_angles(translation: &[A], euler_angles: &[A]) -> Self;
    fn from_slices_quaternion(translation: &[A], quaternion: &[A]) -> Self;
    fn from_slice_scaled_axis(slice: &[A]) -> Self;
    fn from_slice_euler_angles(slice: &[A]) -> Self;
    fn from_slice_quaternion(slice: &[A]) -> Self;
    fn to_isometry_3(&self) -> I3<A>;
    fn is_identity(&self) -> bool;
    fn to_other_ad_type<A2: AD>(&self) -> I3M<A2>;
    fn to_constant_ad(&self) -> Self;
}
impl<A: AD> ApolloIsometryMatrix3Trait<A> for I3M<A> {
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let t = T3::new_random_with_range(min, max);
        let r3 = R3::new_random();
        Self::from_parts(t, r3)
    }

    fn from_slices_scaled_axis(translation: &[A], scaled_axis: &[A]) -> Self {
        Self::new(V3::from_column_slice(translation), V3::from_column_slice(scaled_axis))
    }

    fn from_slices_euler_angles(translation: &[A], euler_angles: &[A]) -> Self {
        Self::from_parts(T3::from_slice(translation), R3::from_slice_euler_angles(euler_angles))
    }

    fn from_slices_quaternion(translation: &[A], quaternion: &[A]) -> Self {
        assert_eq!(quaternion.len(), 4);
        let t = T3::from_slice(translation);
        let r3 = R3::from_slice_quaternion(quaternion);
        Self::from_parts(t, r3)
    }

    fn from_slice_scaled_axis(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let scaled_axis = &slice[3..6];
        Self::from_slices_scaled_axis(translation, scaled_axis)
    }

    fn from_slice_euler_angles(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 6);
        let translation = &slice[0..3];
        let euler_angles = &slice[3..6];
        Self::from_slices_euler_angles(translation, euler_angles)
    }

    fn from_slice_quaternion(slice: &[A]) -> Self {
        assert_eq!(slice.len(), 7);
        let translation = &slice[0..3];
        let quaternion = &slice[3..7];
        Self::from_slices_quaternion(translation, quaternion)
    }

    fn to_isometry_3(&self) -> I3<A> {
        let new_rot = self.rotation.to_unit_quaternion();
        I3::from_parts(self.translation, new_rot)
    }

    fn is_identity(&self) -> bool {
        self.translation.vector.is_identity(A::constant(0.0000001)) && self.rotation == R3::identity()
    }

    fn to_other_ad_type<A2: AD>(&self) -> I3M<A2> {
        I3M::from_slices_quaternion(self.translation.to_other_ad_type::<A2>().vector.as_slice(), self.rotation.to_unit_quaternion().to_other_ad_type::<A2>().coords.as_slice())
    }

    fn to_constant_ad(&self) -> Self {
        I3M::from_slices_quaternion(self.translation.to_constant_ad().vector.as_slice(), self.rotation.to_unit_quaternion().to_constant_ad().coords.as_slice())
    }
}
