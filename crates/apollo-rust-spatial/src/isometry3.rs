use crate::quaternions::{ApolloUnitQuaternionTrait, UQ};
use crate::rotation_matrices::{ApolloRotation3Trait, R3};
use crate::translations::{ApolloTranslation3, T3};
use crate::vectors::V3;
use nalgebra::{Isometry3, IsometryMatrix3};

/// Alias for `Isometry3<f64>`, representing a 3D isometry with 64-bit floating point precision.
pub type I3 = Isometry3<f64>;

/// Alias for `IsometryMatrix3<f64>`, representing a 3D isometry matrix with 64-bit floating point precision.
pub type I3M = IsometryMatrix3<f64>;

/// Trait for operations on 3D isometries (`I3`).
pub trait ApolloIsometry3Trait {
    /// Generates a new random `I3` instance with the translation part within the specified range.
    ///
    /// - `min`: Minimum bound for the random values.
    /// - `max`: Maximum bound for the random values.
    fn new_random_with_range(min: f64, max: f64) -> Self;

    /// Constructs an `I3` instance from translation and scaled axis slices.
    ///
    /// - `translation`: A slice representing the translation vector.
    /// - `scaled_axis`: A slice representing the rotation as a scaled axis.
    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self;

    /// Constructs an `I3` instance from translation and Euler angles slices.
    ///
    /// - `translation`: A slice representing the translation vector.
    /// - `euler_angles`: A slice representing the Euler angles (rotation).
    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self;

    /// Constructs an `I3` instance from translation and quaternion slices.
    ///
    /// - `translation`: A slice representing the translation vector.
    /// - `quaternion`: A slice representing the quaternion `[w, x, y, z]` for rotation.
    fn from_slices_quaternion(translation: &[f64], quaternion: &[f64]) -> Self;

    /// Constructs an `I3` instance from a single slice representing both translation and scaled axis.
    ///
    /// - `slice`: A slice representing the translation and scaled axis vectors.
    fn from_slice_scaled_axis(slice: &[f64]) -> Self;

    /// Constructs an `I3` instance from a single slice representing both translation and Euler angles.
    ///
    /// - `slice`: A slice representing the translation and Euler angles.
    fn from_slice_euler_angles(slice: &[f64]) -> Self;

    /// Constructs an `I3` instance from a single slice representing both translation and quaternion.
    ///
    /// - `slice`: A slice representing the translation and quaternion.
    fn from_slice_quaternion(slice: &[f64]) -> Self;

    /// Converts the `I3` instance to an `I3M` (IsometryMatrix3) instance.
    fn to_isometry_matrix_3(&self) -> I3M;

    /// Checks if the `I3` instance is an identity transformation.
    ///
    /// Returns `true` if the transformation is an identity, `false` otherwise.
    fn is_identity(&self) -> bool;

    /// Adds a small amount of noise to the `I3` instance and returns the new noisy transformation.
    ///
    /// The noise is uniformly distributed in the range `[-0.00005, 0.00005]`.
    fn add_tiny_bit_of_noise(&self) -> Self;
}

impl ApolloIsometry3Trait for I3 {
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let t = T3::new_random_with_range(min, max);
        let uq = UQ::new_random_with_range(min, max);
        Self::from_parts(t, uq)
    }

    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self {
        Self::new(
            V3::from_column_slice(translation),
            V3::from_column_slice(scaled_axis),
        )
    }

    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self {
        Self::from_parts(
            T3::from_slice(translation),
            UQ::from_slice_euler_angles(euler_angles),
        )
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

    #[inline(always)]
    fn is_identity(&self) -> bool {
        return self.translation.vector.norm() < 0.0000001 && self.rotation == UQ::identity();
    }

    #[inline(always)]
    fn add_tiny_bit_of_noise(&self) -> Self {
        let r = Self::new_random_with_range(-0.00005, 0.00005);
        return self * r;
    }
}

/// Trait for operations on 3D isometry matrices (`I3M`).
pub trait ApolloIsometryMatrix3Trait {
    /// Generates a new random `I3M` instance with the translation part within the specified range.
    ///
    /// - `min`: Minimum bound for the random values.
    /// - `max`: Maximum bound for the random values.
    fn new_random_with_range(min: f64, max: f64) -> Self;

    /// Constructs an `I3M` instance from translation and scaled axis slices.
    ///
    /// - `translation`: A slice representing the translation vector.
    /// - `scaled_axis`: A slice representing the rotation as a scaled axis.
    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self;

    /// Constructs an `I3M` instance from translation and Euler angles slices.
    ///
    /// - `translation`: A slice representing the translation vector.
    /// - `euler_angles`: A slice representing the Euler angles (rotation).
    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self;

    /// Constructs an `I3M` instance from translation and quaternion slices.
    ///
    /// - `translation`: A slice representing the translation vector.
    /// - `quaternion`: A slice representing the quaternion `[w, x, y, z]` for rotation.
    fn from_slices_quaternion(translation: &[f64], quaternion: &[f64]) -> Self;

    /// Constructs an `I3M` instance from a single slice representing both translation and scaled axis.
    ///
    /// - `slice`: A slice representing the translation and scaled axis vectors.
    fn from_slice_scaled_axis(slice: &[f64]) -> Self;

    /// Constructs an `I3M` instance from a single slice representing both translation and Euler angles.
    ///
    /// - `slice`: A slice representing the translation and Euler angles.
    fn from_slice_euler_angles(slice: &[f64]) -> Self;

    /// Constructs an `I3M` instance from a single slice representing both translation and quaternion.
    ///
    /// - `slice`: A slice representing the translation and quaternion.
    fn from_slice_quaternion(slice: &[f64]) -> Self;

    /// Converts the `I3M` instance to an `I3` (Isometry3) instance.
    fn to_isometry_3(&self) -> I3;

    /// Checks if the `I3M` instance is an identity transformation.
    ///
    /// Returns `true` if the transformation is an identity, `false` otherwise.
    fn is_identity(&self) -> bool;
}

impl ApolloIsometryMatrix3Trait for I3M {
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let t = T3::new_random_with_range(min, max);
        let r3 = R3::new_random();
        Self::from_parts(t, r3)
    }

    fn from_slices_scaled_axis(translation: &[f64], scaled_axis: &[f64]) -> Self {
        Self::new(
            V3::from_column_slice(translation),
            V3::from_column_slice(scaled_axis),
        )
    }

    fn from_slices_euler_angles(translation: &[f64], euler_angles: &[f64]) -> Self {
        Self::from_parts(
            T3::from_slice(translation),
            R3::from_slice_euler_angles(euler_angles),
        )
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

    #[inline(always)]
    fn is_identity(&self) -> bool {
        return self.translation.vector.is_identity(0.0000001) && self.rotation == R3::identity();
    }
}
