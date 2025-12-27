use crate::complex_numbers::UC;
use crate::matrices::{ApolloMatrix2Trait, ApolloMatrix3Trait, M2, M3};
use crate::quaternions::{ApolloUnitQuaternionTrait, UQ};
use crate::vectors::V3;
use apollo_rust_linalg::M;
use nalgebra::{Rotation2, Rotation3, Unit, Vector1};
use rand::Rng;

/// Alias for `Rotation2<f64>`, representing a 2D rotation with 64-bit floating point precision.
pub type R2 = Rotation2<f64>;

/// Alias for `Rotation3<f64>`, representing a 3D rotation with 64-bit floating point precision.
pub type R3 = Rotation3<f64>;

/// Trait for operations on 2D rotations (`R2`).
pub trait ApolloRotation2Trait {
    /// Creates an `R2` instance from a dynamic matrix `M`.
    ///
    /// - `matrix`: The dynamic matrix `M` to convert from.
    fn from_dmatrix(matrix: &M) -> Self;

    /// Converts the `R2` instance to a dynamic matrix `M`.
    fn to_dmatrix(&self) -> M;

    /// Creates a new random `R2` rotation.
    fn new_random() -> Self;

    /// Converts the `R2` instance to a unit complex number representation (`UC`).
    fn to_unit_complex(&self) -> UC;
}

impl ApolloRotation2Trait for R2 {
    fn from_dmatrix(matrix: &M) -> Self {
        Self::from_matrix(&M2::from_dmatrix(matrix))
    }

    #[inline(always)]
    fn to_dmatrix(&self) -> M {
        M::from_column_slice(3, 3, self.matrix().as_slice())
    }

    fn new_random() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_scaled_axis(Vector1::new(rng.gen_range(-6.28..6.28)))
    }

    #[inline(always)]
    fn to_unit_complex(&self) -> UC {
        UC::from_rotation_matrix(self)
    }
}

/// Trait for operations on 3D rotations (`R3`).
pub trait ApolloRotation3Trait {
    /// Creates an `R3` instance from a dynamic matrix `M`.
    ///
    /// - `matrix`: The dynamic matrix `M` to convert from.
    fn from_dmatrix(matrix: &M) -> Self;

    /// Converts the `R3` instance to a dynamic matrix `M`.
    fn to_dmatrix(&self) -> M;

    /// Creates a new random `R3` rotation.
    fn new_random() -> Self;

    /// Creates an `R3` instance that points the `axis` in the direction of the `look_at_vector`.
    ///
    /// - `look_at_vector`: The vector to look at.
    /// - `axis`: The axis of the rotation to align with the `look_at_vector`.
    fn from_look_at(look_at_vector: &V3, axis: &V3) -> Self;

    /// Converts the `R3` instance to a unit quaternion (`UQ`).
    fn to_unit_quaternion(&self) -> UQ;

    /// Creates an `R3` instance from a slice of three Euler angles.
    ///
    /// - `slice`: A slice representing the Euler angles `[roll, pitch, yaw]`.
    fn from_slice_euler_angles(slice: &[f64]) -> Self;

    /// Creates an `R3` instance from a quaternion slice.
    ///
    /// - `slice`: A slice representing the quaternion `[w, x, y, z]`.
    fn from_slice_quaternion(slice: &[f64]) -> Self;

    fn frame_vectors(&self) -> [V3; 3];
}

impl ApolloRotation3Trait for R3 {
    fn from_dmatrix(matrix: &M) -> Self {
        Self::from_matrix(&M3::from_dmatrix(matrix))
    }

    #[inline(always)]
    fn to_dmatrix(&self) -> M {
        M::from_column_slice(3, 3, self.matrix().as_slice())
    }

    fn new_random() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_euler_angles(
            rng.gen_range(-6.28..6.28),
            rng.gen_range(-6.28..6.28),
            rng.gen_range(-6.28..6.28),
        )
    }

    /// Creates an `R3` instance that points the `axis` in the direction of the `look_at_vector`.
    ///
    /// - `look_at_vector`: The vector to look at.
    /// - `axis`: The vector that will be rotated to align with `look_at_vector`.
    ///
    #[inline]
    fn from_look_at(look_at_vector: &V3, axis: &V3) -> Self {
        let look_at_vector = look_at_vector.normalize();
        let axis = axis.normalize();

        let rotation_axis = axis.cross(&look_at_vector);
        let rotation_axis = V3::new(rotation_axis[0], rotation_axis[1], rotation_axis[2]);
        let u = Unit::new_normalize(rotation_axis);
        let angle = axis.dot(&look_at_vector).acos();

        return Rotation3::from_axis_angle(&u, angle);
    }

    #[inline(always)]
    fn to_unit_quaternion(&self) -> UQ {
        UQ::from_rotation_matrix(self)
    }

    fn from_slice_euler_angles(slice: &[f64]) -> Self {
        Self::from_euler_angles(slice[0], slice[1], slice[2])
    }

    fn from_slice_quaternion(slice: &[f64]) -> Self {
        Self::from(UQ::from_slice_quaternion(slice))
    }

    fn frame_vectors(&self) -> [V3; 3] {
        let c1 = V3::from_column_slice(self.matrix().column(0).as_slice());
        let c2 = V3::from_column_slice(self.matrix().column(1).as_slice());
        let c3 = V3::from_column_slice(self.matrix().column(2).as_slice());

        [c1, c2, c3]
    }
}
