use ad_trait::AD;
use nalgebra::{Rotation2, Rotation3, Unit, Vector1};
use rand::Rng;
use apollo_rust_linalg::{M};
use apollo_rust_linalg::linalg_ad::MAD;
use crate::complex_numbers::{UC, UCAD};
use crate::matrices::{ApolloMatrix2ADTrait, ApolloMatrix2Trait, ApolloMatrix3ADTrait, ApolloMatrix3Trait, M2, M2AD, M3, M3AD};
use crate::quaternions::{ApolloUnitQuaternionADTrait, ApolloUnitQuaternionTrait, UQ, UQAD};
use crate::vectors::{V3, V3AD};

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

    fn frame_vectors(&self) -> [ V3; 3 ];
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
            rng.gen_range(-6.28..6.28)
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

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Alias for `Rotation2<A>`, representing a 2D rotation with AD-compatible elements.
pub type R2AD<A> = Rotation2<A>;

/// Alias for `Rotation3<A>`, representing a 3D rotation with AD-compatible elements.
pub type R3AD<A> = Rotation3<A>;

/// Trait for operations on 2D rotations (`R2AD<A>`).
pub trait ApolloRotation2ADTrait<A: AD> {
    fn from_dmatrix_ad(matrix: &MAD<A>) -> Self;
    fn to_dmatrix_ad(&self) -> MAD<A>;
    fn new_random_ad() -> Self;
    fn to_unit_complex_ad(&self) -> UCAD<A>;
}

impl<A: AD> ApolloRotation2ADTrait<A> for R2AD<A> {
    fn from_dmatrix_ad(matrix: &MAD<A>) -> Self {
        Self::from_matrix(&M2AD::from_dmatrix_ad(matrix))
    }

    fn to_dmatrix_ad(&self) -> MAD<A> {
        MAD::<A>::from_column_slice(2, 2, self.matrix().as_slice())
    }

    fn new_random_ad() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_scaled_axis(Vector1::new(A::constant(rng.gen_range(-6.28..6.28))))
    }

    fn to_unit_complex_ad(&self) -> UCAD<A> {
        UCAD::from_rotation_matrix(self)
    }
}

/// Trait for operations on 3D rotations (`R3AD<A>`).
pub trait ApolloRotation3ADTrait<A: AD> {
    fn from_dmatrix_ad(matrix: &MAD<A>) -> Self;
    fn to_dmatrix_ad(&self) -> MAD<A>;
    fn new_ad_random() -> Self;
    fn from_look_at_ad(look_at_vector: &V3AD<A>, axis: &V3AD<A>) -> Self;
    fn to_unit_quaternion_ad(&self) -> UQAD<A>;
    fn from_slice_euler_angles_ad(slice: &[A]) -> Self;
    fn from_slice_quaternion_ad(slice: &[A]) -> Self;
    fn frame_vectors_ad(&self) -> [V3AD<A>; 3];
}

impl<A: AD> ApolloRotation3ADTrait<A> for R3AD<A> {
    fn from_dmatrix_ad(matrix: &MAD<A>) -> Self {
        Self::from_matrix(&M3AD::from_dmatrix_ad(matrix))
    }

    fn to_dmatrix_ad(&self) -> MAD<A> {
        MAD::from_column_slice(3, 3, self.matrix().as_slice())
    }

    fn new_ad_random() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_euler_angles(
            A::constant(rng.gen_range(-6.28..6.28)),
            A::constant(rng.gen_range(-6.28..6.28)),
            A::constant(rng.gen_range(-6.28..6.28)),
        )
    }

    fn from_look_at_ad(look_at_vector: &V3AD<A>, axis: &V3AD<A>) -> Self {
        let look_at_vector = look_at_vector.normalize();
        let axis = axis.normalize();

        let rotation_axis = axis.cross(&look_at_vector);
        let u = Unit::new_normalize(rotation_axis);
        let angle = axis.dot(&look_at_vector).acos();

        Rotation3::from_axis_angle(&u, angle)
    }

    fn to_unit_quaternion_ad(&self) -> UQAD<A> {
        UQAD::from_rotation_matrix(self)
    }

    fn from_slice_euler_angles_ad(slice: &[A]) -> Self {
        Self::from_euler_angles(
            slice[0],
            slice[1],
            slice[2],
        )
    }

    fn from_slice_quaternion_ad(slice: &[A]) -> Self {
        Self::from(UQAD::from_slice_quaternion_ad(slice))
    }

    fn frame_vectors_ad(&self) -> [V3AD<A>; 3] {
        let c1 = V3AD::from_column_slice(self.matrix().column(0).as_slice());
        let c2 = V3AD::from_column_slice(self.matrix().column(1).as_slice());
        let c3 = V3AD::from_column_slice(self.matrix().column(2).as_slice());

        [c1, c2, c3]
    }
}