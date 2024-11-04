use ad_trait::AD;
use nalgebra::{Rotation2, Rotation3, Unit, Vector1};
use rand::Rng;
use apollo_rust_linalg::linalg_ad::MAD;
use crate::complex_numbers::{UCAD};
use crate::matrices::{ApolloMatrix2ADTrait, ApolloMatrix3ADTrait, M2AD, M3AD};
use crate::quaternions::{ApolloUnitQuaternionADTrait, UQAD};
use crate::vectors::{V3AD};

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