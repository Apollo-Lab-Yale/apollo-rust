use ad_trait::AD;
use nalgebra::{Rotation2, Rotation3, Unit, Vector1};
use rand::Rng;
use apollo_rust_linalg_adtrait::M;
use crate::complex_numbers::UC;
use crate::matrices::{ApolloMatrix2ADTrait, ApolloMatrix3ADTrait, M2, M3};
use crate::quaternions::{ApolloUnitQuaternionADTrait, UQ};
use crate::vectors::V3;

/// Alias for `Rotation2<A>`, representing a 2D rotation with AD-compatible elements.
pub type R2<A> = Rotation2<A>;

/// Alias for `Rotation3<A>`, representing a 3D rotation with AD-compatible elements.
pub type R3<A> = Rotation3<A>;

/// Trait for operations on 2D rotations (`R2AD<A>`).
pub trait ApolloRotation2Trait<A: AD> {
    fn from_dmatrix(matrix: &M<A>) -> Self;
    fn to_dmatrix(&self) -> M<A>;
    fn new_random() -> Self;
    fn to_unit_complex(&self) -> UC<A>;
    fn to_other_ad_type<A2: AD>(&self) -> R2<A2>;
    fn to_constant_ad(&self) -> Self;
}

impl<A: AD> ApolloRotation2Trait<A> for R2<A> {
    fn from_dmatrix(matrix: &M<A>) -> Self {
        Self::from_matrix(&M2::from_dmatrix(matrix))
    }

    fn to_dmatrix(&self) -> M<A> {
        M::<A>::from_column_slice(2, 2, self.matrix().as_slice())
    }

    fn new_random() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_scaled_axis(Vector1::new(A::constant(rng.gen_range(-6.28..6.28))))
    }

    fn to_unit_complex(&self) -> UC<A> {
        UC::from_rotation_matrix(self)
    }

    fn to_other_ad_type<A2: AD>(&self) -> R2<A2> {
        R2::from_matrix(&self.matrix().to_other_ad_type::<A2>())
    }

    fn to_constant_ad(&self) -> Self {
        R2::from_matrix(&self.matrix().to_constant_ad())
    }
}

/// Trait for operations on 3D rotations (`R3AD<A>`).
pub trait ApolloRotation3Trait<A: AD> {
    fn from_dmatrix(matrix: &M<A>) -> Self;
    fn to_dmatrix(&self) -> M<A>;
    fn new_random() -> Self;
    fn from_look_at(look_at_vector: &V3<A>, axis: &V3<A>) -> Self;
    fn to_unit_quaternion(&self) -> UQ<A>;
    fn from_slice_euler_angles(slice: &[A]) -> Self;
    fn from_slice_quaternion(slice: &[A]) -> Self;
    fn frame_vectors(&self) -> [V3<A>; 3];
    fn to_other_ad_type<A2: AD>(&self) -> R3<A2>;
    fn to_constant_ad(&self) -> Self;
}

impl<A: AD> ApolloRotation3Trait<A> for R3<A> {
    fn from_dmatrix(matrix: &M<A>) -> Self {
        Self::from_matrix(&M3::from_dmatrix(matrix))
    }

    fn to_dmatrix(&self) -> M<A> {
        M::from_column_slice(3, 3, self.matrix().as_slice())
    }

    fn new_random() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_euler_angles(
            A::constant(rng.gen_range(-6.28..6.28)),
            A::constant(rng.gen_range(-6.28..6.28)),
            A::constant(rng.gen_range(-6.28..6.28)),
        )
    }

    fn from_look_at(look_at_vector: &V3<A>, axis: &V3<A>) -> Self {
        let look_at_vector = look_at_vector.normalize();
        let axis = axis.normalize();

        let rotation_axis = axis.cross(&look_at_vector);
        let u = Unit::new_normalize(rotation_axis);
        let angle = axis.dot(&look_at_vector).acos();

        Rotation3::from_axis_angle(&u, angle)
    }

    fn to_unit_quaternion(&self) -> UQ<A> {
        UQ::from_rotation_matrix(self)
    }

    fn from_slice_euler_angles(slice: &[A]) -> Self {
        Self::from_euler_angles(
            slice[0],
            slice[1],
            slice[2],
        )
    }

    fn from_slice_quaternion(slice: &[A]) -> Self {
        Self::from(UQ::from_slice_quaternion(slice))
    }

    fn frame_vectors(&self) -> [V3<A>; 3] {
        let c1 = V3::from_column_slice(self.matrix().column(0).as_slice());
        let c2 = V3::from_column_slice(self.matrix().column(1).as_slice());
        let c3 = V3::from_column_slice(self.matrix().column(2).as_slice());

        [c1, c2, c3]
    }

    fn to_other_ad_type<A2: AD>(&self) -> R3<A2> {
        R3::from_matrix(&self.matrix().to_other_ad_type::<A2>())
    }

    fn to_constant_ad(&self) -> Self {
        R3::from_matrix(&self.matrix().to_constant_ad())
    }
}