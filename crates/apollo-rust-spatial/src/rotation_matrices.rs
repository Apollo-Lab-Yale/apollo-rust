use nalgebra::{Rotation2, Rotation3, Unit, Vector1};
use rand::Rng;
use apollo_rust_linalg::{M};
use crate::complex_numbers::UC;
use crate::matrices::{ApolloMatrix2Trait, ApolloMatrix3Trait, M2, M3};
use crate::quaternions::{ApolloUnitQuaternionTrait, UQ};
use crate::vectors::V3;

pub type R2 = Rotation2<f64>;
pub type R3 = Rotation3<f64>;

pub trait ApolloRotation2Trait {
    fn from_dmatrix(matrix: &M) -> Self;
    fn to_dmatrix(&self) -> M;
    fn new_random() -> Self;
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

pub trait ApolloRotation3Trait {
    fn from_dmatrix(matrix: &M) -> Self;
    fn to_dmatrix(&self) -> M;
    fn new_random() -> Self;
    fn from_look_at(look_at_vector: &V3, axis: &V3) -> Self;
    fn to_unit_quaternion(&self) -> UQ;
    fn from_slice_euler_angles(slice: &[f64]) -> Self;
    fn from_slice_quaternion(slice: &[f64]) -> Self;
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
        Self::from_euler_angles(rng.gen_range(-6.28..6.28), rng.gen_range(-6.28..6.28), rng.gen_range(-6.28..6.28))
    }

    /// axis is the vector in the matrix that should be pointing in the look_at_vector direction
    /// e.g., axis = V::new(&[0.,0.,0.1])
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
}

