use nalgebra::{Rotation2, Rotation3};
use rand::Rng;
use apollo_linalg::M;
use crate::matrices::{ApolloMatrix3Trait, M3};
use crate::quaternions::UQ;


pub type R2 = Rotation2<f64>;
pub type R3 = Rotation3<f64>;

pub trait ApolloRotation3Trait {
    fn from_m(matrix: &M) -> Self;
    fn to_m(&self) -> M;
    fn new_random() -> Self;
    fn to_unit_quaternion(&self) -> UQ;
}
impl ApolloRotation3Trait for R3 {
    fn from_m(matrix: &M) -> Self {
        Self::from_matrix(&M3::from_m(matrix))
    }

    #[inline(always)]
    fn to_m(&self) -> M {
        M::from_column_slice(3, 3, self.matrix().as_slice())
    }

    fn new_random() -> Self {
        let mut rng = rand::thread_rng();
        Self::from_euler_angles(rng.gen_range(-6.28..6.28), rng.gen_range(-6.28..6.28), rng.gen_range(-6.28..6.28))
    }

    #[inline(always)]
    fn to_unit_quaternion(&self) -> UQ {
        UQ::from_rotation_matrix(self)
    }
}