use nalgebra::{Matrix2, Matrix3};
use apollo_linalg::{ApolloDMatrixTrait, M};

pub type M2 = Matrix2<f64>;
pub type M3 = Matrix3<f64>;


pub trait ApolloMatrix3Trait {
    fn from_m(matrix: &M) -> Self;
    fn to_m(&self) -> M;
    fn new_random_with_range(min: f64, max: f64) -> Self;
}
impl ApolloMatrix3Trait for M3 {
    fn from_m(matrix: &M) -> Self {
        Self::from_column_slice(matrix.as_slice())
    }
    #[inline(always)]
    fn to_m(&self) -> M {
        M::from_column_slice(3, 3, self.as_slice())
    }
    fn new_random_with_range(min: f64, max: f64) -> Self {
        Self::from_m(&M::new_random_with_range(3, 3, min, max))
    }
}