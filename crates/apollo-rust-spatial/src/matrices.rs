use nalgebra::{Matrix2, Matrix3, Matrix4};
use apollo_rust_linalg::{ApolloDMatrixTrait, M};

pub type M2 = Matrix2<f64>;
pub type M3 = Matrix3<f64>;
pub type M4 = Matrix4<f64>;

macro_rules! impl_apollo_matrix_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        pub trait $trait_name {
            fn from_dmatrix(matrix: &M) -> Self;
            fn to_dmatrix(&self) -> M;
            fn new_random_with_range(min: f64, max: f64) -> Self;
        }
        impl $trait_name for $type_name {
            fn from_dmatrix(matrix: &M) -> Self {
                Self::from_column_slice(matrix.as_slice())
            }
            #[inline(always)]
            fn to_dmatrix(&self) -> M {
                M::from_column_slice($dim, $dim, self.as_slice())
            }
            fn new_random_with_range(min: f64, max: f64) -> Self {
                Self::from_dmatrix(&M::new_random_with_range($dim, $dim, min, max))
            }
        }
    };
}

impl_apollo_matrix_trait!(ApolloMatrix2Trait, M2, 2);
impl_apollo_matrix_trait!(ApolloMatrix3Trait, M3, 3);
impl_apollo_matrix_trait!(ApolloMatrix4Trait, M4, 4);