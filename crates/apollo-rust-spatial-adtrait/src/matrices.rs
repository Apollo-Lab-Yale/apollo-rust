use nalgebra::{Matrix2, Matrix3, Matrix4};
use ad_trait::AD;
use apollo_rust_linalg_adtrait::{M, ApolloDMatrixTrait};

/// Alias for `Matrix2<A>`, representing a 2x2 matrix with AD-compatible elements.
pub type M2<A> = Matrix2<A>;

/// Alias for `Matrix3<A>`, representing a 3x3 matrix with AD-compatible elements.
pub type M3<A> = Matrix3<A>;

/// Alias for `Matrix4<A>`, representing a 4x4 matrix with AD-compatible elements.
pub type M4<A> = Matrix4<A>;

/// Macro to implement AD-compatible matrix traits for different matrix types.
macro_rules! impl_apollo_ad_matrix_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        /// Trait for AD-compatible matrix operations on matrices of type `$type_name`.
        pub trait $trait_name<A: AD> {
            /// Creates an instance of `$type_name` from a dynamic matrix `M`.
            fn from_dmatrix(matrix: &M<A>) -> Self;

            /// Converts the instance of `$type_name` to a dynamic matrix `M`.
            fn to_dmatrix(&self) -> M<A>;

            /// Creates a new random instance of `$type_name` with values in the given range.
            fn new_random_with_range(min: f64, max: f64) -> Self;
        }

        impl<A: AD> $trait_name<A> for $type_name<A> {
            fn from_dmatrix(matrix: &M<A>) -> Self {
                Self::from_column_slice(matrix.as_slice())
            }

            #[inline(always)]
            fn to_dmatrix(&self) -> M<A> {
                M::<A>::from_column_slice($dim, $dim, self.as_slice())
            }

            fn new_random_with_range(min: f64, max: f64) -> Self {
                let random_matrix = M::new_random_with_range($dim, $dim, min, max);
                Self::from_dmatrix(&random_matrix)
            }
        }
    };
}

impl_apollo_ad_matrix_trait!(ApolloMatrix2ADTrait, M2, 2);
impl_apollo_ad_matrix_trait!(ApolloMatrix3ADTrait, M3, 3);
impl_apollo_ad_matrix_trait!(ApolloMatrix4ADTrait, M4, 4);
