use nalgebra::{Matrix2, Matrix3, Matrix4};
use apollo_rust_linalg::{ApolloDMatrixTrait, M};

/// Alias for `Matrix2<f64>`, representing a 2x2 matrix with 64-bit floating point precision.
pub type M2 = Matrix2<f64>;

/// Alias for `Matrix3<f64>`, representing a 3x3 matrix with 64-bit floating point precision.
pub type M3 = Matrix3<f64>;

/// Alias for `Matrix4<f64>`, representing a 4x4 matrix with 64-bit floating point precision.
pub type M4 = Matrix4<f64>;

/// Macro to implement matrix traits for different matrix types.
macro_rules! impl_apollo_matrix_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        /// Trait for matrix operations on matrices of type `$type_name`.
        pub trait $trait_name {
            /// Creates an instance of `$type_name` from a dynamic matrix `M`.
            ///
            /// - `matrix`: The dynamic matrix `M` to convert from.
            fn from_dmatrix(matrix: &M) -> Self;

            /// Converts the instance of `$type_name` to a dynamic matrix `M`.
            fn to_dmatrix(&self) -> M;

            /// Creates a new random instance of `$type_name` with values in the given range.
            ///
            /// - `min`: The minimum value in the range.
            /// - `max`: The maximum value in the range.
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
