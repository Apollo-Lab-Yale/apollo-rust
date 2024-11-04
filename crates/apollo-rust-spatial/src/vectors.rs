
use nalgebra::{Translation3, Vector1, Vector2, Vector3, Vector6};
use rand::Rng;
use apollo_rust_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};
use crate::translations::{ApolloTranslation3, T3};


/// Alias for `Vector1<f64>`, representing a 1D vector with 64-bit floating point precision.
pub type V1 = Vector1<f64>;

/// Alias for `Vector2<f64>`, representing a 2D vector with 64-bit floating point precision.
pub type V2 = Vector2<f64>;

/// Alias for `Vector3<f64>`, representing a 3D vector with 64-bit floating point precision.
pub type V3 = Vector3<f64>;

/// Alias for `Vector6<f64>`, representing a 6D vector with 64-bit floating point precision.
pub type V6 = Vector6<f64>;

/// Macro to implement vector operations traits for different vector types.
macro_rules! impl_apollo_vector_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        /// Trait for vector operations on vectors of type `$type_name`.
        pub trait $trait_name {
            /// Creates an instance of `$type_name` from a dynamic vector `V`.
            ///
            /// - `vector`: The dynamic vector `V` to convert from.
            fn from_dvector(vector: &V) -> Self;

            /// Converts the instance of `$type_name` to a dynamic vector `V`.
            fn to_dvector(&self) -> V;

            /// Converts the instance of `$type_name` to a dynamic matrix `M`.
            fn to_dmatrix(&self) -> M;

            /// Creates a new random instance of `$type_name` with values in the given range.
            ///
            /// - `min`: The minimum value in the range.
            /// - `max`: The maximum value in the range.
            fn new_random_with_range(min: f64, max: f64) -> Self;
        }

        impl $trait_name for $type_name {
            fn from_dvector(vector: &V) -> Self {
                Self::from_column_slice(vector.as_slice())
            }

            #[inline(always)]
            fn to_dvector(&self) -> V {
                V::new(self.as_slice())
            }

            #[inline(always)]
            fn to_dmatrix(&self) -> M {
                M::new(self.as_slice(), $dim, 1)
            }

            fn new_random_with_range(min: f64, max: f64) -> Self {
                let mut rng = rand::thread_rng();
                let mut v = $type_name::zeros();

                for i in 0..$dim {
                    v[i] = rng.gen_range(min..=max);
                }

                v
            }
        }
    };
}

impl_apollo_vector_trait!(ApolloVector2Trait, V2, 2);

impl_apollo_vector_trait!(ApolloVector3Trait, V3, 3);

impl_apollo_vector_trait!(ApolloVector6Trait, V6, 6);

/// Trait for converting a 3D vector (`V3`) to a 3D translation (`T3`).
pub trait ApolloVector3Trait2 {
    /// Converts the `V3` vector to a `Translation3<f64>` instance.
    fn to_translation(&self) -> Translation3<f64>;
}

impl ApolloVector3Trait2 for V3 {
    fn to_translation(&self) -> Translation3<f64> {
        T3::from_vector3(self)
    }
}