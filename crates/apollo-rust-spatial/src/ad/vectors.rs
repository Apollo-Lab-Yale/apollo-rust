use ad_trait::AD;
use nalgebra::{Translation3, Vector1, Vector2, Vector3, Vector6};
use rand::Rng;
use apollo_rust_linalg::linalg_ad::VAD;
use apollo_rust_linalg::linalg_ad::MAD;
use apollo_rust_linalg::linalg_ad::ApolloDVectorADTrait;
use apollo_rust_linalg::linalg_ad::ApolloDMatrixADTrait;

/// Alias for `Vector1<A>`, representing a 1D vector with a generic AD type.
pub type V1AD<A> = Vector1<A>;

/// Alias for `Vector2<A>`, representing a 2D vector with a generic AD type.
pub type V2AD<A> = Vector2<A>;

/// Alias for `Vector3<A>`, representing a 3D vector with a generic AD type.
pub type V3AD<A> = Vector3<A>;

/// Alias for `Vector6<A>`, representing a 6D vector with a generic AD type.
pub type V6AD<A> = Vector6<A>;

/// Macro to implement AD-compatible vector operations traits for different vector types.
macro_rules! impl_apollo_ad_vector_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        /// Trait for AD-compatible vector operations on vectors of type `$type_name`.
        pub trait $trait_name<A: AD> {
            /// Creates an instance of `$type_name` from a dynamic vector `V`.
            fn from_dvector_ad(vector: &VAD<A>) -> Self;

            /// Converts the instance of `$type_name` to a dynamic vector `V`.
            fn to_dvector_ad(&self) -> VAD<A>;

            /// Converts the instance of `$type_name` to a dynamic matrix `M`.
            fn to_dmatrix_ad(&self) -> MAD<A>;

            /// Creates a new random instance of `$type_name` with values in the given range.
            fn new_ad_random_with_range(min: f64, max: f64) -> Self;
        }

        impl<A: AD> $trait_name<A> for $type_name<A> {
            fn from_dvector_ad(vector: &VAD<A>) -> Self {
                Self::from_column_slice(vector.as_slice())
            }

            fn to_dvector_ad(&self) -> VAD<A> {
                VAD::new_ad(self.as_slice())
            }

            fn to_dmatrix_ad(&self) -> MAD<A> {
                MAD::new_ad(self.as_slice(), $dim, 1)
            }

            fn new_ad_random_with_range(min: f64, max: f64) -> Self {
                let mut rng = rand::thread_rng();
                let mut v = $type_name::<A>::zeros();

                for i in 0..$dim {
                    v[i] = A::constant(rng.gen_range(min..=max));
                }

                v
            }
        }
    };
}

impl_apollo_ad_vector_trait!(ApolloVector2ADTrait, V2AD, 2);
impl_apollo_ad_vector_trait!(ApolloVector3ADTrait, V3AD, 3);
impl_apollo_ad_vector_trait!(ApolloVector6ADTrait, V6AD, 6);

/// Trait for converting a 3D vector (`V3AD<A>`) to a 3D translation (`Translation3<A>`).
pub trait ApolloVector3ADTrait2<A: AD> {
    /// Converts the `V3AD<A>` vector to a `Translation3<A>` instance.
    fn to_translation_ad(&self) -> Translation3<A>;
}

impl<A: AD> ApolloVector3ADTrait2<A> for V3AD<A> {
    fn to_translation_ad(&self) -> Translation3<A> {
        Translation3::from(self.clone())
    }
}