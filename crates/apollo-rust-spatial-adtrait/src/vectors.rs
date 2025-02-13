use ad_trait::AD;
use nalgebra::{Translation3, Vector1, Vector2, Vector3, Vector4, Vector6};
use rand::Rng;
use apollo_rust_linalg_adtrait::{V, M, ApolloDMatrixTrait, ApolloDVectorTrait};

/// Alias for `Vector1<A>`, representing a 1D vector with a generic AD type.
pub type V1<A> = Vector1<A>;

/// Alias for `Vector2<A>`, representing a 2D vector with a generic AD type.
pub type V2<A> = Vector2<A>;

/// Alias for `Vector3<A>`, representing a 3D vector with a generic AD type.
pub type V3<A> = Vector3<A>;

/// Alias for `Vector4<A>`, representing a 4D vector with a generic AD type.
pub type V4<A> = Vector4<A>;

/// Alias for `Vector6<A>`, representing a 6D vector with a generic AD type.
pub type V6<A> = Vector6<A>;

/// Macro to implement AD-compatible vector operations traits for different vector types.
macro_rules! impl_apollo_ad_vector_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        /// Trait for AD-compatible vector operations on vectors of type `$type_name`.
        pub trait $trait_name<A: AD> {
            /// Creates an instance of `$type_name` from a dynamic vector `V`.
            fn from_dvector(vector: &V<A>) -> Self;

            /// Converts the instance of `$type_name` to a dynamic vector `V`.
            fn to_dvector(&self) -> V<A>;

            /// Converts the instance of `$type_name` to a dynamic matrix `M`.
            fn to_dmatrix(&self) -> M<A>;

            /// Creates a new random instance of `$type_name` with values in the given range.
            fn new_random_with_range(min: f64, max: f64) -> Self;

            fn to_other_ad_type<A2: AD>(&self) -> $type_name<A2>;

            fn to_constant_ad(&self) -> Self;
        }

        impl<A: AD> $trait_name<A> for $type_name<A> {
            fn from_dvector(vector: &V<A>) -> Self {
                Self::from_column_slice(vector.as_slice())
            }

            fn to_dvector(&self) -> V<A> {
                V::new(self.as_slice())
            }

            fn to_dmatrix(&self) -> M<A> {
                M::new(self.as_slice(), $dim, 1)
            }

            fn new_random_with_range(min: f64, max: f64) -> Self {
                let mut rng = rand::thread_rng();
                let mut v = $type_name::<A>::zeros();

                for i in 0..$dim {
                    v[i] = A::constant(rng.gen_range(min..=max));
                }

                v
            }

            fn to_other_ad_type<A2: AD>(&self) -> $type_name<A2> {
                let s = self.as_slice().iter().map(|x| x.to_other_ad_type::<A2>()).collect::<Vec<A2>>();
                return $type_name::<A2>::from_column_slice(&s);
            }

            fn to_constant_ad(&self) -> Self {
                let s = self.as_slice().iter().map(|x| x.to_constant_ad()).collect::<Vec<A>>();
                return Self::from_column_slice(&s);
            }
        }
    };
}

impl_apollo_ad_vector_trait!(ApolloVector2ADTrait, V2, 2);
impl_apollo_ad_vector_trait!(ApolloVector3ADTrait, V3, 3);
impl_apollo_ad_vector_trait!(ApolloVector4ADTrait, V4, 6);
impl_apollo_ad_vector_trait!(ApolloVector6ADTrait, V6, 6);

/// Trait for converting a 3D vector (`V3<A>`) to a 3D translation (`Translation3<A>`).
pub trait ApolloVector3Trait2<A: AD> {
    /// Converts the `V3AD<A>` vector to a `Translation3<A>` instance.
    fn to_translation(&self) -> Translation3<A>;
}

impl<A: AD> ApolloVector3Trait2<A> for V3<A> {
    fn to_translation(&self) -> Translation3<A> {
        Translation3::from(self.clone())
    }
}