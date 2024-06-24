use nalgebra::{Translation3, Vector1, Vector2, Vector3, Vector6};
use rand::Rng;
use apollo_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};

pub type V1 = Vector1<f64>;
pub type V2 = Vector2<f64>;
pub type V3 = Vector3<f64>;
pub type V6 = Vector6<f64>;

macro_rules! impl_apollo_vector_trait {
    ($trait_name:ident, $type_name:ident, $dim:expr) => {
        pub trait $trait_name {
            fn from_dvector(vector: &V) -> Self;
            fn to_dvector(&self) -> V;
            fn to_dmatrix(&self) -> M;
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

pub trait ApolloVector3Trait2 {
    fn to_translation(&self) -> Translation3<f64>;
}
