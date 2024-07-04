use nalgebra::{Translation2, Translation3};
use crate::vectors::{ApolloVector2Trait, ApolloVector3Trait, V2, V3};

pub type T2 = Translation2<f64>;
pub type T3 = Translation3<f64>;

pub trait ApolloTranslation2 {
    fn from_slice(slice: &[f64]) -> Self;
    fn from_vector2(v2: &V2) -> Self;
    fn to_vector2(&self) -> V2;
    fn new_random_with_range(min: f64, max: f64) -> Self;
}
impl ApolloTranslation2 for T2 {
    fn from_slice(slice: &[f64]) -> Self {
        Self::from_vector2(&V2::from_column_slice(slice))
    }

    fn from_vector2(v2: &V2) -> Self {
        T2::new(v2.x, v2.y)
    }

    #[inline(always)]
    fn to_vector2(&self) -> V2 {
        V2::from_column_slice(self.vector.as_slice())
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        Self::from_vector2(&V2::new_random_with_range(min, max))
    }
}

pub trait ApolloTranslation3 {
    fn from_slice(slice: &[f64]) -> Self;
    fn from_vector3(v3: &V3) -> Self;
    fn to_vector3(&self) -> V3;
    fn new_random_with_range(min: f64, max: f64) -> Self;
}
impl ApolloTranslation3 for T3 {
    fn from_slice(slice: &[f64]) -> Self {
        Self::from_vector3(&V3::from_column_slice(slice))
    }

    fn from_vector3(v3: &V3) -> Self {
        T3::new(v3.x, v3.y, v3.z)
    }

    #[inline(always)]
    fn to_vector3(&self) -> V3 {
        V3::from_column_slice(self.vector.as_slice())
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        Self::from_vector3(&V3::new_random_with_range(min, max))
    }
}
