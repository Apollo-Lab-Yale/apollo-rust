use ad_trait::AD;
use nalgebra::{Translation2, Translation3};
use crate::vectors::{ApolloVector2ADTrait, ApolloVector3ADTrait, V2, V3};

/// Alias for `Translation2<A>`, representing a 2D translation with AD-compatible elements.
pub type T2<A> = Translation2<A>;

/// Alias for `Translation3<A>`, representing a 3D translation with AD-compatible elements.
pub type T3<A> = Translation3<A>;

/// Trait for operations on 2D translations (`T2AD<A>`).
pub trait ApolloTranslation2AD<A: AD> {
    /// Creates a `T2AD<A>` instance from a slice of two elements.
    fn from_slice(slice: &[A]) -> Self;

    /// Creates a `T2AD<A>` instance from a 2D vector `V2AD<A>`.
    fn from_vector2(v2: &V2<A>) -> Self;

    /// Converts the `T2AD<A>` instance to a 2D vector `V2AD<A>`.
    fn to_vector2(&self) -> V2<A>;

    /// Creates a new random 2D translation with values in the given range.
    fn new_random_with_range(min: f64, max: f64) -> Self;

    fn to_other_ad_type<A2: AD>(&self) -> T2<A2>;

    fn to_constant_ad(&self) -> Self;
}

impl<A: AD> ApolloTranslation2AD<A> for T2<A> {
    fn from_slice(slice: &[A]) -> Self {
        Self::from_vector2(&V2::from_column_slice(slice))
    }

    fn from_vector2(v2: &V2<A>) -> Self {
        T2::new(v2.x.clone(), v2.y.clone())
    }

    fn to_vector2(&self) -> V2<A> {
        V2::new(self.vector.x.clone(), self.vector.y.clone())
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        let v2 = V2::new_random_with_range(min, max);
        Self::from_vector2(&v2)
    }

    fn to_other_ad_type<A2: AD>(&self) -> T2<A2> {
        let s = self.vector.as_slice().iter().map(|x| x.to_other_ad_type::<A2>()).collect::<Vec<A2>>();
        return T2::<A2>::from_slice(&s);
    }

    fn to_constant_ad(&self) -> Self {
        let s = self.vector.as_slice().iter().map(|x| x.to_constant_ad()).collect::<Vec<A>>();
        return Self::from_slice(&s);
    }
}

/// Trait for operations on 3D translations (`T3AD<A>`).
pub trait ApolloTranslation3AD<A: AD> {
    /// Creates a `T3AD<A>` instance from a slice of three elements.
    fn from_slice(slice: &[A]) -> Self;

    /// Creates a `T3AD<A>` instance from a 3D vector `V3AD<A>`.
    fn from_vector3(v3: &V3<A>) -> Self;

    /// Converts the `T3AD<A>` instance to a 3D vector `V3AD<A>`.
    fn to_vector3(&self) -> V3<A>;

    /// Creates a new random 3D translation with values in the given range.
    fn new_random_with_range(min: f64, max: f64) -> Self;

    fn to_other_ad_type<A2: AD>(&self) -> T3<A2>;

    fn to_constant_ad(&self) -> Self;
}

impl<A: AD> ApolloTranslation3AD<A> for T3<A> {
    fn from_slice(slice: &[A]) -> Self {
        Self::from_vector3(&V3::from_column_slice(slice))
    }

    fn from_vector3(v3: &V3<A>) -> Self {
        T3::new(v3.x.clone(), v3.y.clone(), v3.z.clone())
    }

    fn to_vector3(&self) -> V3<A> {
        V3::new(self.vector.x.clone(), self.vector.y.clone(), self.vector.z.clone())
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        let v3 = V3::new_random_with_range(min, max);
        Self::from_vector3(&v3)
    }

    fn to_other_ad_type<A2: AD>(&self) -> T3<A2> {
        let s = self.vector.as_slice().iter().map(|x| x.to_other_ad_type::<A2>()).collect::<Vec<A2>>();
        return T3::<A2>::from_slice(&s);
    }

    fn to_constant_ad(&self) -> Self {
        let s = self.vector.as_slice().iter().map(|x| x.to_constant_ad()).collect::<Vec<A>>();
        return Self::from_slice(&s);
    }
}