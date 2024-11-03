use ad_trait::AD;
use nalgebra::{Translation2, Translation3};
use crate::vectors::{ApolloVector2ADTrait, ApolloVector2Trait, ApolloVector3ADTrait, ApolloVector3Trait, V2, V2AD, V3, V3AD};

/// Alias for `Translation2<f64>`, representing a 2D translation with 64-bit floating point precision.
pub type T2 = Translation2<f64>;

/// Alias for `Translation3<f64>`, representing a 3D translation with 64-bit floating point precision.
pub type T3 = Translation3<f64>;

/// Trait for operations on 2D translations (`T2`).
pub trait ApolloTranslation2 {
    /// Creates a `T2` instance from a slice of two elements.
    ///
    /// - `slice`: A slice representing the 2D translation components `[x, y]`.
    fn from_slice(slice: &[f64]) -> Self;

    /// Creates a `T2` instance from a 2D vector `V2`.
    ///
    /// - `v2`: The 2D vector to convert from.
    fn from_vector2(v2: &V2) -> Self;

    /// Converts the `T2` instance to a 2D vector `V2`.
    fn to_vector2(&self) -> V2;

    /// Creates a new random 2D translation with values in the given range.
    ///
    /// - `min`: The minimum value in the range.
    /// - `max`: The maximum value in the range.
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

/// Trait for operations on 3D translations (`T3`).
pub trait ApolloTranslation3 {
    /// Creates a `T3` instance from a slice of three elements.
    ///
    /// - `slice`: A slice representing the 3D translation components `[x, y, z]`.
    fn from_slice(slice: &[f64]) -> Self;

    /// Creates a `T3` instance from a 3D vector `V3`.
    ///
    /// - `v3`: The 3D vector to convert from.
    fn from_vector3(v3: &V3) -> Self;

    /// Converts the `T3` instance to a 3D vector `V3`.
    fn to_vector3(&self) -> V3;

    /// Creates a new random 3D translation with values in the given range.
    ///
    /// - `min`: The minimum value in the range.
    /// - `max`: The maximum value in the range.
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

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Alias for `Translation2<A>`, representing a 2D translation with AD-compatible elements.
pub type T2AD<A> = Translation2<A>;

/// Alias for `Translation3<A>`, representing a 3D translation with AD-compatible elements.
pub type T3AD<A> = Translation3<A>;

/// Trait for operations on 2D translations (`T2AD<A>`).
pub trait ApolloTranslation2AD<A: AD> {
    /// Creates a `T2AD<A>` instance from a slice of two elements.
    fn from_slice_ad(slice: &[A]) -> Self;

    /// Creates a `T2AD<A>` instance from a 2D vector `V2AD<A>`.
    fn from_vector2_ad(v2: &V2AD<A>) -> Self;

    /// Converts the `T2AD<A>` instance to a 2D vector `V2AD<A>`.
    fn to_vector2_ad(&self) -> V2AD<A>;

    /// Creates a new random 2D translation with values in the given range.
    fn new_ad_random_with_range(min: f64, max: f64) -> Self;
}

impl<A: AD> ApolloTranslation2AD<A> for T2AD<A> {
    fn from_slice_ad(slice: &[A]) -> Self {
        Self::from_vector2_ad(&V2AD::from_column_slice(slice))
    }

    fn from_vector2_ad(v2: &V2AD<A>) -> Self {
        T2AD::new(v2.x.clone(), v2.y.clone())
    }

    fn to_vector2_ad(&self) -> V2AD<A> {
        V2AD::new(self.vector.x.clone(), self.vector.y.clone())
    }

    fn new_ad_random_with_range(min: f64, max: f64) -> Self {
        let v2 = V2AD::new_ad_random_with_range(min, max);
        Self::from_vector2_ad(&v2)
    }
}

/// Trait for operations on 3D translations (`T3AD<A>`).
pub trait ApolloTranslation3AD<A: AD> {
    /// Creates a `T3AD<A>` instance from a slice of three elements.
    fn from_slice_ad(slice: &[A]) -> Self;

    /// Creates a `T3AD<A>` instance from a 3D vector `V3AD<A>`.
    fn from_vector3_ad(v3: &V3AD<A>) -> Self;

    /// Converts the `T3AD<A>` instance to a 3D vector `V3AD<A>`.
    fn to_vector3_ad(&self) -> V3AD<A>;

    /// Creates a new random 3D translation with values in the given range.
    fn new_ad_random_with_range(min: f64, max: f64) -> Self;
}

impl<A: AD> ApolloTranslation3AD<A> for T3AD<A> {
    fn from_slice_ad(slice: &[A]) -> Self {
        Self::from_vector3_ad(&V3AD::from_column_slice(slice))
    }

    fn from_vector3_ad(v3: &V3AD<A>) -> Self {
        T3AD::new(v3.x.clone(), v3.y.clone(), v3.z.clone())
    }

    fn to_vector3_ad(&self) -> V3AD<A> {
        V3AD::new(self.vector.x.clone(), self.vector.y.clone(), self.vector.z.clone())
    }

    fn new_ad_random_with_range(min: f64, max: f64) -> Self {
        let v3 = V3AD::new_ad_random_with_range(min, max);
        Self::from_vector3_ad(&v3)
    }
}