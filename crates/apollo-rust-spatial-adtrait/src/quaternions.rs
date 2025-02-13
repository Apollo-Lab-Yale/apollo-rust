use ad_trait::AD;
use nalgebra::{Quaternion, UnitQuaternion};
use apollo_rust_linalg_adtrait::{ApolloDVectorTrait, V};
use crate::vectors::{ApolloVector4ADTrait, V3};

/// Alias for `Quaternion<A>`, representing a quaternion with AD-compatible elements.
pub type Q<A> = Quaternion<A>;

/// Alias for `UnitQuaternion<A>`, representing a unit quaternion with AD-compatible elements.
pub type UQ<A> = UnitQuaternion<A>;

/// Trait for operations on quaternions (`QAD<A>`).
pub trait ApolloQuaternionTrait<A: AD> {
    fn from_slice(slice: &[A]) -> Self;
    fn from_dvector(v: &V<A>) -> Self;
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn to_unit_quaternion(&self) -> UQ<A>;
    fn to_other_ad_type<A2: AD>(&self) -> Q<A2>;
    fn to_constant_ad(&self) -> Q<A>;
}
impl<A: AD> ApolloQuaternionTrait<A> for Q<A> {
    fn from_slice(slice: &[A]) -> Self {
        Self::new(
            slice[0],
            slice[1],
            slice[2],
            slice[3],
        )
    }

    fn from_dvector(v: &V<A>) -> Self {
        Self::from_slice(v.as_slice())
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        Self::from_dvector(&V::new_random_with_range(4, min, max))
    }

    fn to_unit_quaternion(&self) -> UQ<A> {
        UQ::from_quaternion(*self)
    }

    fn to_other_ad_type<A2: AD>(&self) -> Q<A2> {
        Q::from_slice(self.coords.to_other_ad_type::<A2>().as_slice())
    }

    fn to_constant_ad(&self) -> Q<A> {
        Q::from_slice(self.coords.to_constant_ad().as_slice())
    }
}

/// Trait for operations on unit quaternions (`UQAD<A>`).
pub trait ApolloUnitQuaternionADTrait<A: AD> {
    fn new_random() -> Self;
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn from_slice_quaternion(slice: &[A]) -> Self;
    fn from_slice_euler_angles(euler_angles: &[A]) -> Self;
    fn from_slice_scaled_axis(scaled_axis: &[A]) -> Self;
    fn to_other_ad_type<A2: AD>(&self) -> UQ<A2>;
    fn to_constant_ad(&self) -> Self;
}
impl<A: AD> ApolloUnitQuaternionADTrait<A> for UQ<A> {
    fn new_random() -> Self {
        let v = V::new_random_with_range(3, -1.0, 1.0);
        Self::from_euler_angles(A::constant(v[0]), A::constant(v[1]), A::constant(v[2]))
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        let v = V::new_random_with_range(3, min, max);
        Self::from_euler_angles(A::constant(v[0]), A::constant(v[1]), A::constant(v[2]))
    }

    fn from_slice_quaternion(slice: &[A]) -> Self {
        Self::from_quaternion(Q::from_slice(slice))
    }

    fn from_slice_euler_angles(euler_angles: &[A]) -> Self {
        Self::from_euler_angles(
            euler_angles[0],
            euler_angles[1],
            euler_angles[2],
        )
    }

    fn from_slice_scaled_axis(scaled_axis: &[A]) -> Self {
        Self::from_scaled_axis(V3::from_column_slice(scaled_axis))
    }

    fn to_other_ad_type<A2: AD>(&self) -> UQ<A2> {
        UQ::from_quaternion(Q::from_slice(self.coords.to_other_ad_type::<A2>().as_slice()))
    }

    fn to_constant_ad(&self) -> Self {
        UQ::from_quaternion(Q::from_slice(self.coords.to_constant_ad().as_slice()))
    }
}