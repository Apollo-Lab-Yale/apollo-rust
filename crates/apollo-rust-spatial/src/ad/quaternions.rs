use ad_trait::AD;
use nalgebra::{Quaternion, UnitQuaternion};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_linalg::linalg_ad::{ApolloDVectorADTrait, VAD};
use crate::vectors::V3AD;

/// Alias for `Quaternion<A>`, representing a quaternion with AD-compatible elements.
pub type QAD<A> = Quaternion<A>;

/// Alias for `UnitQuaternion<A>`, representing a unit quaternion with AD-compatible elements.
pub type UQAD<A> = UnitQuaternion<A>;

/// Trait for operations on quaternions (`QAD<A>`).
pub trait ApolloQuaternionADTrait<A: AD> {
    fn from_slice_ad(slice: &[A]) -> Self;
    fn from_dvector_ad(v: &VAD<A>) -> Self;
    fn new_ad_random_with_range(min: f64, max: f64) -> Self;
    fn to_unit_quaternion_ad(&self) -> UQAD<A>;
}
impl<A: AD> ApolloQuaternionADTrait<A> for QAD<A> {
    fn from_slice_ad(slice: &[A]) -> Self {
        Self::new(
            slice[0],
            slice[1],
            slice[2],
            slice[3],
        )
    }

    fn from_dvector_ad(v: &VAD<A>) -> Self {
        Self::from_slice_ad(v.as_slice())
    }

    fn new_ad_random_with_range(min: f64, max: f64) -> Self {
        Self::from_dvector_ad(&VAD::new_ad_random_with_range(4, min, max))
    }

    fn to_unit_quaternion_ad(&self) -> UQAD<A> {
        UQAD::from_quaternion(*self)
    }
}

/// Trait for operations on unit quaternions (`UQAD<A>`).
pub trait ApolloUnitQuaternionADTrait<A: AD> {
    fn new_ad_random() -> Self;
    fn new_ad_random_with_range(min: f64, max: f64) -> Self;
    fn from_slice_quaternion_ad(slice: &[A]) -> Self;
    fn from_slice_euler_angles_ad(euler_angles: &[A]) -> Self;
    fn from_slice_scaled_axis_ad(scaled_axis: &[A]) -> Self;
}
impl<A: AD> ApolloUnitQuaternionADTrait<A> for UQAD<A> {
    fn new_ad_random() -> Self {
        let v = V::new_random_with_range(3, -1.0, 1.0);
        Self::from_euler_angles(A::constant(v[0]), A::constant(v[1]), A::constant(v[2]))
    }

    fn new_ad_random_with_range(min: f64, max: f64) -> Self {
        let v = V::new_random_with_range(3, min, max);
        Self::from_euler_angles(A::constant(v[0]), A::constant(v[1]), A::constant(v[2]))
    }

    fn from_slice_quaternion_ad(slice: &[A]) -> Self {
        Self::from_quaternion(QAD::from_slice_ad(slice))
    }

    fn from_slice_euler_angles_ad(euler_angles: &[A]) -> Self {
        Self::from_euler_angles(
            euler_angles[0],
            euler_angles[1],
            euler_angles[2],
        )
    }

    fn from_slice_scaled_axis_ad(scaled_axis: &[A]) -> Self {
        Self::from_scaled_axis(V3AD::from_column_slice(scaled_axis))
    }
}