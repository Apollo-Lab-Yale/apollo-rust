use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use apollo_linalg::{ApolloDVectorTrait, V};


pub type Q = Quaternion<f64>;
pub type UQ = UnitQuaternion<f64>;


pub trait ApolloQuaternionTrait {
    /// [w x y z] format
    fn from_slice(slice: &[f64]) -> Self;
    fn from_dvector(v: &V) -> Self;
    fn new_random_with_range(min: f64, max: f64) -> Self;
    fn to_unit_quaternion(&self) -> UQ;
}
impl ApolloQuaternionTrait for Q {
    /// [w x y z] format
    fn from_slice(slice: &[f64]) -> Self {
        Self::new(slice[0], slice[1], slice[2], slice[3])
    }

    fn from_dvector(v: &V) -> Self {
        Self::from_slice(v.as_slice())
    }

    fn new_random_with_range(min: f64, max: f64) -> Self {
        Self::from_dvector(&V::new_random_with_range(4, min, max))
    }

    #[inline(always)]
    fn to_unit_quaternion(&self) -> UQ {
        UQ::from_quaternion(*self)
    }
}


pub trait ApolloUnitQuaternionTrait {
    fn new_random() -> Self;
    fn from_slice_quaternion(slice: &[f64]) -> Self;
    fn from_slice_euler_angles(euler_angles: &[f64]) -> Self;
    fn from_slice_scaled_axis(scaled_axis: &[f64]) -> Self;
}
impl ApolloUnitQuaternionTrait for UQ {
    fn new_random() -> Self {
        let v = V::new_random_with_range(3, -1.0, 1.0);
        Self::from_euler_angles(v[0], v[1], v[2])
    }

    fn from_slice_quaternion(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 4);
        Self::from_quaternion(Q::from_slice(slice))
    }

    fn from_slice_euler_angles(euler_angles: &[f64]) -> Self {
        Self::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2])
    }

    fn from_slice_scaled_axis(scaled_axis: &[f64]) -> Self {
        Self::from_scaled_axis(Vector3::from_column_slice(scaled_axis))
    }
}