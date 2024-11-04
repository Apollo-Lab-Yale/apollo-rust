use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

/// Alias for `Quaternion<f64>`, representing a quaternion with 64-bit floating point precision.
pub type Q = Quaternion<f64>;

/// Alias for `UnitQuaternion<f64>`, representing a unit quaternion with 64-bit floating point precision.
pub type UQ = UnitQuaternion<f64>;

/// Trait for operations on quaternions (`Q`).
pub trait ApolloQuaternionTrait {
    /// Creates a quaternion from a slice of four elements in `[w x y z]` format.
    ///
    /// - `slice`: A slice representing the quaternion components.
    fn from_slice(slice: &[f64]) -> Self;

    /// Creates a quaternion from a dynamic vector `V`.
    ///
    /// - `v`: The dynamic vector `V` to convert from.
    fn from_dvector(v: &V) -> Self;

    /// Creates a random quaternion with values in the given range.
    ///
    /// - `min`: The minimum value in the range.
    /// - `max`: The maximum value in the range.
    fn new_random_with_range(min: f64, max: f64) -> Self;

    /// Converts the quaternion to a unit quaternion (`UQ`).
    fn to_unit_quaternion(&self) -> UQ;
}

impl ApolloQuaternionTrait for Q {
    /// Creates a quaternion from a slice in `[w x y z]` format.
    fn from_slice(slice: &[f64]) -> Self {
        Self::new(slice[0], slice[1], slice[2], slice[3])
    }

    /// Creates a quaternion from a dynamic vector `V`.
    fn from_dvector(v: &V) -> Self {
        Self::from_slice(v.as_slice())
    }

    /// Creates a random quaternion with values in the given range.
    fn new_random_with_range(min: f64, max: f64) -> Self {
        Self::from_dvector(&V::new_random_with_range(4, min, max))
    }

    /// Converts the quaternion to a unit quaternion (`UQ`).
    #[inline(always)]
    fn to_unit_quaternion(&self) -> UQ {
        UQ::from_quaternion(*self)
    }
}

/// Trait for operations on unit quaternions (`UQ`).
pub trait ApolloUnitQuaternionTrait {
    /// Creates a new random unit quaternion.
    fn new_random() -> Self;

    /// Creates a new random unit quaternion with values in the given range.
    ///
    /// - `min`: The minimum value in the range.
    /// - `max`: The maximum value in the range.
    fn new_random_with_range(min: f64, max: f64) -> Self;

    /// Creates a unit quaternion from a slice of four elements in `[w x y z]` format.
    ///
    /// - `slice`: A slice representing the quaternion components.
    fn from_slice_quaternion(slice: &[f64]) -> Self;

    /// Creates a unit quaternion from Euler angles.
    ///
    /// - `euler_angles`: A slice representing the Euler angles `[roll, pitch, yaw]`.
    fn from_slice_euler_angles(euler_angles: &[f64]) -> Self;

    /// Creates a unit quaternion from a scaled axis representation.
    ///
    /// - `scaled_axis`: A slice representing the scaled axis.
    fn from_slice_scaled_axis(scaled_axis: &[f64]) -> Self;
}

impl ApolloUnitQuaternionTrait for UQ {
    /// Creates a new random unit quaternion using Euler angles within the range `[-1.0, 1.0]`.
    fn new_random() -> Self {
        let v = V::new_random_with_range(3, -1.0, 1.0);
        Self::from_euler_angles(v[0], v[1], v[2])
    }

    /// Creates a new random unit quaternion with values in the specified range.
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let v = V::new_random_with_range(3, min, max);
        Self::from_euler_angles(v[0], v[1], v[2])
    }

    /// Creates a unit quaternion from a slice in `[w x y z]` format.
    fn from_slice_quaternion(slice: &[f64]) -> Self {
        assert_eq!(slice.len(), 4);
        Self::from_quaternion(Q::from_slice(slice))
    }

    /// Creates a unit quaternion from Euler angles.
    fn from_slice_euler_angles(euler_angles: &[f64]) -> Self {
        Self::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2])
    }

    /// Creates a unit quaternion from a scaled axis representation.
    fn from_slice_scaled_axis(scaled_axis: &[f64]) -> Self {
        Self::from_scaled_axis(Vector3::from_column_slice(scaled_axis))
    }
}