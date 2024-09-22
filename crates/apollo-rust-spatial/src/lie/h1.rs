use serde::{Deserialize, Serialize};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation3DLieGroupElement;
use crate::quaternions::{Q, UQ};
use crate::vectors::V3;

/// A struct representing a Lie group element `H1`, implemented using `UnitQuaternion<f64>`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupH1(pub UQ);

impl LieGroupH1 {
    /// Creates a new `LieGroupH1` instance from a `UQ` (unit quaternion).
    ///
    /// - `field0`: The unit quaternion representing the Lie group element.
    #[inline(always)]
    pub fn new(field0: UQ) -> Self {
        Self(field0)
    }

    /// Creates a `LieGroupH1` instance from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V3) -> Self {
        return exponential_coordinates.to_lie_alg_h1().exp();
    }
}

/// A struct representing a Lie algebra element `H1`, implemented using `Quaternion<f64>`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgH1(pub Q);

impl LieAlgH1 {
    /// Creates a new `LieAlgH1` instance from a `Q` (quaternion).
    ///
    /// - `field0`: The quaternion representing the Lie algebra element.
    #[inline(always)]
    pub fn new(field0: Q) -> Self {
        Self(field0)
    }
}

impl LieGroupElement for LieGroupH1 {
    type LieAlgebraElementType = LieAlgH1;

    /// Implements the group operator for the Lie group `H1`.
    ///
    /// - `other`: Another `LieGroupH1` element to multiply with.
    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    /// Returns the identity element for the Lie group `H1`.
    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(UQ::identity())
    }

    /// Returns the inverse of the current Lie group element.
    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.inverse())
    }

    /// Returns the logarithm (Lie algebra element) of the current Lie group element.
    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let w = self.0.w.min(1.0);
        let acos = w.acos();
        return if acos == 0.0 {
            LieAlgH1::new(Q::new(0., 0., 0., 0.))
        } else {
            let ss = acos / acos.sin();
            LieAlgH1::new(Q::new(0.0, ss * self.0.i, ss * self.0.j, ss * self.0.k))
        }
    }
}

/// Implements the `Rotation3DLieGroupElement` trait for `LieGroupH1`.
impl Rotation3DLieGroupElement for LieGroupH1 {}

impl LieAlgebraElement for LieAlgH1 {
    type LieGroupElementType = LieGroupH1;
    type EuclideanSpaceElementType = V3;

    /// Creates a `LieAlgH1` element from a Euclidean space element (vector).
    ///
    /// - `e`: The vector representing the Euclidean space element.
    #[inline(always)]
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_h1()
    }

    /// Scales the Lie algebra element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    #[inline(always)]
    fn scale(&self, scalar: f64) -> Self {
        Self::new(scalar * self.0)
    }

    /// Exponentiates the Lie algebra element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let v = self.0.imag();
        let vn = v.norm();
        return if vn == 0.0 {
            LieGroupH1::new(UQ::identity())
        } else {
            let cc = vn.cos();
            let ss = vn.sin() / vn;
            LieGroupH1::new(UQ::new_unchecked(Q::new(cc, ss * v[0], ss * v[1], ss * v[2])))
        }
    }

    /// Converts the Lie algebra element into its corresponding Euclidean space vector.
    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        self.0.imag()
    }
}

/// Trait for converting quaternions to `LieAlgH1`.
pub trait ApolloQuaternionH1LieTrait {
    /// Converts a quaternion to a `LieAlgH1` element.
    fn to_lie_alg_h1(&self) -> LieAlgH1;
}

impl ApolloQuaternionH1LieTrait for Q {
    fn to_lie_alg_h1(&self) -> LieAlgH1 {
        LieAlgH1::new(*self)
    }
}

/// Trait for converting unit quaternions to `LieGroupH1`.
pub trait ApolloUnitQuaternionH1LieTrait {
    /// Converts a unit quaternion to a `LieGroupH1` element.
    fn to_lie_group_h1(&self) -> LieGroupH1;
}

impl ApolloUnitQuaternionH1LieTrait for UQ {
    fn to_lie_group_h1(&self) -> LieGroupH1 {
        LieGroupH1::new(*self)
    }
}

/// Trait for converting vectors to `LieAlgH1`.
pub trait ApolloLieAlgPackH1Trait {
    /// Converts a vector to a `LieAlgH1` element.
    fn to_lie_alg_h1(&self) -> LieAlgH1;
}

impl ApolloLieAlgPackH1Trait for V3 {
    #[inline(always)]
    fn to_lie_alg_h1(&self) -> LieAlgH1 {
        LieAlgH1::new(Q::new(0.0, self[0], self[1], self[2]))
    }
}
