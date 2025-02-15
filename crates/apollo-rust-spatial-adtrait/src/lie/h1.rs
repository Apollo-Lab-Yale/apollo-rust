use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation3DLieGroupElement;
use crate::quaternions::{ApolloQuaternionTrait, ApolloUnitQuaternionADTrait, Q, UQ};
use crate::vectors::V3;

/// A struct representing a Lie group element `H1`, implemented using `UnitQuaternion<f64>`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupH1<A: AD>(#[serde(deserialize_with = "UQ::<A>::deserialize")] pub UQ<A>);
impl<A: AD> LieGroupH1<A> {
    /// Creates a new `LieGroupH1` instance from a `UQ` (unit quaternion).
    ///
    /// - `field0`: The unit quaternion representing the Lie group element.
    #[inline(always)]
    pub fn new(field0: UQ<A>) -> Self {
        Self(field0)
    }

    /// Creates a `LieGroupH1` instance from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V3<A>) -> Self {
        return exponential_coordinates.to_lie_alg_h1().exp();
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieGroupH1<A2> {
        LieGroupH1::new(self.0.to_other_ad_type::<A2>())
    }

    pub fn to_constant_ad(&self) -> Self {
        Self::new(self.0.to_constant_ad())
    }
}

/// A struct representing a Lie algebra element `H1`, implemented using `Quaternion<f64>`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgH1<A: AD>(#[serde(deserialize_with = "Q::<A>::deserialize")] pub Q<A>);
impl<A: AD> LieAlgH1<A> {
    /// Creates a new `LieAlgH1` instance from a `Q` (quaternion).
    ///
    /// - `field0`: The quaternion representing the Lie algebra element.
    #[inline(always)]
    pub fn new(field0: Q<A>) -> Self {
        Self(field0)
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieAlgH1<A2> {
        LieAlgH1::new(self.0.to_other_ad_type::<A2>())
    }

    pub fn to_constant_ad(&self) -> Self {
        Self::new(self.0.to_constant_ad())
    }
}

impl<A: AD> LieGroupElement<A> for LieGroupH1<A> {
    type LieAlgebraElementType = LieAlgH1<A>;

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
        let w = self.0.w.min(A::constant(1.0));
        let acos = w.acos();
        return if acos == A::zero() {
            LieAlgH1::new(Q::new(A::zero(), A::zero(), A::zero(), A::zero()))
        } else {
            let ss = acos / acos.sin();
            LieAlgH1::new(Q::new(A::zero(), ss * self.0.i, ss * self.0.j, ss * self.0.k))
        }
    }
}

/// Implements the `Rotation3DLieGroupElement` trait for `LieGroupH1`.
impl<A: AD> Rotation3DLieGroupElement<A> for LieGroupH1<A> {}

impl<A: AD> LieAlgebraElement<A> for LieAlgH1<A> {
    type LieGroupElementType = LieGroupH1<A>;
    type EuclideanSpaceElementType = V3<A>;

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
    fn scale(&self, scalar: A) -> Self {
        Self::new(self.0 * scalar)
    }

    /// Exponentiates the Lie algebra element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let v = self.0.imag();
        let vn = v.norm();
        return if vn == A::zero() {
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
pub trait ApolloQuaternionH1LieTrait<A: AD> {
    /// Converts a quaternion to a `LieAlgH1` element.
    fn to_lie_alg_h1(&self) -> LieAlgH1<A>;
}

impl<A: AD> ApolloQuaternionH1LieTrait<A> for Q<A> {
    fn to_lie_alg_h1(&self) -> LieAlgH1<A> {
        LieAlgH1::new(*self)
    }
}

/// Trait for converting unit quaternions to `LieGroupH1`.
pub trait ApolloUnitQuaternionH1LieTrait<A: AD> {
    /// Converts a unit quaternion to a `LieGroupH1` element.
    fn to_lie_group_h1(&self) -> LieGroupH1<A>;
}

impl<A: AD> ApolloUnitQuaternionH1LieTrait<A> for UQ<A> {
    fn to_lie_group_h1(&self) -> LieGroupH1<A> {
        LieGroupH1::new(*self)
    }
}

/// Trait for converting vectors to `LieAlgH1`.
pub trait ApolloLieAlgPackH1Trait<A: AD> {
    /// Converts a vector to a `LieAlgH1` element.
    fn to_lie_alg_h1(&self) -> LieAlgH1<A>;
}

impl<A: AD> ApolloLieAlgPackH1Trait<A> for V3<A> {
    #[inline(always)]
    fn to_lie_alg_h1(&self) -> LieAlgH1<A> {
        LieAlgH1::new(Q::new(A::zero(), self[0], self[1], self[2]))
    }
}
