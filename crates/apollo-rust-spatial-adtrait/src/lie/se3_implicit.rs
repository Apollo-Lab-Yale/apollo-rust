use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use crate::isometry3::{ApolloIsometryMatrix3Trait, I3M};
use crate::lie::so3::{ApolloLieAlgPackSO3Trait, ApolloMatrix3SO3LieTrait, ApolloRotation3SO3LieTrait};
use crate::lie::TranslationAndRotation3DLieGroupElement;
use crate::matrices::{ApolloMatrix3ADTrait, M3};
use crate::vectors::{ApolloVector3ADTrait, ApolloVector3Trait2, V3, V6};

/// Alias for `LieGroupISE3`, representing a 3D Lie group element in SE(3).
pub type ISE3<A> = LieGroupISE3<A>;

/// Alias for `LieAlgISE3`, representing a 3D Lie algebra element in se(3).
pub type Ise3<A> = LieAlgISE3<A>;

/// Struct representing a 3D Lie group element in SE(3) based on `I3M`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupISE3<A: AD>(#[serde(deserialize_with = "I3M::<A>::deserialize")] pub I3M<A>);
impl<A: AD> LieGroupISE3<A> {
    /// Creates a new `LieGroupISE3` instance.
    ///
    /// - `field0`: The isometry matrix `I3M`.
    #[inline(always)]
    pub fn new(field0: I3M<A>) -> Self {
        Self(field0)
    }

    /// Constructs a `LieGroupISE3` element from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V6<A>) -> Self {
        return exponential_coordinates.to_lie_alg_ise3().exp()
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieGroupISE3<A2> {
        LieGroupISE3(self.0.to_other_ad_type::<A2>())
    }

    pub fn to_constant_ad(&self) -> Self {
        Self::new(self.0.to_constant_ad())
    }
}

/// Struct representing a 3D Lie algebra element in se(3) consisting of a matrix `M3` and vector `V3`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgISE3<A: AD> {
    #[serde(deserialize_with = "M3::<A>::deserialize")]
    m: M3<A>,
    #[serde(deserialize_with = "V3::<A>::deserialize")]
    v: V3<A>,
}
impl<A: AD> LieAlgISE3<A> {
    /// Creates a new `LieAlgISE3` instance.
    ///
    /// - `m`: The matrix `M3` representing the rotational part.
    /// - `v`: The vector `V3` representing the translational part.
    pub fn new(m: M3<A>, v: V3<A>) -> Self {
        Self { m, v }
    }

    /// Returns a reference to the matrix `M3` representing the rotational part.
    #[inline(always)]
    pub fn m(&self) -> &M3<A> {
        &self.m
    }

    /// Returns a reference to the vector `V3` representing the translational part.
    #[inline(always)]
    pub fn v(&self) -> &V3<A> {
        &self.v
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieAlgISE3<A2> {
        LieAlgISE3 {
            m: self.m.to_other_ad_type::<A2>(),
            v: self.v.to_other_ad_type::<A2>(),
        }
    }

    pub fn to_constant_ad(&self) -> Self {
        Self {
            m: self.m.to_constant_ad(),
            v: self.v.to_constant_ad(),
        }
    }
}
impl<A: AD> LieGroupElement<A> for ISE3<A> {
    type LieAlgebraElementType = LieAlgISE3<A>;

    /// Implements the group operator for the `ISE3` Lie group element.
    ///
    /// - `other`: Another `ISE3` element to multiply with.
    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        ISE3::new(self.0 * other.0)
    }

    /// Returns the identity element for the `ISE3` Lie group.
    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(I3M::identity())
    }

    /// Returns the inverse of the current Lie group element.
    #[inline(always)]
    fn inverse(&self) -> Self {
        ISE3::new(self.0.inverse())
    }

    /// Returns the logarithm (Lie algebra element) of the current Lie group element.
    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let a_mat = self.0.rotation.to_lie_group_so3().ln();
        let u = a_mat.vee();
        let beta = u.norm();

        let (p, q) = if beta.abs() < A::constant(0.0001) {
            let pp = A::constant(0.5) - (beta.powi(2) / A::constant(24.0)) + (beta.powi(4) / A::constant(720.0));
            let qq = (A::constant(1.0 / 6.0)) - (beta.powi(2) / A::constant(120.0)) + (beta.powi(4) / A::constant(5040.0));
            (pp, qq)
        } else {
            (
                (A::constant(1.0) - A::cos(beta)) / A::powi(beta, 2),
                (beta - A::sin(beta)) / A::powi(beta, 3),
            )
        };

        let c_mat = M3::identity() + a_mat.0 * p +  a_mat.0.pow(2) * q;
        let c_inv = c_mat.try_inverse().expect("error");

        let b = c_inv * self.0.translation.vector;

        Ise3::new(a_mat.0, b)
    }
}

/// Implements `TranslationAndRotation3DLieGroupElement` trait for `LieGroupISE3`.
impl<A: AD> TranslationAndRotation3DLieGroupElement<A> for LieGroupISE3<A> {}

impl<A: AD> LieAlgebraElement<A> for Ise3<A> {
    type LieGroupElementType = ISE3<A>;
    type EuclideanSpaceElementType = V6<A>;

    /// Converts a Euclidean space element (vector) to a `LieAlgISE3` element.
    ///
    /// - `e`: The Euclidean space vector representing the element.
    #[inline(always)]
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_ise3()
    }

    /// Scales the `LieAlgISE3` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    #[inline(always)]
    fn scale(&self, scalar: A) -> Self {
        return Self::new(self.m.scale(scalar), self.v.scale(scalar))
    }

    /// Exponentiates the `LieAlgISE3` element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let a_mat = self.m.to_lie_alg_so3();
        let u = a_mat.vee();
        let beta = u.norm();

        let (p, q) = if beta.abs() < A::constant(0.0001) {
            let pp = A::constant(0.5) - (beta.powi(2) / A::constant(24.0)) + (beta.powi(4) / A::constant(720.0));
            let qq = A::constant(1.0 / 6.0) - (beta.powi(2) / A::constant(120.0)) + (beta.powi(4) / A::constant(5040.0));
            (pp, qq)
        } else {
            (
                (A::constant(1.0) - A::cos(beta)) / A::powi(beta, 2),
                (beta - A::sin(beta)) / A::powi(beta, 3),
            )
        };

        let c_mat = M3::identity() + a_mat.0 * p + a_mat.0.pow(2) * q;
        let t = c_mat * self.v;
        let r_mat = a_mat.exp();

        ISE3::new(I3M::from_parts(t.to_translation(), r_mat.0))
    }

    /// Converts the `LieAlgISE3` element to its corresponding Euclidean space vector.
    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let u = self.m.to_lie_alg_so3().vee();
        let v = &self.v;

        V6::new(u[0], u[1], u[2], v[0], v[1], v[2])
    }
}

/// Trait for converting vectors to `LieAlgISE3`.
pub trait ApolloLieAlgPackIse3Trait<A: AD> {
    /// Converts a vector to a `LieAlgISE3` element.
    fn to_lie_alg_ise3(&self) -> LieAlgISE3<A>;
}

impl<A: AD> ApolloLieAlgPackIse3Trait<A> for V6<A> {
    #[inline(always)]
    fn to_lie_alg_ise3(&self) -> LieAlgISE3<A> {
        let u = V3::new(self[0], self[1], self[2]);
        let m = u.to_lie_alg_so3();
        let v = V3::new(self[3], self[4], self[5]);

        LieAlgISE3::new(m.0, v)
    }
}
