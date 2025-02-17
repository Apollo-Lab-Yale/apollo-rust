use serde::{Deserialize, Serialize};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use crate::isometry3::I3M;
use crate::lie::so3::{ApolloLieAlgPackSO3Trait, ApolloMatrix3SO3LieTrait, ApolloRotation3SO3LieTrait};
use crate::lie::TranslationAndRotation3DLieGroupElement;
use crate::matrices::M3;
use crate::vectors::{ApolloVector3Trait2, V3, V6};

/// Alias for `LieGroupISE3`, representing a 3D Lie group element in SE(3).
pub type ISE3 = LieGroupISE3;

/// Alias for `LieAlgISE3`, representing a 3D Lie algebra element in se(3).
pub type Ise3 = LieAlgISE3;

/// Struct representing a 3D Lie group element in SE(3) based on `I3M`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupISE3(pub I3M);
impl LieGroupISE3 {
    /// Creates a new `LieGroupISE3` instance.
    ///
    /// - `field0`: The isometry matrix `I3M`.
    #[inline(always)]
    pub fn new(field0: I3M) -> Self {
        Self(field0)
    }

    /// Constructs a `LieGroupISE3` element from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V6) -> Self {
        return exponential_coordinates.to_lie_alg_ise3().exp()
    }
}

/// Struct representing a 3D Lie algebra element in se(3) consisting of a matrix `M3` and vector `V3`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgISE3 {
    m: M3,
    v: V3,
}

impl LieAlgISE3 {
    /// Creates a new `LieAlgISE3` instance.
    ///
    /// - `m`: The matrix `M3` representing the rotational part.
    /// - `v`: The vector `V3` representing the translational part.
    pub fn new(m: M3, v: V3) -> Self {
        Self { m, v }
    }

    /// Returns a reference to the matrix `M3` representing the rotational part.
    #[inline(always)]
    pub fn m(&self) -> &M3 {
        &self.m
    }

    /// Returns a reference to the vector `V3` representing the translational part.
    #[inline(always)]
    pub fn v(&self) -> &V3 {
        &self.v
    }
}

impl LieGroupElement for ISE3 {
    type LieAlgebraElementType = LieAlgISE3;

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

        let (p, q) = if beta.abs() < 0.0001 {
            let pp = 0.5 - (beta.powi(2) / 24.0) + (beta.powi(4) / 720.0);
            let qq = (1.0 / 6.0) - (beta.powi(2) / 120.0) + (beta.powi(4) / 5040.0);
            (pp, qq)
        } else {
            (
                (1.0 - f64::cos(beta)) / f64::powi(beta, 2),
                (beta - f64::sin(beta)) / f64::powi(beta, 3),
            )
        };

        let c_mat = M3::identity() + p * a_mat.0 + q * a_mat.0.pow(2);
        let c_inv = c_mat.try_inverse().expect("error");

        let b = c_inv * self.0.translation.vector;

        Ise3::new(a_mat.0, b)
    }
}

/// Implements `TranslationAndRotation3DLieGroupElement` trait for `LieGroupISE3`.
impl TranslationAndRotation3DLieGroupElement for LieGroupISE3 {}

impl LieAlgebraElement for Ise3 {
    type LieGroupElementType = ISE3;
    type EuclideanSpaceElementType = V6;

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
    fn scale(&self, scalar: f64) -> Self {
        return Self::new(self.m.scale(scalar), self.v.scale(scalar))
    }

    /// Exponentiates the `LieAlgISE3` element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let a_mat = self.m.to_lie_alg_so3();
        let u = a_mat.vee();
        let beta = u.norm();

        let (p, q) = if beta.abs() < 0.0001 {
            let pp = 0.5 - (beta.powi(2) / 24.0) + (beta.powi(4) / 720.0);
            let qq = (1.0 / 6.0) - (beta.powi(2) / 120.0) + (beta.powi(4) / 5040.0);
            (pp, qq)
        } else {
            (
                (1.0 - f64::cos(beta)) / f64::powi(beta, 2),
                (beta - f64::sin(beta)) / f64::powi(beta, 3),
            )
        };

        let c_mat = M3::identity() + p * a_mat.0 + q * a_mat.0.pow(2);
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
pub trait ApolloLieAlgPackIse3Trait {
    /// Converts a vector to a `LieAlgISE3` element.
    fn to_lie_alg_ise3(&self) -> LieAlgISE3;
}

impl ApolloLieAlgPackIse3Trait for V6 {
    #[inline(always)]
    fn to_lie_alg_ise3(&self) -> LieAlgISE3 {
        let u = V3::new(self[0], self[1], self[2]);
        let m = u.to_lie_alg_so3();
        let v = V3::new(self[3], self[4], self[5]);

        LieAlgISE3::new(m.0, v)
    }
}
