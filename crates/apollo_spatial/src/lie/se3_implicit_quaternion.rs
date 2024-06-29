use serde::{Deserialize, Serialize};
use apollo_lie::{LieAlgebraElement, LieGroupElement};
use crate::isometry3::I3;
use crate::lie::h1::{ApolloLieAlgPackH1Trait, ApolloQuaternionH1LieTrait, ApolloUnitQuaternionH1LieTrait};
use crate::lie::so3::ApolloLieAlgPackSO3Trait;
use crate::lie::TranslationAndRotation3DLieGroupElement;
use crate::matrices::M3;
use crate::quaternions::Q;
use crate::translations::ApolloTranslation3;
use crate::vectors::{ApolloVector3Trait2, V3, V6};

pub type ISE3q = LieGroupISE3q;
pub type Ise3q = LieAlgISE3q;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupISE3q(pub I3);
impl LieGroupISE3q {
    #[inline(always)]
    pub fn new(field0: I3) -> Self {
        Self(field0)
    }

    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V6) -> Self {
        return exponential_coordinates.to_lie_alg_ise3q().exp()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgISE3q { q: Q, v: V3 }
impl LieAlgISE3q {
    pub fn new(q: Q, v: V3) -> Self {
        Self { q, v }
    }
    #[inline(always)]
    pub fn q(&self) -> &Q {
        &self.q
    }
    #[inline(always)]
    pub fn v(&self) -> &V3 {
        &self.v
    }
}

impl LieGroupElement for LieGroupISE3q {
    type LieAlgebraElementType = LieAlgISE3q;

    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(I3::identity())
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.inverse())
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let a_quat = self.0.rotation.to_lie_group_h1().ln();
        let u = a_quat.0.imag();
        let a_mat = u.to_lie_alg_so3();
        let beta = u.norm();

        let (p, q) = if beta.abs() < 0.0001 {
            let pp = 0.5 - (beta.powi(2) / 24.0) + (beta.powi(4) / 720.0);
            let qq = (1.0/6.0) - (beta.powi(2) / 120.0) + (beta.powi(4) / 5040.0);
            (pp, qq)
        } else {
            (
                (1.0 - f64::cos(beta)) / f64::powi(beta, 2),
                (beta - f64::sin(beta)) / f64::powi(beta, 3)
            )
        };

        let c_mat = M3::identity() + p*a_mat.0 + q*a_mat.0.pow(2);
        let c_inv = c_mat.try_inverse().expect("error");

        let b = c_inv * self.0.translation.vector;

        Ise3q::new(a_quat.0, b)
    }
}
impl TranslationAndRotation3DLieGroupElement for LieGroupISE3q { }

impl LieAlgebraElement for LieAlgISE3q {
    type LieGroupElementType = LieGroupISE3q;
    type EuclideanSpaceElementType = V6;

    #[inline(always)]
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_ise3q()
    }

    #[inline(always)]
    fn scale(&self, scalar: f64) -> Self {
        return Self::new(scalar*self.q, self.v.scale(scalar))
    }

    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let u = self.q.to_lie_alg_h1().vee();
        let a_mat = u.to_lie_alg_so3();
        let beta = u.norm();

        let (p, q) = if beta.abs() < 0.0001 {
            let pp = 0.5 - (beta.powi(2) / 24.0) + (beta.powi(4) / 720.0);
            let qq = (1.0/6.0) - (beta.powi(2) / 120.0) + (beta.powi(4) / 5040.0);
            (pp, qq)
        } else {
            (
                (1.0 - f64::cos(beta)) / f64::powi(beta, 2),
                (beta - f64::sin(beta)) / f64::powi(beta, 3)
            )
        };

        let c_mat = M3::identity() + p*a_mat.0 + q*a_mat.0.pow(2);
        let t = c_mat * self.v;
        let q = self.q.to_lie_alg_h1().exp();

        ISE3q::new(I3::from_parts(t.to_translation(), q.0))
    }

    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let u = self.q.to_lie_alg_h1().vee();
        let v = &self.v;

        V6::new(u[0], u[1], u[2], v[0], v[1], v[2])
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloLieAlgPackIse3qTrait {
    fn to_lie_alg_ise3q(&self) -> LieAlgISE3q;
}
impl ApolloLieAlgPackIse3qTrait for V6 {
    #[inline(always)]
    fn to_lie_alg_ise3q(&self) -> LieAlgISE3q {
        let u = V3::new(self[0], self[1], self[2]);
        let q = u.to_lie_alg_h1();
        let v = V3::new(self[3], self[4], self[5]);

        LieAlgISE3q::new(q.0, v)
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct PseudoLieGroupISE3q(pub I3);
impl PseudoLieGroupISE3q {
    #[inline(always)]
    pub fn new(field0: I3) -> Self {
        Self(field0)
    }

    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V6) -> Self {
        return exponential_coordinates.to_pseudo_lie_alg_ise3q().exp()
    }
}

#[derive(Clone, Debug)]
pub struct PseudoLieAlgISE3q { q: Q, v: V3 }
impl PseudoLieAlgISE3q {
    pub fn new(q: Q, v: V3) -> Self {
        Self { q, v }
    }
    #[inline(always)]
    pub fn q(&self) -> &Q {
        &self.q
    }
    #[inline(always)]
    pub fn v(&self) -> &V3 {
        &self.v
    }
}

impl LieGroupElement for PseudoLieGroupISE3q {
    type LieAlgebraElementType = PseudoLieAlgISE3q;

    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    fn identity_element() -> Self {
        Self::new(I3::identity())
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.inverse())
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let q = self.0.rotation.to_lie_group_h1().ln();
        PseudoLieAlgISE3q::new(q.0, self.0.translation.to_vector3())
    }
}

impl LieAlgebraElement for PseudoLieAlgISE3q {
    type LieGroupElementType = PseudoLieGroupISE3q;
    type EuclideanSpaceElementType = V6;

    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_pseudo_lie_alg_ise3q()
    }

    #[inline(always)]
    fn scale(&self, scalar: f64) -> Self {
        return Self::new(scalar*self.q, self.v.scale(scalar))
    }

    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let q = self.q.to_lie_alg_h1().exp();
        PseudoLieGroupISE3q::new(I3::from_parts(self.v.to_translation(), q.0))
    }

    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let u = self.q.to_lie_alg_h1().vee();
        let v = &self.v;

        V6::new(u[0], u[1], u[2], v[0], v[1], v[2])
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloPseudoLieAlgPackIse3qTrait {
    fn to_pseudo_lie_alg_ise3q(&self) -> PseudoLieAlgISE3q;
}
impl ApolloPseudoLieAlgPackIse3qTrait for V6 {
    #[inline(always)]
    fn to_pseudo_lie_alg_ise3q(&self) -> PseudoLieAlgISE3q {
        let u = V3::new(self[0], self[1], self[2]);
        let q = u.to_lie_alg_h1();
        let v = V3::new(self[3], self[4], self[5]);

        PseudoLieAlgISE3q::new(q.0, v)
    }
}