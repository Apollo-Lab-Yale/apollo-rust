use serde::{Deserialize, Serialize};
use apollo_lie::{LieAlgebraElement, LieGroupElement};
use crate::isometry3::I3M;
use crate::lie::so3::{ApolloLieAlgPackSO3Trait, ApolloMatrix3SO3LieTrait, ApolloRotation3SO3LieTrait};
use crate::lie::TranslationAndRotation3DLieGroupElement;
use crate::matrices::M3;
use crate::vectors::{ApolloVector3Trait2, V3, V6};


pub type ISE3 = LieGroupISE3;
pub type Ise3 = LieAlgISE3;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupISE3(pub I3M);
impl LieGroupISE3 {
    pub fn new(field0: I3M) -> Self {
        Self(field0)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgISE3 { m: M3, v: V3 }
impl LieAlgISE3 {
    pub fn new(m: M3, v: V3) -> Self {
        Self { m, v }
    }
    #[inline(always)]
    pub fn m(&self) -> &M3 {
        &self.m
    }
    #[inline(always)]
    pub fn v(&self) -> &V3 {
        &self.v
    }
}

impl LieGroupElement for ISE3 {
    type LieAlgebraElementType = LieAlgISE3;

    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        ISE3::new(self.0*other.0)
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        ISE3::new(self.0.inverse())
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let a_mat = self.0.rotation.to_lie_group_so3().ln();
        let u = a_mat.vee();
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

        Ise3::new(a_mat.0, b)
    }
}
impl TranslationAndRotation3DLieGroupElement for LieGroupISE3 { }

impl LieAlgebraElement for Ise3 {
    type LieGroupElementType = ISE3;
    type EuclideanSpaceElementType = V6;

    #[inline(always)]
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_ise3()
    }

    #[inline(always)]
    fn scale(&self, scalar: f64) -> Self {
        return Self::new(self.m.scale(scalar), self.v.scale(scalar))
    }

    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let a_mat = self.m.to_lie_alg_so3();
        let u = a_mat.vee();
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
        let r_mat = a_mat.exp();

        ISE3::new(I3M::from_parts(t.to_translation(), r_mat.0))
    }

    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let u = self.m.to_lie_alg_so3().vee();
        let v = &self.v;

        V6::new(u[0], u[1], u[2], v[0], v[1], v[2])
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloLieAlgPackIse3Trait {
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
