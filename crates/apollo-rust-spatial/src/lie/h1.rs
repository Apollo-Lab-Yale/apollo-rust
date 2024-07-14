use serde::{Deserialize, Serialize};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation3DLieGroupElement;
use crate::quaternions::{Q, UQ};
use crate::vectors::V3;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupH1(pub UQ);
impl LieGroupH1 {
    #[inline(always)]
    pub fn new(field0: UQ) -> Self {
        Self(field0)
    }
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V3) -> Self {
        return exponential_coordinates.to_lie_alg_h1().exp();
    }
}


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgH1(pub Q);
impl LieAlgH1 {
    #[inline(always)]
    pub fn new(field0: Q) -> Self {
        Self(field0)
    }
}

impl LieGroupElement for LieGroupH1 {
    type LieAlgebraElementType = LieAlgH1;

    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0*other.0)
    }

    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(UQ::identity())
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.inverse())
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let w = self.0.w;
        let acos = w.acos();
        return if acos == 0.0 {
            LieAlgH1::new(Q::new(0.,0.,0.,0.))
        } else {
            let ss = acos / acos.sin();
            LieAlgH1::new(Q::new( 0.0, ss*self.0.i, ss*self.0.j, ss*self.0.k ))
        }
    }
}
impl Rotation3DLieGroupElement for LieGroupH1 { }

impl LieAlgebraElement for LieAlgH1 {
    type LieGroupElementType = LieGroupH1;
    type EuclideanSpaceElementType = V3;

    #[inline(always)]
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_h1()
    }

    #[inline(always)]
    fn scale(&self, scalar: f64) -> Self {
        Self::new(scalar*self.0)
    }

    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let v = self.0.imag();
        let vn = v.norm();
        return if vn == 0.0 {
            LieGroupH1::new(UQ::identity())
        } else {
            let cc = vn.cos();
            let ss = vn.sin() / vn;
            LieGroupH1::new(UQ::new_unchecked(Q::new(cc, ss*v[0], ss*v[1], ss*v[2])))
        }
    }

    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        self.0.imag()
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloQuaternionH1LieTrait {
    fn to_lie_alg_h1(&self) -> LieAlgH1;
}
impl ApolloQuaternionH1LieTrait for Q {
    fn to_lie_alg_h1(&self) -> LieAlgH1 {
        LieAlgH1::new(*self)
    }
}

pub trait ApolloUnitQuaternionH1LieTrait {
    fn to_lie_group_h1(&self) -> LieGroupH1;
}
impl ApolloUnitQuaternionH1LieTrait for UQ {
    fn to_lie_group_h1(&self) -> LieGroupH1 {
        LieGroupH1::new(*self)
    }
}

pub trait ApolloLieAlgPackH1Trait {
    fn to_lie_alg_h1(&self) -> LieAlgH1;
}
impl ApolloLieAlgPackH1Trait for V3 {
    #[inline(always)]
    fn to_lie_alg_h1(&self) -> LieAlgH1 {
        LieAlgH1::new(Q::new(0.0, self[0], self[1], self[2]))
    }
}