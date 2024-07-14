use serde::{Deserialize, Serialize};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation2DLieGroupElement;
use crate::matrices::M2;
use crate::rotation_matrices::R2;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupSO2(pub R2);
impl LieGroupSO2 {
    pub fn new(field0: R2) -> Self {
        Self(field0)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgSO2(pub M2);
impl LieAlgSO2 {
    pub fn new(field0: M2) -> Self {
        assert_eq!(field0.m11, 0.0);
        assert_eq!(field0.m22, 0.0);

        Self(field0)
    }
}

impl LieGroupElement for LieGroupSO2 {
    type LieAlgebraElementType = LieAlgSO2;

    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(R2::identity())
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.transpose())
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let c = self.0[(0,0)];
        let s = self.0[(1,0)];
        let a = f64::atan2(s, c);

        LieAlgSO2::new(
            M2::new(0.0, -a, a, 0.0)
        )
    }
}
impl Rotation2DLieGroupElement for LieGroupSO2 { }
impl LieAlgebraElement for LieAlgSO2 {
    type LieGroupElementType = LieGroupSO2;
    type EuclideanSpaceElementType = f64;

    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_so2()
    }

    fn scale(&self, scalar: f64) -> Self {
        Self::new(scalar * self.0)
    }

    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let a = self.0.m21;
        return LieGroupSO2::new(
            R2::new(a)
        );
    }

    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        return self.0.m21;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloMatrix2SO2LieTrait {
    fn to_lie_alg_so2(&self) -> LieAlgSO2;
}
impl ApolloMatrix2SO2LieTrait for M2 {
    fn to_lie_alg_so2(&self) -> LieAlgSO2 {
        LieAlgSO2::new(*self)
    }
}

pub trait ApolloRotation2SO2LieTrait {
    fn to_lie_group_so2(&self) -> LieGroupSO2;
}
impl ApolloRotation2SO2LieTrait for R2 {
    #[inline(always)]
    fn to_lie_group_so2(&self) -> LieGroupSO2 {
        LieGroupSO2::new(*self)
    }
}

pub trait ApolloLieAlgPackSO2Trait {
    fn to_lie_alg_so2(&self) -> LieAlgSO2;
}
impl ApolloLieAlgPackSO2Trait for f64 {
    fn to_lie_alg_so2(&self) -> LieAlgSO2 {
        LieAlgSO2::new(M2::new(0.0, -*self, *self, 0.0))
    }
}