use nalgebra::Complex;
use apollo_lie::{LieAlgebraElement, LieGroupElement};
use crate::complex_numbers::{C, UC};

#[derive(Clone, Debug)]
pub struct LieGroupU1(pub UC);
impl LieGroupU1 {
    pub fn new(field0: UC) -> Self {
        Self(field0)
    }
}

#[derive(Clone, Debug)]
pub struct LieAlgU1(pub C);
impl LieAlgU1 {
    pub fn new(field0: C) -> Self {
        assert_eq!(field0.re, 0.0);

        Self(field0)
    }
}

impl LieGroupElement for LieGroupU1 {
    type LieAlgebraElementType = LieAlgU1;

    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        LieGroupU1::new(self.0 * other.0)
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        LieGroupU1::new(self.0.conjugate())
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let a = self.0.re;
        let b = self.0.im;
        let atan2 = f64::atan2(b, a);
        LieAlgU1::new(C::new(0.0, atan2))
    }
}
impl LieAlgebraElement for LieAlgU1 {
    type LieGroupElementType = LieGroupU1;
    type EuclideanSpaceElementType = f64;

    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let alpha = self.0.im;
        return LieGroupU1::new(UC::new(alpha))
    }

    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let alpha = self.0.im;
        return alpha
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloComplexU1LieTrait {
    fn to_lie_alg_u1(&self) -> LieAlgU1;
}
impl ApolloComplexU1LieTrait for C {
    #[inline(always)]
    fn to_lie_alg_u1(&self) -> LieAlgU1 {
        LieAlgU1::new(*self)
    }
}

pub trait ApolloUnitComplexU1LieTrait {
    fn to_lie_group_u1(&self) -> LieGroupU1;
}
impl ApolloUnitComplexU1LieTrait for UC {
    #[inline(always)]
    fn to_lie_group_u1(&self) -> LieGroupU1 { LieGroupU1::new(*self) }
}

pub trait ApolloLieAlgPackU1Trait {
    fn to_lie_alg_u1(&self) -> LieAlgU1;
}
impl ApolloLieAlgPackU1Trait for f64 {
    fn to_lie_alg_u1(&self) -> LieAlgU1 {
        LieAlgU1::new(Complex::new(0.0, *self))
    }
}
