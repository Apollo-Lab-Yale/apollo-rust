use nalgebra::Complex;
use serde::{Deserialize, Serialize};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use crate::complex_numbers::{C, UC};
use crate::lie::Rotation2DLieGroupElement;

/// Struct representing a Lie group element in U(1) rotation using unit complex numbers.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupU1(pub UC);

impl LieGroupU1 {
    /// Creates a new `LieGroupU1` instance.
    ///
    /// - `field0`: The unit complex number (`UC`) representing the Lie group element.
    pub fn new(field0: UC) -> Self {
        Self(field0)
    }
}

/// Struct representing a Lie algebra element in u(1) using complex numbers.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgU1(pub C);

impl LieAlgU1 {
    /// Creates a new `LieAlgU1` instance.
    ///
    /// - `field0`: The complex number (`C`) representing the Lie algebra element. Must satisfy `re = 0`.
    pub fn new(field0: C) -> Self {
        assert_eq!(field0.re, 0.0);

        Self(field0)
    }
}

impl LieGroupElement for LieGroupU1 {
    type LieAlgebraElementType = LieAlgU1;

    /// Implements the group operator for the `U1` Lie group element.
    ///
    /// - `other`: Another `LieGroupU1` element to multiply with.
    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        LieGroupU1::new(self.0 * other.0)
    }

    /// Returns the identity element of the `U1` Lie group.
    fn identity_element() -> Self {
        Self::new(UC::identity())
    }

    /// Returns the inverse of the current Lie group element (complex conjugate).
    #[inline(always)]
    fn inverse(&self) -> Self {
        LieGroupU1::new(self.0.conjugate())
    }

    /// Returns the logarithm (Lie algebra element) of the current Lie group element.
    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let a = self.0.re;
        let b = self.0.im;
        let atan2 = f64::atan2(b, a);
        LieAlgU1::new(C::new(0.0, atan2))
    }
}

/// Implements the `Rotation2DLieGroupElement` trait for `LieGroupU1`.
impl Rotation2DLieGroupElement for LieGroupU1 {}

impl LieAlgebraElement for LieAlgU1 {
    type LieGroupElementType = LieGroupU1;
    type EuclideanSpaceElementType = f64;

    /// Converts a Euclidean space element (a scalar) to a `LieAlgU1` element.
    ///
    /// - `e`: The scalar representing the element in Euclidean space.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_u1()
    }

    /// Scales the `LieAlgU1` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    fn scale(&self, scalar: f64) -> Self {
        Self::new(scalar * self.0)
    }

    /// Exponentiates the `LieAlgU1` element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let alpha = self.0.im;
        return LieGroupU1::new(UC::new(alpha))
    }

    /// Converts the `LieAlgU1` element to its corresponding Euclidean space element (a scalar).
    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let alpha = self.0.im;
        return alpha
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Trait for converting a complex number (`C`) to a `LieAlgU1` element.
pub trait ApolloComplexU1LieTrait {
    /// Converts the complex number `C` to a `LieAlgU1` element.
    fn to_lie_alg_u1(&self) -> LieAlgU1;
}

impl ApolloComplexU1LieTrait for C {
    #[inline(always)]
    fn to_lie_alg_u1(&self) -> LieAlgU1 {
        LieAlgU1::new(*self)
    }
}

/// Trait for converting a unit complex number (`UC`) to a `LieGroupU1` element.
pub trait ApolloUnitComplexU1LieTrait {
    /// Converts the unit complex number `UC` to a `LieGroupU1` element.
    fn to_lie_group_u1(&self) -> LieGroupU1;
}

impl ApolloUnitComplexU1LieTrait for UC {
    #[inline(always)]
    fn to_lie_group_u1(&self) -> LieGroupU1 {
        LieGroupU1::new(*self)
    }
}

/// Trait for converting a scalar to a `LieAlgU1` element.
pub trait ApolloLieAlgPackU1Trait {
    /// Converts the scalar to a `LieAlgU1` element.
    fn to_lie_alg_u1(&self) -> LieAlgU1;
}

impl ApolloLieAlgPackU1Trait for f64 {
    fn to_lie_alg_u1(&self) -> LieAlgU1 {
        LieAlgU1::new(Complex::new(0.0, *self))
    }
}
