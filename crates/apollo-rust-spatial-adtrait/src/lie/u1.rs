use ad_trait::AD;
use nalgebra::Complex;
use serde::{Deserialize, Serialize};
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use crate::complex_numbers::{C, UC};
use crate::lie::Rotation2DLieGroupElement;

/// Struct representing a Lie group element in U(1) rotation using unit complex numbers.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupU1<A: AD>(#[serde(deserialize_with = "UC::<A>::deserialize")] pub UC<A>);

impl<A: AD> LieGroupU1<A> {
    /// Creates a new `LieGroupU1` instance.
    ///
    /// - `field0`: The unit complex number (`UC`) representing the Lie group element.
    pub fn new(field0: UC<A>) -> Self {
        Self(field0)
    }
}

/// Struct representing a Lie algebra element in u(1) using complex numbers.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgU1<A: AD>(#[serde(deserialize_with = "C::<A>::deserialize")] pub C<A>);

impl<A: AD> LieAlgU1<A> {
    /// Creates a new `LieAlgU1` instance.
    ///
    /// - `field0`: The complex number (`C`) representing the Lie algebra element. Must satisfy `re = 0`.
    pub fn new(field0: C<A>) -> Self {
        assert_eq!(field0.re, A::zero());

        Self(field0)
    }
}

impl<A: AD> LieGroupElement<A> for LieGroupU1<A> {
    type LieAlgebraElementType = LieAlgU1<A>;

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
        let atan2 = A::atan2(b, a);
        LieAlgU1::new(C::new(A::zero(), atan2))
    }
}

/// Implements the `Rotation2DLieGroupElement` trait for `LieGroupU1`.
impl<A: AD> Rotation2DLieGroupElement<A> for LieGroupU1<A> {}

impl<A: AD> LieAlgebraElement<A> for LieAlgU1<A> {
    type LieGroupElementType = LieGroupU1<A>;
    type EuclideanSpaceElementType = A;

    /// Converts a Euclidean space element (a scalar) to a `LieAlgU1` element.
    ///
    /// - `e`: The scalar representing the element in Euclidean space.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_u1()
    }

    /// Scales the `LieAlgU1` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    fn scale(&self, scalar: A) -> Self {
        Self::new(self.0 * scalar)
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
pub trait ApolloComplexU1LieTrait<A: AD> {
    /// Converts the complex number `C` to a `LieAlgU1` element.
    fn to_lie_alg_u1(&self) -> LieAlgU1<A>;
}

impl<A: AD> ApolloComplexU1LieTrait<A> for C<A> {
    #[inline(always)]
    fn to_lie_alg_u1(&self) -> LieAlgU1<A> {
        LieAlgU1::new(*self)
    }
}

/// Trait for converting a unit complex number (`UC`) to a `LieGroupU1` element.
pub trait ApolloUnitComplexU1LieTrait<A: AD> {
    /// Converts the unit complex number `UC` to a `LieGroupU1` element.
    fn to_lie_group_u1(&self) -> LieGroupU1<A>;
}

impl<A: AD> ApolloUnitComplexU1LieTrait<A> for UC<A> {
    #[inline(always)]
    fn to_lie_group_u1(&self) -> LieGroupU1<A> {
        LieGroupU1::new(*self)
    }
}

/// Trait for converting a scalar to a `LieAlgU1` element.
pub trait ApolloLieAlgPackU1Trait<A: AD> {
    /// Converts the scalar to a `LieAlgU1` element.
    fn to_lie_alg_u1(&self) -> LieAlgU1<A>;
}

impl<A: AD> ApolloLieAlgPackU1Trait<A> for A {
    fn to_lie_alg_u1(&self) -> LieAlgU1<A> {
        LieAlgU1::new(Complex::new(A::constant(0.0), *self))
    }
}