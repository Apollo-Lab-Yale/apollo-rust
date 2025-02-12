use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation2DLieGroupElement;
use crate::matrices::{M2};
use crate::rotation_matrices::{R2};

/// Struct representing a Lie group element in SO(2) rotation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupSO2<A: AD>(#[serde(deserialize_with = "R2::<A>::deserialize")] pub R2<A>);
impl<A: AD> LieGroupSO2<A> {
    /// Creates a new `LieGroupSO2` instance.
    ///
    /// - `field0`: The 2D rotation matrix `R2` representing the Lie group element.
    pub fn new(field0: R2<A>) -> Self {
        Self(field0)
    }
}

/// Struct representing a Lie algebra element in so(2).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgSO2<A: AD>(#[serde(deserialize_with = "M2::<A>::deserialize")] pub M2<A>);
impl<A: AD> LieAlgSO2<A> {
    /// Creates a new `LieAlgSO2` instance.
    ///
    /// - `field0`: The matrix `M2` representing the Lie algebra element. Must satisfy `m11 = 0` and `m22 = 0`.
    pub fn new(field0: M2<A>) -> Self {
        assert_eq!(field0.m11, A::zero());
        assert_eq!(field0.m22, A::zero());

        Self(field0)
    }
}

impl<A: AD> LieGroupElement<A> for LieGroupSO2<A> {
    type LieAlgebraElementType = LieAlgSO2<A>;

    /// Implements the group operator for the `SO2` Lie group element.
    ///
    /// - `other`: Another `LieGroupSO2` element to multiply with.
    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    /// Returns the identity element of the `SO2` Lie group.
    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(R2::identity())
    }

    /// Returns the inverse of the current Lie group element (transposed rotation matrix).
    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.transpose())
    }

    /// Returns the logarithm (Lie algebra element) of the current Lie group element.
    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let c = self.0[(0,0)];
        let s = self.0[(1,0)];
        let a = A::atan2(s, c);

        LieAlgSO2::new(
            M2::new(A::zero(), -a, a, A::zero())
        )
    }
}

/// Implements the `Rotation2DLieGroupElement` trait for `LieGroupSO2`.
impl<A: AD> Rotation2DLieGroupElement<A> for LieGroupSO2<A> {}
impl<A: AD> LieAlgebraElement<A> for LieAlgSO2<A> {
    type LieGroupElementType = LieGroupSO2<A>;
    type EuclideanSpaceElementType = A;

    /// Converts a Euclidean space element (a scalar) to a `LieAlgSO2` element.
    ///
    /// - `e`: The scalar representing the element in Euclidean space.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_so2()
    }

    /// Scales the `LieAlgSO2` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    fn scale(&self, scalar: A) -> Self {
        Self::new(&self.0 * scalar)
    }

    /// Exponentiates the `LieAlgSO2` element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let a = self.0.m21.clone();
        return LieGroupSO2::new(
            R2::new(a)
        );
    }

    /// Converts the `LieAlgSO2` element to its corresponding Euclidean space element (a scalar).
    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        return self.0.m21.clone();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Trait for converting a 2D matrix (`M2`) to a `LieAlgSO2` element.
pub trait ApolloMatrix2SO2LieTrait<A: AD> {
    /// Converts the matrix `M2` to a `LieAlgSO2` element.
    fn to_lie_alg_so2(&self) -> LieAlgSO2<A>;
}

impl<A: AD> ApolloMatrix2SO2LieTrait<A> for M2<A> {
    fn to_lie_alg_so2(&self) -> LieAlgSO2<A> {
        LieAlgSO2::new(*self)
    }
}

/// Trait for converting a 2D rotation matrix (`R2`) to a `LieGroupSO2` element.
pub trait ApolloRotation2SO2LieTrait<A: AD> {
    /// Converts the rotation matrix `R2` to a `LieGroupSO2` element.
    fn to_lie_group_so2(&self) -> LieGroupSO2<A>;
}

impl<A: AD> ApolloRotation2SO2LieTrait<A> for R2<A> {
    #[inline(always)]
    fn to_lie_group_so2(&self) -> LieGroupSO2<A> {
        LieGroupSO2::new(*self)
    }
}

/// Trait for converting a scalar to a `LieAlgSO2` element.
pub trait ApolloLieAlgPackSO2Trait<A: AD> {
    /// Converts the scalar to a `LieAlgSO2` element.
    fn to_lie_alg_so2(&self) -> LieAlgSO2<A>;
}

impl<A: AD> ApolloLieAlgPackSO2Trait<A> for A {
    fn to_lie_alg_so2(&self) -> LieAlgSO2<A> {
        LieAlgSO2::new(M2::new(A::zero(), -*self, *self, A::zero()))
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
/// Struct representing a Lie group element in SO(2) rotation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupSO2AD<A: AD>(
    #[serde(deserialize_with = "R2AD::deserialize")]
    pub R2AD<A>
);
impl<A: AD> LieGroupSO2AD<A> {
    /// Creates a new `LieGroupSO2` instance.
    ///
    /// - `field0`: The 2D rotation matrix `R2` representing the Lie group element.
    pub fn new(field0: R2AD<A>) -> Self {
        Self(field0)
    }
}
*/