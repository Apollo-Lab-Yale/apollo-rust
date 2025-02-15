use std::f64::consts::PI;
use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation3DLieGroupElement;
use crate::matrices::{ApolloMatrix3ADTrait, M3};
use crate::rotation_matrices::{ApolloRotation3Trait, R3};
use crate::vectors::V3;

/// Struct representing a Lie group element in SO(3) rotation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupSO3<A: AD>(#[serde(deserialize_with = "R3::<A>::deserialize")] pub R3<A>);
impl<A: AD> LieGroupSO3<A> {
    /// Creates a new `LieGroupSO3` instance.
    ///
    /// - `field0`: The 3D rotation matrix `R3` representing the Lie group element.
    #[inline(always)]
    pub fn new(field0: R3<A>) -> Self {
        Self(field0)
    }

    /// Constructs a `LieGroupSO3` element from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V3<A>) -> Self {
        exponential_coordinates.to_lie_alg_so3().exp()
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieGroupSO3<A2> {
        LieGroupSO3::new(self.0.to_other_ad_type::<A2>())
    }

    pub fn to_constant_ad(&self) -> Self {
        Self::new(self.0.to_constant_ad())
    }
}

/// Struct representing a Lie algebra element in so(3).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgSO3<A: AD>(#[serde(deserialize_with = "M3::<A>::deserialize")] pub M3<A>);
impl<A: AD> LieAlgSO3<A> {
    /// Creates a new `LieAlgSO3` instance.
    ///
    /// - `field0`: The matrix `M3` representing the Lie algebra element.
    ///   Must satisfy skew-symmetric matrix conditions.
    #[inline(always)]
    pub fn new(field0: M3<A>) -> Self {
        assert_eq!(field0.m11, A::zero());
        assert_eq!(field0.m22, A::zero());
        assert_eq!(field0.m33, A::zero());

        assert_eq!(field0.m12, -field0.m21, "{}", format!("{}, {}", field0.m12, -field0.m21));
        assert_eq!(field0.m13, -field0.m31, "{}", format!("{}, {}", field0.m13, -field0.m31));
        assert_eq!(field0.m23, -field0.m32, "{}", format!("{}, {}", field0.m23, -field0.m32));

        Self(field0)
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieAlgSO3<A2> {
        LieAlgSO3::new(self.0.to_other_ad_type::<A2>())
    }

    pub fn to_constant_ad(&self) -> Self {
        Self::new(self.0.to_constant_ad())
    }
}
impl<A: AD> LieGroupElement<A> for LieGroupSO3<A> {
    type LieAlgebraElementType = LieAlgSO3<A>;

    /// Implements the group operator for the `SO3` Lie group element.
    ///
    /// - `other`: Another `LieGroupSO3` element to multiply with.
    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    /// Returns the identity element of the `SO3` Lie group.
    fn identity_element() -> Self {
        Self::new(R3::identity())
    }

    /// Returns the inverse of the current Lie group element (transposed rotation matrix).
    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.transpose())
    }

    /// Returns the logarithm (Lie algebra element) of the current Lie group element.
    #[inline]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let tr = self.0.matrix().trace();
        let beta = A::acos((tr - A::constant(1.0)) / A::constant(2.0));

        return if beta.abs() < A::constant(0.00001) {
            let f = A::constant(0.5) + (beta.powi(2) / A::constant(12.0)) + (A::constant(7.0) * beta.powi(4) / A::constant(720.0));
            let m = (self.0.matrix() - self.0.matrix().transpose()).scale(f);
            LieAlgSO3::new(m)
        } else if beta == A::constant(PI) {
            let r11 = A::constant(PI) * A::sqrt(A::constant(0.5) * (self.0[(0, 0)] + A::constant(1.0)));
            let r22 = A::constant(PI) * A::sqrt(A::constant(0.5) * (self.0[(1, 1)] + A::constant(1.0)));
            let r33 = A::constant(PI) * A::sqrt(A::constant(0.5) * (self.0[(2, 2)] + A::constant(1.0)));
            let m = M3::from_row_slice(
                &[A::zero(), -r33, r22,
                    r33, A::zero(), -r11,
                    -r22, r11, A::zero()]
            );
            LieAlgSO3::new(m)
        } else {
            let f = beta / (A::constant(2.0) * beta.sin());
            let m = (self.0.matrix() - self.0.matrix().transpose()).scale(f);
            LieAlgSO3::new(m)
        }
    }
}

/// Implements the `Rotation3DLieGroupElement` trait for `LieGroupSO3`.
impl<A: AD> Rotation3DLieGroupElement<A> for LieGroupSO3<A> {}
impl<A: AD> LieAlgebraElement<A> for LieAlgSO3<A> {
    type LieGroupElementType = LieGroupSO3<A>;
    type EuclideanSpaceElementType = V3<A>;

    /// Converts a Euclidean space element (vector) to a `LieAlgSO3` element.
    ///
    /// - `e`: The vector representing the element in Euclidean space.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_so3()
    }

    /// Scales the `LieAlgSO3` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    fn scale(&self, scalar: A) -> Self {
        Self::new(self.0 * scalar)
    }

    /// Exponentiates the `LieAlgSO3` element to produce the corresponding Lie group element.
    #[inline]
    fn exp(&self) -> Self::LieGroupElementType {
        let a = self.0.m32;
        let b = self.0.m13;
        let c = self.0.m21;

        let u = V3::new(a, b, c);

        let beta = u.norm();

        let (p, q) = if beta < A::constant(0.0001) {
            let pp = A::constant(1.0) - (beta.powi(2) / A::constant(6.0)) + (beta.powi(4) / A::constant(120.0));
            let qq = A::constant(0.5) - (beta.powi(2) / A::constant(24.0)) + (beta.powi(4) / A::constant(720.0));
            (pp, qq)
        } else {
            (beta.sin() / beta, (A::constant(1.0) - beta.cos()) / beta.powi(2))
        };

        let m = M3::identity() + self.0 * p + (self.0.pow(2)) * q;

        return LieGroupSO3::new(R3::from_matrix_unchecked(m));
    }

    /// Converts the `LieAlgSO3` element to its corresponding Euclidean space element (a vector).
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let a = self.0.m32;
        let b = self.0.m13;
        let c = self.0.m21;

        V3::new(a, b, c)
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Trait for converting a 3D matrix (`M3`) to a `LieAlgSO3` element.
pub trait ApolloMatrix3SO3LieTrait<A: AD> {
    /// Converts the matrix `M3` to a `LieAlgSO3` element.
    fn to_lie_alg_so3(&self) -> LieAlgSO3<A>;
}
impl<A: AD> ApolloMatrix3SO3LieTrait<A> for M3<A> {
    fn to_lie_alg_so3(&self) -> LieAlgSO3<A> {
        LieAlgSO3::new(*self)
    }
}

/// Trait for converting a 3D rotation matrix (`R3`) to a `LieGroupSO3` element.
pub trait ApolloRotation3SO3LieTrait<A: AD> {
    /// Converts the rotation matrix `R3` to a `LieGroupSO3` element.
    fn to_lie_group_so3(&self) -> LieGroupSO3<A>;
}
impl<A: AD> ApolloRotation3SO3LieTrait<A> for R3<A> {
    #[inline(always)]
    fn to_lie_group_so3(&self) -> LieGroupSO3<A> {
        LieGroupSO3::new(*self)
    }
}

/// Trait for converting a vector (`V3`) to a `LieAlgSO3` element.
pub trait ApolloLieAlgPackSO3Trait<A: AD> {
    /// Converts the vector to a `LieAlgSO3` element.
    fn to_lie_alg_so3(&self) -> LieAlgSO3<A>;
}
impl<A: AD> ApolloLieAlgPackSO3Trait<A> for V3<A> {
    fn to_lie_alg_so3(&self) -> LieAlgSO3<A> {
        let a = self[0];
        let b = self[1];
        let c = self[2];

        LieAlgSO3::new(M3::new(A::zero(), -c, b, c, A::zero(), -a, -b, a, A::zero()))
    }
}
