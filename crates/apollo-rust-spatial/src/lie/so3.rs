use std::f64::consts::PI;
use serde::{Deserialize, Serialize};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use crate::lie::Rotation3DLieGroupElement;
use crate::matrices::{M3};
use crate::rotation_matrices::{R3};
use crate::vectors::V3;

/// Struct representing a Lie group element in SO(3) rotation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieGroupSO3(pub R3);

impl LieGroupSO3 {
    /// Creates a new `LieGroupSO3` instance.
    ///
    /// - `field0`: The 3D rotation matrix `R3` representing the Lie group element.
    #[inline(always)]
    pub fn new(field0: R3) -> Self {
        Self(field0)
    }

    /// Constructs a `LieGroupSO3` element from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V3) -> Self {
        exponential_coordinates.to_lie_alg_so3().exp()
    }
}

/// Struct representing a Lie algebra element in so(3).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgSO3(pub M3);

impl LieAlgSO3 {
    /// Creates a new `LieAlgSO3` instance.
    ///
    /// - `field0`: The matrix `M3` representing the Lie algebra element.
    ///   Must satisfy skew-symmetric matrix conditions.
    #[inline(always)]
    pub fn new(field0: M3) -> Self {
        assert_eq!(field0.m11, 0.0);
        assert_eq!(field0.m22, 0.0);
        assert_eq!(field0.m33, 0.0);

        assert_eq!(field0.m12, -field0.m21, "{}", format!("{}, {}", field0.m12, -field0.m21));
        assert_eq!(field0.m13, -field0.m31, "{}", format!("{}, {}", field0.m13, -field0.m31));
        assert_eq!(field0.m23, -field0.m32, "{}", format!("{}, {}", field0.m23, -field0.m32));

        Self(field0)
    }
}

impl LieGroupElement for LieGroupSO3 {
    type LieAlgebraElementType = LieAlgSO3;

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
        let beta = f64::acos((tr - 1.0) / 2.0);

        return if beta.abs() < 0.00001 {
            let f = 0.5 + (beta.powi(2) / 12.0) + (7.0 * beta.powi(4) / 720.0);
            let m = (self.0.matrix() - self.0.matrix().transpose()).scale(f);
            LieAlgSO3::new(m)
        } else if beta == PI {
            let r11 = PI * f64::sqrt(0.5 * (self.0[(0, 0)] + 1.0));
            let r22 = PI * f64::sqrt(0.5 * (self.0[(1, 1)] + 1.0));
            let r33 = PI * f64::sqrt(0.5 * (self.0[(2, 2)] + 1.0));
            let m = M3::from_row_slice(
                &[0.0, -r33, r22,
                    r33, 0.0, -r11,
                    -r22, r11, 0.0]
            );
            LieAlgSO3::new(m)
        } else {
            let f = beta / (2.0 * beta.sin());
            let m = (self.0.matrix() - self.0.matrix().transpose()).scale(f);
            LieAlgSO3::new(m)
        }
    }
}

/// Implements the `Rotation3DLieGroupElement` trait for `LieGroupSO3`.
impl Rotation3DLieGroupElement for LieGroupSO3 {}

impl LieAlgebraElement for LieAlgSO3 {
    type LieGroupElementType = LieGroupSO3;
    type EuclideanSpaceElementType = V3;

    /// Converts a Euclidean space element (vector) to a `LieAlgSO3` element.
    ///
    /// - `e`: The vector representing the element in Euclidean space.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_so3()
    }

    /// Scales the `LieAlgSO3` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    fn scale(&self, scalar: f64) -> Self {
        Self::new(scalar * self.0)
    }

    /// Exponentiates the `LieAlgSO3` element to produce the corresponding Lie group element.
    #[inline]
    fn exp(&self) -> Self::LieGroupElementType {
        let a = self.0.m32;
        let b = self.0.m13;
        let c = self.0.m21;

        let u = V3::new(a, b, c);

        let beta = u.norm();

        let (p, q) = if beta < 0.0001 {
            let pp = 1.0 - (beta.powi(2) / 6.0) + (beta.powi(4) / 120.0);
            let qq = 0.5 - (beta.powi(2) / 24.0) + (beta.powi(4) / 720.0);
            (pp, qq)
        } else {
            (beta.sin() / beta, (1.0 - beta.cos()) / beta.powi(2))
        };

        let m = M3::identity() + p * self.0 + q * (self.0.pow(2));

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
pub trait ApolloMatrix3SO3LieTrait {
    /// Converts the matrix `M3` to a `LieAlgSO3` element.
    fn to_lie_alg_so3(&self) -> LieAlgSO3;
}

impl ApolloMatrix3SO3LieTrait for M3 {
    fn to_lie_alg_so3(&self) -> LieAlgSO3 {
        LieAlgSO3::new(*self)
    }
}

/// Trait for converting a 3D rotation matrix (`R3`) to a `LieGroupSO3` element.
pub trait ApolloRotation3SO3LieTrait {
    /// Converts the rotation matrix `R3` to a `LieGroupSO3` element.
    fn to_lie_group_so3(&self) -> LieGroupSO3;
}

impl ApolloRotation3SO3LieTrait for R3 {
    #[inline(always)]
    fn to_lie_group_so3(&self) -> LieGroupSO3 {
        LieGroupSO3::new(*self)
    }
}

/// Trait for converting a vector (`V3`) to a `LieAlgSO3` element.
pub trait ApolloLieAlgPackSO3Trait {
    /// Converts the vector to a `LieAlgSO3` element.
    fn to_lie_alg_so3(&self) -> LieAlgSO3;
}

impl ApolloLieAlgPackSO3Trait for V3 {
    fn to_lie_alg_so3(&self) -> LieAlgSO3 {
        let a = self[0];
        let b = self[1];
        let c = self[2];

        LieAlgSO3::new(M3::new(0.0, -c, b, c, 0.0, -a, -b, a, 0.0))
    }
}
