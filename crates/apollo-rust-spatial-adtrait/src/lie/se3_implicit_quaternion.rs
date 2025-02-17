use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use crate::isometry3::{ApolloIsometry3Trait, I3};
use crate::lie::h1::{ApolloLieAlgPackH1Trait, ApolloQuaternionH1LieTrait, ApolloUnitQuaternionH1LieTrait};
use crate::lie::so3::ApolloLieAlgPackSO3Trait;
use crate::lie::TranslationAndRotation3DLieGroupElement;
use crate::matrices::M3;
use crate::quaternions::{ApolloQuaternionTrait, Q};
use crate::translations::ApolloTranslation3AD;
use crate::vectors::{ApolloVector3ADTrait, ApolloVector3Trait2, V3, V6};

/// Alias for `LieGroupISE3q`, representing a Lie group in SE(3) with quaternion rotation.
pub type ISE3q<A> = LieGroupISE3q<A>;

/// Alias for `LieAlgISE3q`, representing a Lie algebra in se(3) with quaternion rotation.
pub type Ise3q<A> = LieAlgISE3q<A>;

/// Struct representing a Lie group in SE(3) with quaternion rotation (`I3` for translation and rotation).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
pub struct LieGroupISE3q<A: AD>(#[serde(deserialize_with = "I3::<A>::deserialize")] pub I3<A>);
impl<A: AD> LieGroupISE3q<A> {
    /// Creates a new `LieGroupISE3q` instance.
    ///
    /// - `field0`: The isometry (`I3`) representing the Lie group element.
    #[inline(always)]
    pub fn new(field0: I3<A>) -> Self {
        Self(field0)
    }

    /// Returns the identity element of the Lie group `ISE3q`.
    #[inline(always)]
    pub fn identity() -> Self {
        Self::new(I3::identity())
    }

    /// Creates a new random instance of `LieGroupISE3q` within the range `[-3.0, 3.0]`.
    pub fn new_random() -> Self {
        Self::new(I3::new_random_with_range(-3.0, 3.0))
    }

    /// Constructs a `LieGroupISE3q` element from exponential coordinates.
    ///
    /// - `exponential_coordinates`: The vector representing the exponential coordinates.
    #[inline(always)]
    pub fn from_exponential_coordinates(exponential_coordinates: &V6<A>) -> Self {
        return exponential_coordinates.to_lie_alg_ise3q().exp();
    }

    /// Maps a point using the Lie group transformation.
    ///
    /// - `point`: The point to be transformed.
    #[inline(always)]
    pub fn map_point(&self, point: &V3<A>) -> V3<A> {
        self.0.rotation * point + self.0.translation.vector
    }

    /// Returns the pseudo-logarithm of the Lie group `ISE3q`.
    #[inline(always)]
    pub fn pseudo_ln(&self) -> PseudoLieAlgISE3q<A> {
        let q = self.0.rotation.to_lie_group_h1().ln();
        PseudoLieAlgISE3q::new(q.0, self.0.translation.to_vector3())
    }

    /// Adds a small amount of noise to the `LieGroupISE3q` element.
    #[inline(always)]
    pub fn add_tiny_bit_of_noise(&self) -> Self {
        Self::new(self.0.add_tiny_bit_of_noise())
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieGroupISE3q<A2> {
        LieGroupISE3q(self.0.to_other_ad_type::<A2>())
    }

    pub fn to_constant_ad(&self) -> Self {
        Self::new(self.0.to_constant_ad())
    }
}

/// Struct representing a Lie algebra in se(3) with quaternion rotation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LieAlgISE3q<A: AD> {
    #[serde(deserialize_with = "Q::<A>::deserialize")]
    q: Q<A>,
    #[serde(deserialize_with = "V3::<A>::deserialize")]
    v: V3<A>,
}
impl<A: AD> LieAlgISE3q<A> {
    /// Creates a new `LieAlgISE3q` instance.
    ///
    /// - `q`: The quaternion `Q` representing rotation.
    /// - `v`: The vector `V3` representing translation.
    pub fn new(q: Q<A>, v: V3<A>) -> Self {
        Self { q, v }
    }

    /// Returns a reference to the quaternion representing rotation.
    #[inline(always)]
    pub fn q(&self) -> &Q<A> {
        &self.q
    }

    /// Returns a reference to the vector representing translation.
    #[inline(always)]
    pub fn v(&self) -> &V3<A> {
        &self.v
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> LieAlgISE3q<A2> {
        LieAlgISE3q {
            q: self.q.to_other_ad_type::<A2>(),
            v: self.v.to_other_ad_type::<A2>(),
        }
    }

    pub fn to_constant_ad(&self) -> Self {
        Self {
            q: self.q.to_constant_ad(),
            v: self.v.to_constant_ad(),
        }
    }
}
impl<A: AD> LieGroupElement<A> for LieGroupISE3q<A> {
    type LieAlgebraElementType = LieAlgISE3q<A>;

    /// Implements the group operator for the `ISE3q` Lie group element.
    ///
    /// - `other`: Another `LieGroupISE3q` element to multiply with.
    #[inline(always)]
    fn group_operator(&self, other: &Self) -> Self {
        Self::new(self.0 * other.0)
    }

    /// Returns the identity element of the Lie group `ISE3q`.
    #[inline(always)]
    fn identity_element() -> Self {
        Self::new(I3::identity())
    }

    /// Returns the inverse of the current Lie group element.
    #[inline(always)]
    fn inverse(&self) -> Self {
        Self::new(self.0.inverse())
    }

    /// Returns the logarithm (Lie algebra element) of the current Lie group element.
    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraElementType {
        let a_quat = self.0.rotation.to_lie_group_h1().ln();
        let u = a_quat.0.imag();
        let a_mat = u.to_lie_alg_so3();
        let beta = u.norm();

        let (p, q) = if beta.abs() < A::constant(0.0001) {
            let pp = A::constant(0.5) - (beta.powi(2) / A::constant(24.0)) + (beta.powi(4) / A::constant(720.0));
            let qq = A::constant(1.0 / 6.0) - (beta.powi(2) / A::constant(120.0)) + (beta.powi(4) / A::constant(5040.0));
            (pp, qq)
        } else {
            (
                (A::constant(1.0) - A::cos(beta)) / A::powi(beta, 2),
                (beta - A::sin(beta)) / A::powi(beta, 3),
            )
        };

        let c_mat = M3::identity() + a_mat.0 * p + a_mat.0.pow(2) * q;
        let c_inv = c_mat.try_inverse().expect("error");

        let b = c_inv * self.0.translation.vector;

        Ise3q::new(a_quat.0, b)
    }
}

/// Implements the `TranslationAndRotation3DLieGroupElement` trait for `LieGroupISE3q`.
impl<A: AD> TranslationAndRotation3DLieGroupElement<A> for LieGroupISE3q<A> {}
impl<A: AD> LieAlgebraElement<A> for LieAlgISE3q<A> {
    type LieGroupElementType = LieGroupISE3q<A>;
    type EuclideanSpaceElementType = V6<A>;

    /// Converts a Euclidean space element (vector) to a `LieAlgISE3q` element.
    ///
    /// - `e`: The Euclidean space vector representing the element.
    #[inline(always)]
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_lie_alg_ise3q()
    }

    /// Scales the `LieAlgISE3q` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    #[inline(always)]
    fn scale(&self, scalar: A) -> Self {
        return Self::new(self.q * scalar, self.v.scale(scalar))
    }

    /// Exponentiates the `LieAlgISE3q` element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let u = self.q.to_lie_alg_h1().vee();
        let a_mat = u.to_lie_alg_so3();
        let beta = u.norm();

        let (p, q) = if beta.abs() < A::constant(0.0001) {
            let pp = A::constant(0.5) - (beta.powi(2) / A::constant(24.0)) + (beta.powi(4) / A::constant(720.0));
            let qq = A::constant(1.0 / 6.0) - (beta.powi(2) / A::constant(120.0)) + (beta.powi(4) / A::constant(5040.0));
            (pp, qq)
        } else {
            (
                (A::constant(1.0) - A::cos(beta)) / A::powi(beta, 2),
                (beta - A::sin(beta)) / A::powi(beta, 3),
            )
        };

        let c_mat = M3::identity() + a_mat.0 * p + a_mat.0.pow(2) * q;
        let t = c_mat * self.v;
        let q = self.q.to_lie_alg_h1().exp();

        ISE3q::new(I3::from_parts(t.to_translation(), q.0))
    }

    /// Converts the `LieAlgISE3q` element to its corresponding Euclidean space vector.
    #[inline(always)]
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let u = self.q.to_lie_alg_h1().vee();
        let v = &self.v;

        V6::new(u[0], u[1], u[2], v[0], v[1], v[2])
    }
}

/// Trait for converting vectors to `LieAlgISE3q`.
pub trait ApolloLieAlgPackIse3qTrait<A: AD> {
    /// Converts a vector to a `LieAlgISE3q` element.
    fn to_lie_alg_ise3q(&self) -> LieAlgISE3q<A>;
}
impl<A: AD> ApolloLieAlgPackIse3qTrait<A> for V6<A> {
    #[inline(always)]
    fn to_lie_alg_ise3q(&self) -> LieAlgISE3q<A> {
        let u = V3::new(self[0], self[1], self[2]);
        let q = u.to_lie_alg_h1();
        let v = V3::new(self[3], self[4], self[5]);

        LieAlgISE3q::new(q.0, v)
    }
}

/*
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
*/

/// Struct representing a pseudo-Lie algebra in se(3) with quaternion rotation.
#[derive(Clone, Debug)]
pub struct PseudoLieAlgISE3q<A: AD> {
    q: Q<A>,
    v: V3<A>,
}
impl<A: AD> PseudoLieAlgISE3q<A> {
    /// Creates a new `PseudoLieAlgISE3q` instance.
    ///
    /// - `q`: The quaternion `Q` representing rotation.
    /// - `v`: The vector `V3` representing translation.
    pub fn new(q: Q<A>, v: V3<A>) -> Self {
        Self { q, v }
    }

    /// Returns a reference to the quaternion representing rotation.
    #[inline(always)]
    pub fn q(&self) -> &Q<A> {
        &self.q
    }

    /// Returns a reference to the vector representing translation.
    #[inline(always)]
    pub fn v(&self) -> &V3<A> {
        &self.v
    }
}

/*
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
*/

impl<A: AD> LieAlgebraElement<A> for PseudoLieAlgISE3q<A> {
    type LieGroupElementType = LieGroupISE3q<A>;
    type EuclideanSpaceElementType = V6<A>;

    /// Converts a Euclidean space element (vector) to a `PseudoLieAlgISE3q` element.
    ///
    /// - `e`: The Euclidean space vector representing the element.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self {
        e.to_pseudo_lie_alg_ise3q()
    }

    /// Scales the `PseudoLieAlgISE3q` element by a scalar value.
    ///
    /// - `scalar`: The scaling factor.
    #[inline(always)]
    fn scale(&self, scalar: A) -> Self {
        return Self::new(self.q * scalar, self.v.scale(scalar))
    }

    /// Exponentiates the `PseudoLieAlgISE3q` element to produce the corresponding Lie group element.
    #[inline(always)]
    fn exp(&self) -> Self::LieGroupElementType {
        let q = self.q.to_lie_alg_h1().exp();
        LieGroupISE3q::new(I3::from_parts(self.v.to_translation(), q.0))
    }

    /// Converts the `PseudoLieAlgISE3q` element to its corresponding Euclidean space vector.
    fn vee(&self) -> Self::EuclideanSpaceElementType {
        let u = self.q.to_lie_alg_h1().vee();
        let v = &self.v;

        V6::new(u[0], u[1], u[2], v[0], v[1], v[2])
    }
}

/// Trait for converting vectors to `PseudoLieAlgISE3q`.
pub trait ApolloPseudoLieAlgPackIse3qTrait<A: AD> {
    /// Converts a vector to a `PseudoLieAlgISE3q` element.
    fn to_pseudo_lie_alg_ise3q(&self) -> PseudoLieAlgISE3q<A>;
}
impl<A: AD> ApolloPseudoLieAlgPackIse3qTrait<A> for V6<A> {
    #[inline(always)]
    fn to_pseudo_lie_alg_ise3q(&self) -> PseudoLieAlgISE3q<A> {
        let u = V3::new(self[0], self[1], self[2]);
        let q = u.to_lie_alg_h1();
        let v = V3::new(self[3], self[4], self[5]);

        PseudoLieAlgISE3q::new(q.0, v)
    }
}
