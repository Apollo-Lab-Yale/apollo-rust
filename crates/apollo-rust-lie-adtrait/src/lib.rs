use ad_trait::AD;
use nalgebra::{Vector2, Vector3, Vector6};

/// Trait representing an element of a Lie group.
pub trait LieGroupElement<A: AD>: Sized {
    /// Associated type representing the corresponding Lie algebra element for this Lie group.
    type LieAlgebraElementType: LieAlgebraElement<A, LieGroupElementType = Self>;

    /// Group operation for combining two elements of the Lie group.
    fn group_operator(&self, other: &Self) -> Self;

    /// Returns the identity element of the Lie group.
    fn identity_element() -> Self;

    /// Returns the inverse of the Lie group element.
    fn inverse(&self) -> Self;

    /// Computes the displacement (group difference) between two Lie group elements.
    #[inline(always)]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse().group_operator(other)
    }

    /// Computes the distance between two Lie group elements based on their displacement.
    #[inline(always)]
    fn displacement_based_distance(&self, other: &Self) -> A {
        let disp = self.displacement(other);
        let ln = disp.ln();
        let vee = ln.vee();
        return vee.norm();
    }

    /// Computes the logarithm map of the Lie group element, returning the corresponding Lie algebra element.
    fn ln(&self) -> Self::LieAlgebraElementType;

    /// Interpolates between two Lie group elements using a parameter `t` in the range `[0, 1]`.
    #[inline(always)]
    fn interpolate(&self, other: &Self, t: A) -> Self {
        let disp = self.displacement(other);
        let ln = disp.ln();
        let scaled = ln.scale(t);
        let exp = scaled.exp();
        return self.group_operator(&exp);
    }
}

/// Trait representing an element of a Lie algebra.
pub trait LieAlgebraElement<A: AD> {
    /// Associated type representing the corresponding Lie group element for this Lie algebra.
    type LieGroupElementType: LieGroupElement<A>;

    /// Associated type representing the corresponding Euclidean space element for this Lie algebra.
    type EuclideanSpaceElementType: EuclideanSpaceElement<A>;

    /// Constructs a Lie algebra element from a Euclidean space element.
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self;

    /// Scales the Lie algebra element by a scalar.
    fn scale(&self, scalar: A) -> Self;

    /// Computes the exponential map to a Lie group element.
    fn exp(&self) -> Self::LieGroupElementType;

    /// Maps the Lie algebra element to a Euclidean space element (vee operator).
    fn vee(&self) -> Self::EuclideanSpaceElementType;
}

/// Trait representing an element in Euclidean space.
pub trait EuclideanSpaceElement<A: AD> {
    /// Computes the norm (magnitude) of the Euclidean space element.
    fn norm(&self) -> A;

    /// Scales the Euclidean space element by a scalar.
    fn scale(&self, scalar: A) -> Self;
}

/////

impl<A: AD> EuclideanSpaceElement<A> for A {
    #[inline(always)]
    fn norm(&self) -> A {
        self.abs()
    }

    fn scale(&self, scalar: A) -> Self {
        *self * scalar
    }
}

/// Implementation of `EuclideanSpaceElement` for 2D vectors (`Vector2<f64>`).
impl<A: AD> EuclideanSpaceElement<A> for Vector2<A> {
    #[inline(always)]
    fn norm(&self) -> A {
        self.norm()
    }

    fn scale(&self, scalar: A) -> Self {
        *self * scalar
    }
}

/// Implementation of `EuclideanSpaceElement` for 3D vectors (`Vector3<f64>`).
impl<A: AD> EuclideanSpaceElement<A> for Vector3<A> {
    #[inline(always)]
    fn norm(&self) -> A {
        self.norm()
    }

    fn scale(&self, scalar: A) -> Self {
        *self * scalar
    }
}

/// Implementation of `EuclideanSpaceElement` for 6D vectors (`Vector6<f64>`).
impl<A: AD> EuclideanSpaceElement<A> for Vector6<A> {
    #[inline(always)]
    fn norm(&self) -> A {
        self.norm()
    }

    fn scale(&self, scalar: A) -> Self {
        *self * scalar
    }
}
