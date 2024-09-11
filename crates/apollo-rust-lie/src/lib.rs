use nalgebra::{Vector2, Vector3, Vector6};

// Trait representing an element of a Lie group
pub trait LieGroupElement: Sized {
    // Associated type representing the corresponding Lie algebra element for this Lie group
    type LieAlgebraElementType: LieAlgebraElement<LieGroupElementType= Self>;

    // Group operation for combining two elements of the Lie group
    fn group_operator(&self, other: &Self) -> Self;

    // Returns the identity element of the Lie group
    fn identity_element() -> Self;

    // Returns the inverse of the Lie group element
    fn inverse(&self) -> Self;

    // Computes the displacement (group difference) between two Lie group elements
    #[inline(always)]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse().group_operator(other)
    }

    // Computes the distance between two Lie group elements based on their displacement
    #[inline(always)]
    fn displacement_based_distance(&self, other: &Self) -> f64 {
        let disp = self.displacement(other); // Compute displacement
        let ln = disp.ln(); // Compute the logarithm map of the displacement
        let vee = ln.vee(); // Map to the Lie algebra
        return vee.norm() // Compute the norm (magnitude) of the Lie algebra element
    }

    // Computes the logarithm map of the Lie group element
    fn ln(&self) -> Self::LieAlgebraElementType;

    // Interpolates between two Lie group elements using a parameter t in [0, 1]
    #[inline(always)]
    fn interpolate(&self, other: &Self, t: f64) -> Self {
        let disp = self.displacement(other); // Compute displacement
        let ln = disp.ln(); // Compute the logarithm map of the displacement
        let scaled = ln.scale(t); // Scale the Lie algebra element by t
        let exp = scaled.exp(); // Exponentiate to get a Lie group element
        return self.group_operator(&exp); // Apply group operation to interpolate
    }
}

// Trait representing an element of a Lie algebra
pub trait LieAlgebraElement {
    type LieGroupElementType: LieGroupElement;
    type EuclideanSpaceElementType: EuclideanSpaceElement;

    // Constructs a Lie algebra element from a Euclidean space element
    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self;

    // Scales the Lie algebra element by a scalar
    fn scale(&self, scalar: f64) -> Self;

    // Computes the exponential map to a Lie group element
    fn exp(&self) -> Self::LieGroupElementType;

    // Maps the Lie algebra element to a Euclidean space element
    fn vee(&self) -> Self::EuclideanSpaceElementType;
}

// Trait representing an element in Euclidean space
pub trait EuclideanSpaceElement {
    // Computes the norm (magnitude) of the Euclidean space element
    fn norm(&self) -> f64;

    // Scales the Euclidean space element by a scalar
    fn scale(&self, scalar: f64) -> Self;
}

// Implementation of EuclideanSpaceElement for f64
impl EuclideanSpaceElement for f64 {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.abs() // Norm is the absolute value for scalars
    }

    fn scale(&self, scalar: f64) -> Self {
        *self * scalar // Scalar multiplication for f64
    }
}

// Implementation of EuclideanSpaceElement for 2D vectors
impl EuclideanSpaceElement for Vector2<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm() // Use nalgebra's built-in norm computation
    }

    fn scale(&self, scalar: f64) -> Self {
        scalar * *self // Scalar multiplication for 2D vectors
    }
}

// Implementation of EuclideanSpaceElement for 3D vectors
impl EuclideanSpaceElement for Vector3<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm() // Use nalgebra's built-in norm computation
    }

    fn scale(&self, scalar: f64) -> Self {
        scalar * *self // Scalar multiplication for 3D vectors
    }
}

// Implementation of EuclideanSpaceElement for 6D vectors
impl EuclideanSpaceElement for Vector6<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm() // Use nalgebra's built-in norm computation
    }

    fn scale(&self, scalar: f64) -> Self {
        scalar * *self // Scalar multiplication for 6D vectors
    }
}
