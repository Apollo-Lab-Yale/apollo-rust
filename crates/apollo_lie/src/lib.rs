use nalgebra::{Vector2, Vector3, Vector6};

pub trait LieGroupElement : Sized {
    type LieAlgebraElementType: LieAlgebraElement<LieGroupElementType= Self>;

    fn group_operator(&self, other: &Self) -> Self;
    fn inverse(&self) -> Self;
    #[inline(always)]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse().group_operator(other)
    }
    #[inline(always)]
    fn displacement_based_distance(&self, other: &Self) -> f64 {
        let disp = self.displacement(other);
        let ln = disp.ln();
        let vee = ln.vee();
        return vee.norm()
    }
    fn ln(&self) -> Self::LieAlgebraElementType;
    #[inline(always)]
    fn interpolate(&self, other: &Self, t: f64) -> Self {
        let disp = self.displacement(other);
        let ln = disp.ln();
        let scaled = ln.scale(t);
        let exp = scaled.exp();
        return self.group_operator(&exp);
    }
}

pub trait LieAlgebraElement {
    type LieGroupElementType: LieGroupElement<LieAlgebraElementType= Self>;
    type EuclideanSpaceElementType : EuclideanSpaceElement;

    fn from_euclidean_space_element(e: &Self::EuclideanSpaceElementType) -> Self;
    fn scale(&self, scalar: f64) -> Self;
    fn exp(&self) -> Self::LieGroupElementType;
    fn vee(&self) -> Self::EuclideanSpaceElementType;
}

pub trait EuclideanSpaceElement {
    fn norm(&self) -> f64;
    fn scale(&self, scalar: f64) -> Self;
}

impl EuclideanSpaceElement for f64 {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.abs()
    }
    fn scale(&self, scalar: f64) -> Self {
        *self * scalar
    }
}
impl EuclideanSpaceElement for Vector2<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm()
    }

    fn scale(&self, scalar: f64) -> Self {
        scalar * *self
    }
}
impl EuclideanSpaceElement for Vector3<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm()
    }

    fn scale(&self, scalar: f64) -> Self {
        scalar * *self
    }
}
impl EuclideanSpaceElement for Vector6<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm()
    }

    fn scale(&self, scalar: f64) -> Self {
        scalar * *self
    }
}