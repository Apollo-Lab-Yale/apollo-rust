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
}

pub trait LieAlgebraElement {
    type LieGroupElementType: LieGroupElement<LieAlgebraElementType= Self>;
    type EuclideanSpaceElementType : EuclideanSpaceElement;

    fn exp(&self) -> Self::LieGroupElementType;
    fn vee(&self) -> Self::EuclideanSpaceElementType;
}

pub trait EuclideanSpaceElement {
    fn norm(&self) -> f64;
}

impl EuclideanSpaceElement for f64 {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.abs()
    }
}
impl EuclideanSpaceElement for Vector2<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm()
    }
}
impl EuclideanSpaceElement for Vector3<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm()
    }
}
impl EuclideanSpaceElement for Vector6<f64> {
    #[inline(always)]
    fn norm(&self) -> f64 {
        self.norm()
    }
}