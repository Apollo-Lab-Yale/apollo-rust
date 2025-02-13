use ad_trait::AD;
use nalgebra::{Complex, UnitComplex};

/// Alias for `Complex<A>`, representing a complex number with AD-compatible elements.
pub type C<A> = Complex<A>;

/// Alias for `UnitComplex<A>`, representing a unit complex number with AD-compatible elements.
pub type UC<A> = UnitComplex<A>;


pub trait ApolloComplexTrait<A: AD> {
    fn to_other_ad_type<A2: AD>(&self) -> C<A2>;
    fn to_constant_ad(&self) -> C<A>;
}
impl<A: AD> ApolloComplexTrait<A> for C<A> {
    fn to_other_ad_type<A2: AD>(&self) -> C<A2> {
        C::new(self.re.to_other_ad_type::<A2>(), self.im.to_other_ad_type::<A2>())
    }

    fn to_constant_ad(&self) -> C<A> {
        C::new(self.re.to_constant_ad(), self.im.to_constant_ad())
    }
}


pub trait ApolloUnitComplexTrait<A: AD> {
    fn to_other_ad_type<A2: AD>(&self) -> UC<A2>;
    fn to_constant_ad(&self) -> UC<A>;
}
impl<A: AD> ApolloUnitComplexTrait<A> for UC<A> {
    fn to_other_ad_type<A2: AD>(&self) -> UC<A2> {
        let angle = self.angle();
        UC::new(angle.to_other_ad_type::<A2>())
    }

    fn to_constant_ad(&self) -> UC<A> {
        let angle = self.angle();
        UC::new(angle.to_constant_ad())
    }
}