use nalgebra::{Complex, UnitComplex};

/// Alias for `Complex<A>`, representing a complex number with AD-compatible elements.
pub type C<A> = Complex<A>;

/// Alias for `UnitComplex<A>`, representing a unit complex number with AD-compatible elements.
pub type UC<A> = UnitComplex<A>;
