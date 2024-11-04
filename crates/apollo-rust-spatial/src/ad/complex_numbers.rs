use nalgebra::{Complex, UnitComplex};

/// Alias for `Complex<A>`, representing a complex number with AD-compatible elements.
pub type CAD<A> = Complex<A>;

/// Alias for `UnitComplex<A>`, representing a unit complex number with AD-compatible elements.
pub type UCAD<A> = UnitComplex<A>;
