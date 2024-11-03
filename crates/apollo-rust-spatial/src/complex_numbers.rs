use nalgebra::{Complex, UnitComplex};

pub type C = Complex<f64>;
pub type UC = UnitComplex<f64>;

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Alias for `Complex<A>`, representing a complex number with AD-compatible elements.
pub type CAD<A> = Complex<A>;

/// Alias for `UnitComplex<A>`, representing a unit complex number with AD-compatible elements.
pub type UCAD<A> = UnitComplex<A>;
