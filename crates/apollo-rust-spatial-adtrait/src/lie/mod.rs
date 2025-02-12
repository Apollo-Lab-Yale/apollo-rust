use ad_trait::AD;
use apollo_rust_lie_adtrait::LieGroupElement;

pub mod u1;
pub mod so2;
pub mod h1;
pub mod so3;
pub mod se3_implicit;
pub mod se3_implicit_quaternion;

pub trait Rotation2DLieGroupElement<A: AD>: LieGroupElement<A> { }
pub trait Rotation3DLieGroupElement<A: AD>: LieGroupElement<A> { }
pub trait TranslationAndRotation2DLieGroupElement<A: AD>: LieGroupElement<A> { }
pub trait TranslationAndRotation3DLieGroupElement<A: AD>: LieGroupElement<A> { }