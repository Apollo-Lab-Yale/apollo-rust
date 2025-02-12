#![allow(dead_code)]

use ad_trait::AD;
use apollo_rust_lie_adtrait::LieGroupElement;

pub mod u1;
mod so2;

pub trait Rotation2DLieGroupElement<A: AD>: LieGroupElement<A> { }
pub trait Rotation3DLieGroupElement<A: AD>: LieGroupElement<A> { }
pub trait TranslationAndRotation2DLieGroupElement<A: AD>: LieGroupElement<A> { }
pub trait TranslationAndRotation3DLieGroupElement<A: AD>: LieGroupElement<A> { }