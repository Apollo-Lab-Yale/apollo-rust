use apollo_lie::LieGroupElement;

pub mod u1;
pub mod so2;
pub mod so3;
pub mod se3_implicit_quaternion;
pub mod se3_implicit;
pub mod h1;


pub trait Rotation2DLieGroupElement: LieGroupElement { }
pub trait Rotation3DLieGroupElement: LieGroupElement { }
pub trait TranslationAndRotation2DLieGroupElement: LieGroupElement { }
pub trait TranslationAndRotation3DLieGroupElement: LieGroupElement { }