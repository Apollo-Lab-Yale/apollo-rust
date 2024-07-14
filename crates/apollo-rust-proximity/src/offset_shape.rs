use std::borrow::Cow;
use parry3d_f64::na::UnitQuaternion;
use parry3d_f64::shape::{Ball, Cuboid, Shape};
use apollo_rust_spatial::isometry3::I3;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::ApolloVector3Trait2;
use apollo_rust_lie::LieGroupElement;

pub struct OffsetShape {
    shape: Box<dyn Shape>,
    offset: Option<ISE3q>,
}
impl OffsetShape {
    pub fn new<S: Shape>(shape: S, offset: Option<ISE3q>) -> Self {
        Self { shape: Box::new(shape), offset }
    }

    #[inline(always)]
    pub fn shape(&self) -> &Box<dyn Shape> {
        &self.shape
    }

    #[inline(always)]
    pub fn offset(&self) -> &Option<ISE3q> {
        &self.offset
    }

    #[inline(always)]
    pub fn get_transform<'a>(&'a self, variable_transform: &'a ISE3q) -> Cow<ISE3q> {
        return match &self.offset {
            None => {
                Cow::Borrowed(variable_transform)
            }
            Some(offset) => {
                Cow::Owned( offset.group_operator(variable_transform) )
            }
        }
    }
}
impl Clone for OffsetShape {
    #[inline(always)]
    fn clone(&self) -> Self {
        Self {
            shape: self.shape.clone_box(),
            offset: self.offset.clone(),
        }
    }
}

pub fn to_offset_shape_bounding_sphere<S: Shape>(shape: &S) -> OffsetShape {
    let bounding_sphere = shape.compute_local_bounding_sphere();
    let radius = bounding_sphere.radius;
    let center = bounding_sphere.center().coords.xyz();
    let ball = Ball::new(radius);
    let offset = ISE3q::new(I3::from_parts(center.to_translation(), UnitQuaternion::identity()));

    OffsetShape::new(ball, Some(offset))
}

pub fn to_offset_shape_obb<S: Shape>(shape: &S) -> OffsetShape {
    let obb = shape.compute_local_aabb();
    let extents = obb.extents().xyz();
    let half_extents = 0.5*extents;
    let center = obb.center().coords.xyz();
    let cuboid = Cuboid::new(half_extents);
    let offset = ISE3q::new(I3::from_parts(center.to_translation(), UnitQuaternion::identity()));

    OffsetShape::new(cuboid, Some(offset))
}

