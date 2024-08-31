use nalgebra::Point3;
use parry3d_f64::bounding_volume::{Aabb, BoundingSphere, BoundingVolume};
use parry3d_f64::query::distance;
use parry3d_f64::shape::{Ball, Cuboid};
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use crate::offset_shape::OffsetShape;

pub trait BvhShape : Sized {
    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self;
    fn new_from_combined(bvh_shapes: &Vec<Self>) -> Self;
    fn volume(&self) -> f64;
    fn intersect(&self, other: &Self) -> bool;
    fn signed_distance(&self, other: &Self) -> f64;
}

#[derive(Clone, Debug)]
pub struct BvhShapeAABB {
    cuboid: Cuboid,
    aabb: Aabb,
    pose: ISE3q
}
impl BvhShapeAABB {
    #[inline]
    fn new_from_aabbs(aabbs: &Vec<Aabb>) -> Self {
        let max_x = aabbs.iter().max_by(|x, y| x.maxs.x.partial_cmp(&y.maxs.x).unwrap()).unwrap().maxs.x;
        let max_y = aabbs.iter().max_by(|x, y| x.maxs.y.partial_cmp(&y.maxs.y).unwrap()).unwrap().maxs.y;
        let max_z = aabbs.iter().max_by(|x, y| x.maxs.z.partial_cmp(&y.maxs.z).unwrap()).unwrap().maxs.z;

        let min_x = aabbs.iter().min_by(|x, y| x.mins.x.partial_cmp(&y.mins.x).unwrap()).unwrap().mins.x;
        let min_y = aabbs.iter().min_by(|x, y| x.mins.y.partial_cmp(&y.mins.y).unwrap()).unwrap().mins.y;
        let min_z = aabbs.iter().min_by(|x, y| x.mins.z.partial_cmp(&y.mins.z).unwrap()).unwrap().mins.z;

        let aabb = Aabb::new(Point3::new(min_x, min_y, min_z), Point3::new(max_x, max_y, max_z));
        let cuboid = Cuboid::new(aabb.half_extents());
        let pose = ISE3q::new(I3::from_slices_euler_angles(&aabb.center().coords.as_slice(), &[0.,0.,0.]));

        Self {
            cuboid,
            aabb,
            pose,
        }
    }
}
impl BvhShape for BvhShapeAABB {
    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self {
        assert_eq!(offset_shapes.len(), poses.len());

        let aabbs: Vec<Aabb> = offset_shapes.iter().zip(poses).map(|(x, y)| {
            let pose = x.get_transform(y);
            x.shape().compute_aabb(&pose.as_ref().0)
        }).collect();

        Self::new_from_aabbs(&aabbs)
    }

    fn new_from_combined(bvh_shapes: &Vec<Self>) -> Self {
        let aabbs = bvh_shapes.iter().map(|x| x.aabb).collect();
        Self::new_from_aabbs(&aabbs)
    }

    #[inline(always)]
    fn volume(&self) -> f64 {
        self.aabb.volume()
    }

    #[inline(always)]
    fn intersect(&self, other: &Self) -> bool {
        self.aabb.intersects(&other.aabb)
    }

    #[inline(always)]
    fn signed_distance(&self, other: &Self) -> f64 {
        distance(&self.pose.0, &self.cuboid, &other.pose.0, &other.cuboid).expect("error")
    }
}

#[derive(Clone, Debug)]
pub struct BvhShapeBoundingSphere {
    ball: Ball,
    bounding_sphere: BoundingSphere,
    pose: ISE3q
}
impl BvhShapeBoundingSphere {
    fn new_from_bounding_spheres(bounding_spheres: &Vec<BoundingSphere>) -> Self {
        assert!(bounding_spheres.len() > 1);
        let mut center = V3::zeros();
        bounding_spheres.iter().for_each(|x| center += x.center.coords);
        center /= bounding_spheres.len() as f64;

        let mut radius = f64::NEG_INFINITY;
        bounding_spheres.iter().for_each(|x| {
            let tmp = (x.center.coords - &center).norm() + x.radius;
            if tmp > radius { radius = tmp; }
        });

        let ball = Ball::new(radius);
        let bounding_sphere = BoundingSphere::new(center.try_into().unwrap(), radius);
        let pose = ISE3q::new(I3::from_slices_euler_angles(&center.as_slice(), &[0.,0.,0.]));

        Self {
            ball,
            bounding_sphere,
            pose,
        }
    }
}
impl BvhShape for BvhShapeBoundingSphere {
    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self {
        let bounding_spheres = offset_shapes.iter().zip(poses.iter()).map(|(x, y)| {
            let pose = x.get_transform(y);
            x.shape().compute_bounding_sphere(&pose.0)
        }).collect();

        Self::new_from_bounding_spheres(&bounding_spheres)
    }

    fn new_from_combined(bvh_shapes: &Vec<Self>) -> Self {
        let bounding_spheres = bvh_shapes.iter().map(|x| x.bounding_sphere).collect();
        Self::new_from_bounding_spheres(&bounding_spheres)
    }

    fn volume(&self) -> f64 {
        4.0 * std::f64::consts::PI * self.bounding_sphere.radius.powi(2)
    }

    fn intersect(&self, other: &Self) -> bool {
        self.bounding_sphere.intersects(&other.bounding_sphere)
    }

    fn signed_distance(&self, other: &Self) -> f64 {
        distance(&self.pose.0, &self.ball, &other.pose.0, &other.ball).expect("error")
    }
}

#[derive(Clone, Debug)]
pub struct Bvh {

}