use std::sync::Arc;
use apollo_rust_linalg::{ApolloDMatrixTrait, M};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use serde::{Deserialize, Serialize};

pub trait ShapeTrait {
    fn support_function(&self, dir: &V3, shape_pose: &ISE3q) -> V3;
}
impl<S: ShapeTrait> ShapeTrait for Arc<S> {
    fn support_function(&self, dir: &V3, shape_pose: &ISE3q) -> V3 {
        (**self).support_function(dir, shape_pose)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ShapeBall {
    radius: f64
}
impl ShapeBall {
    pub fn new(radius: f64) -> Self {
        assert!(radius >= 0.0);
        Self { radius }
    }
}
impl ShapeTrait for ShapeBall {
    #[inline(always)]
    fn support_function(&self, dir: &V3, shape_pose: &ISE3q) -> V3 {
        let dn = dir.norm();
        if dn < 1e-12 {
            return shape_pose.0.translation.vector;
        }
        let dir_normalized = dir / dn;
        let dir_scaled = self.radius * dir_normalized;
        shape_pose.0.translation.vector + dir_scaled
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ShapeFromPoints {
    points: Vec<V3>,
    points_as_matrix: M,
    rotate_dir: bool
}
impl ShapeFromPoints {
    pub fn new(points: Vec<V3>, rotate_dir: bool) -> Self {
        let mut points_as_matrix = M::zeros(points.len(), 3);
        points.iter().enumerate().for_each(|(i, x)| {
            points_as_matrix.set_row_from_slice(i, x.as_slice());
        });
        Self {
            points,
            points_as_matrix,
            rotate_dir,
        }
    }
}
impl ShapeTrait for ShapeFromPoints {
    #[inline(always)]
    fn support_function(&self, dir: &V3, shape_pose: &ISE3q) -> V3 {
        let dir_local = if self.rotate_dir {
             shape_pose.0.inverse_transform_vector(&dir)
        } else {
            dir.clone()
        };
        let d = &self.points_as_matrix * &dir_local;
        let m = d.iter()
            .enumerate()
            .max_by(|(_, x), (_, y)| x.partial_cmp(y).expect(&format!("dir: {:?}", dir)))
            .map(|(i, _)| i)
            .expect("error");

        return shape_pose.map_point(&self.points[m]);
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ShapeAABB {
    shape_from_points: ShapeFromPoints
}
impl ShapeAABB {
    pub fn new(half_extents: V3) -> Self {
        let points = vec![
            V3::new(half_extents[0], half_extents[1], half_extents[2]),
            V3::new(-half_extents[0], half_extents[1], half_extents[2]),
            V3::new(half_extents[0], -half_extents[1], half_extents[2]),
            V3::new(half_extents[0], half_extents[1], -half_extents[2]),
            V3::new(-half_extents[0], -half_extents[1], half_extents[2]),
            V3::new(half_extents[0], -half_extents[1], -half_extents[2]),
            V3::new(-half_extents[0], half_extents[1], -half_extents[2]),
            V3::new(-half_extents[0], -half_extents[1], -half_extents[2]),
        ];
        Self {
            shape_from_points: ShapeFromPoints::new(points, false)
        }
    }
}
impl ShapeTrait for ShapeAABB {
    fn support_function(&self, dir: &V3, shape_pose: &ISE3q) -> V3 {
        self.shape_from_points.support_function(dir, shape_pose)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ShapeOBB {
    shape_from_points: ShapeFromPoints
}
impl ShapeOBB {
    pub fn new(half_extents: V3) -> Self {
        let points = vec![
            V3::new(half_extents[0], half_extents[1], half_extents[2]),
            V3::new(-half_extents[0], half_extents[1], half_extents[2]),
            V3::new(half_extents[0], -half_extents[1], half_extents[2]),
            V3::new(half_extents[0], half_extents[1], -half_extents[2]),
            V3::new(-half_extents[0], -half_extents[1], half_extents[2]),
            V3::new(half_extents[0], -half_extents[1], -half_extents[2]),
            V3::new(-half_extents[0], half_extents[1], -half_extents[2]),
            V3::new(-half_extents[0], -half_extents[1], -half_extents[2]),
        ];
        Self {
            shape_from_points: ShapeFromPoints::new(points, true)
        }
    }
}
impl ShapeTrait for ShapeOBB {
    fn support_function(&self, dir: &V3, shape_pose: &ISE3q) -> V3 {
        self.shape_from_points.support_function(dir, shape_pose)
    }
}