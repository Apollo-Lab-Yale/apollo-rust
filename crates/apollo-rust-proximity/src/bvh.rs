use nalgebra::Point3;
use parry3d_f64::bounding_volume::{Aabb, BoundingSphere, BoundingVolume};
use parry3d_f64::query::{contact};
use parry3d_f64::shape::{Ball, Cuboid};
use apollo_rust_algs::combinations_of_n;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use crate::offset_shape::OffsetShape;

pub trait BvhShape : Sized + Clone {
    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self;
    fn new_from_combined(bvh_shapes: &Vec<Self>) -> Self;
    fn volume(&self) -> f64;
    fn intersect(&self, other: &Self) -> bool;
    fn signed_distance(&self, other: &Self) -> f64;
}

/*
pub trait BvhShapeBuilder {
    type BvhShapeType : BvhShape;

    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self::BvhShapeType;
    fn new_from_combined(bvh_shapes: &Vec<Self::BvhShapeType>) -> Self::BvhShapeType;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

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

        BvhShapeAABB::new_from_aabbs(&aabbs)
    }

    fn new_from_combined(bvh_shapes: &Vec<Self>) -> Self {
        let aabbs = bvh_shapes.iter().map(|x| x.aabb).collect();
        BvhShapeAABB::new_from_aabbs(&aabbs)
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
        contact(&self.pose.0, &self.cuboid, &other.pose.0, &other.cuboid, f64::INFINITY).expect("error").unwrap().dist
    }
}

/*
pub struct BvhShapeBuilderAABB;
impl BvhShapeBuilder for BvhShapeBuilderAABB {
    type BvhShapeType = BvhShapeAABB;

    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self::BvhShapeType {
        assert_eq!(offset_shapes.len(), poses.len());

        let aabbs: Vec<Aabb> = offset_shapes.iter().zip(poses).map(|(x, y)| {
            let pose = x.get_transform(y);
            x.shape().compute_aabb(&pose.as_ref().0)
        }).collect();

        BvhShapeAABB::new_from_aabbs(&aabbs)
    }

    fn new_from_combined(bvh_shapes: &Vec<Self::BvhShapeType>) -> Self::BvhShapeType {
        let aabbs = bvh_shapes.iter().map(|x| x.aabb).collect();
        BvhShapeAABB::new_from_aabbs(&aabbs)
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)]
pub struct BvhShapeBoundingSphere {
    ball: Ball,
    bounding_sphere: BoundingSphere,
    pose: ISE3q
}
impl BvhShapeBoundingSphere {
    fn new_from_bounding_spheres(bounding_spheres: &Vec<BoundingSphere>) -> Self {
        assert!(bounding_spheres.len() > 0);
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

        BvhShapeBoundingSphere::new_from_bounding_spheres(&bounding_spheres)
    }

    fn new_from_combined(bvh_shapes: &Vec<Self>) -> Self {
        let bounding_spheres = bvh_shapes.iter().map(|x| x.bounding_sphere).collect();
        BvhShapeBoundingSphere::new_from_bounding_spheres(&bounding_spheres)
    }

    fn volume(&self) -> f64 {
        4.0 * std::f64::consts::PI * self.bounding_sphere.radius.powi(2)
    }

    fn intersect(&self, other: &Self) -> bool {
        self.bounding_sphere.intersects(&other.bounding_sphere)
    }

    fn signed_distance(&self, other: &Self) -> f64 {
        contact(&self.pose.0, &self.ball, &other.pose.0, &other.ball, f64::INFINITY).expect("error").unwrap().dist
    }
}

/*
pub struct BvhShapeBuilderBoundingSphere;
impl BvhShapeBuilder for BvhShapeBuilderBoundingSphere {
    type BvhShapeType = BvhShapeBoundingSphere;

    fn new_from_offset_shapes(offset_shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) -> Self::BvhShapeType {
        let bounding_spheres = offset_shapes.iter().zip(poses.iter()).map(|(x, y)| {
            let pose = x.get_transform(y);
            x.shape().compute_bounding_sphere(&pose.0)
        }).collect();

        BvhShapeBoundingSphere::new_from_bounding_spheres(&bounding_spheres)
    }

    fn new_from_combined(bvh_shapes: &Vec<Self::BvhShapeType>) -> Self::BvhShapeType {
        let bounding_spheres = bvh_shapes.iter().map(|x| x.bounding_sphere).collect();
        BvhShapeBoundingSphere::new_from_bounding_spheres(&bounding_spheres)
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)]
pub struct Bvh<B: BvhShape> {
    pub layers: Vec<BvhLayer<B>>,
    pub branch_factor: usize
}
impl<B: BvhShape> Bvh<B> {
    pub fn build(shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>, branch_factor: usize) -> Self {
        let bvh_shapes = shapes.iter().zip(poses.iter()).map(|(x, y)| B::new_from_offset_shapes(&vec![x.clone()], &vec![y.clone()])).collect();
        let mut layers = vec![BvhLayer::build(&bvh_shapes, branch_factor)];
        while !(layers[0].bvh_shapes.len() == 1) {
            let new_layer = BvhLayer::build(&layers[0].bvh_shapes, branch_factor);
            layers.insert(0, new_layer);
        }
        Self {
            layers,
            branch_factor,
        }
    }

    pub fn update(&mut self, shapes: &Vec<OffsetShape>, poses: &Vec<ISE3q>) {
        let bvh_shapes = shapes.iter().zip(poses.iter()).map(|(x, y)| B::new_from_offset_shapes(&vec![x.clone()], &vec![y.clone()])).collect();
        *self.layers.last_mut().unwrap() = BvhLayer::build(&bvh_shapes, self.branch_factor);
        if self.layers.len() == 1 { return; }
        let mut curr_idx = self.layers.len() - 2;
        loop {
            let l = self.layers[curr_idx].bvh_shapes.len();
            for i in 0..l {
                let parent_idxs = self.layers[curr_idx].parent_indices[i].clone();
                let parent_shapes = parent_idxs.iter().map(|x| self.layers[curr_idx+1].bvh_shapes[*x].clone() ).collect();
                let updated_shape = B::new_from_combined(&parent_shapes);
                self.layers[curr_idx].bvh_shapes[i] = updated_shape;
            }

            if curr_idx == 0 { return; }
            curr_idx -= 1;
        }
    }

    pub fn intersection_filter(&self, other: &Bvh<B>) -> Vec<(usize, usize)> {
        let self_num_layers = self.layers.len();
        let other_num_layers = other.layers.len();

        let mut self_curr_layer = 0;
        let mut other_curr_layer = 0;

        let mut out_list = vec![(0,0)];

        loop {
            let out_list_clone = out_list.clone();
            let mut curr_out_list = vec![];

            out_list_clone.iter().for_each(|(i,j)| {
                let self_shape = &self.layers[self_curr_layer].bvh_shapes[*i];
                let other_shape = &other.layers[other_curr_layer].bvh_shapes[*j];
                let intersect = self_shape.intersect(other_shape);
                if intersect {
                    for self_idx in &self.layers[self_curr_layer].parent_indices[*i] {
                        for other_idx in &other.layers[other_curr_layer].parent_indices[*j] {
                            curr_out_list.push( (*self_idx, *other_idx) );
                        }
                    }
                }
            });

            out_list = curr_out_list;
            if self_curr_layer == self_num_layers - 1 && other_curr_layer == other_num_layers - 1 { return out_list; }
            self_curr_layer = (self_curr_layer + 1).min(self_num_layers - 1);
            other_curr_layer = (other_curr_layer + 1).min(other_num_layers - 1);
        }

    }

    pub fn distance_filter(&self, other: &Bvh<B>, distance_threshold: f64) -> Vec<(usize, usize)> {
        let self_num_layers = self.layers.len();
        let other_num_layers = other.layers.len();

        let mut self_curr_layer = 0;
        let mut other_curr_layer = 0;

        let mut out_list = vec![(0,0)];

        loop {
            let out_list_clone = out_list.clone();
            let mut curr_out_list = vec![];

            out_list_clone.iter().for_each(|(i,j)| {
                let self_shape = &self.layers[self_curr_layer].bvh_shapes[*i];
                let other_shape = &other.layers[other_curr_layer].bvh_shapes[*j];
                let distance = self_shape.signed_distance(other_shape);
                if distance < distance_threshold {
                    for self_idx in &self.layers[self_curr_layer].parent_indices[*i] {
                        for other_idx in &other.layers[other_curr_layer].parent_indices[*j] {
                            curr_out_list.push( (*self_idx, *other_idx) );
                        }
                    }
                }
            });

            out_list = curr_out_list;
            if self_curr_layer == self_num_layers - 1 && other_curr_layer == other_num_layers - 1 { return out_list; }
            self_curr_layer = (self_curr_layer + 1).min(self_num_layers - 1);
            other_curr_layer = (other_curr_layer + 1).min(other_num_layers - 1);
        }

    }
}

#[derive(Clone, Debug)]
pub struct BvhLayer<B: BvhShape> {
    pub bvh_shapes: Vec<B>,
    pub parent_indices: Vec<Vec<usize>>
}
impl<B: BvhShape> BvhLayer<B> {
    pub fn build(bvh_shapes: &Vec<B>, branch_factor: usize) -> Self {
        let res = get_bvh_layer_info(bvh_shapes, branch_factor);
        return Self {
            bvh_shapes: res.0,
            parent_indices: res.1,
        }
    }
}

pub fn get_bvh_layer_info<B: BvhShape>(inputs: &Vec<B>, branch_factor: usize) -> (Vec<B>, Vec<Vec<usize>>) {
    if inputs.len() < branch_factor { return get_bvh_layer_info(inputs, 1); }

    let mut out_shapes = vec![];
    let mut out_idxs = vec![];

    let combinations = combinations_of_n((0..inputs.len()).collect(), branch_factor);

    let mut done = vec![false; inputs.len()];

    let res: Vec<(B, f64)> = combinations.iter().map(|x| {
        let tmp: Vec<B> = x.iter().map(|y| inputs[*y].clone()).collect();
        let combined = B::new_from_combined(&tmp);
        let v = combined.volume();
        (combined, v)
    }).collect();

    let combined_shapes: Vec<B> = res.iter().map(|x| x.0.clone()).collect();
    let volumes: Vec<f64> = res.iter().map(|y| y.1.clone()).collect();

    let mut sorted_idxs: Vec<usize> = (0..volumes.len()).collect();
    sorted_idxs.sort_by(|x, y| volumes[*x].partial_cmp(&volumes[*y]).unwrap());

    'l: for idx in sorted_idxs {
        let combination = &combinations[idx];
        for c in combination { if done[*c] { continue 'l; } }
        for c in combination { done[*c] = true; }

        out_shapes.push(combined_shapes[idx].clone());
        out_idxs.push(combination.clone());
    }

    let mut not_done = vec![];
    for i in 0..done.len() { if !done[i] { not_done.push(i); } }

    for i in not_done {
        out_shapes.push(inputs[i].clone());
        out_idxs.push(vec![i]);
    }

    (out_shapes, out_idxs)
}

