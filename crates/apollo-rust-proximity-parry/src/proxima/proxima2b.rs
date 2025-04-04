/*
use std::borrow::Cow;
use nalgebra::DMatrix;
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_spatial::lie::h1::ApolloUnitQuaternionH1LieTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use crate::double_group_queries::{DoubleGroupProximityQueryMode, pairwise_group_query_contact};
use crate::offset_shape::OffsetShape;
use crate::proxima::proxima_core::{ProximaCacheElementTrait, ProximaCacheTrait, ProximaTrait};

#[derive(Clone, Debug)]
pub struct Proxima2bCache {
    pub elements: DMatrix<Proxima2bCacheElement>
}
impl Proxima2bCache {
    pub fn new(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupProximityQueryMode, skips: Option<&DMatrix<bool>>) -> Self {
        if let DoubleGroupProximityQueryMode::SubsetOfPairs(_) = query_mode {
            panic!("you probably shouldn't use a subset of pairs to initialize the Proxima cache.");
        }
        assert_eq!(group_a.len(), poses_a.len());
        assert_eq!(group_b.len(), poses_b.len());

        let mut elements = DMatrix::from_vec(group_a.len(), group_b.len(), vec![Proxima2bCacheElement::default(); group_a.len()*group_b.len()]);
        let contacts = pairwise_group_query_contact(group_a, poses_a, group_b, poses_b, query_mode, skips, false, f64::INFINITY);
        // contacts.iter().for_each(|((i,j),c)| {
        contacts.outputs.iter().zip(contacts.shape_idxs).for_each(|(c, (i, j))| {
            let pose_a_j = &poses_a[i];
            let pose_b_j = &poses_b[j];
            let disp_between_a_and_b_j = pose_a_j.displacement(pose_b_j);
            let c = c.unwrap();
            let raw_distance_j = c.dist;
            let closest_point_a_j = c.point1.coords.xyz();
            let closest_point_b_j = c.point2.coords.xyz();

            elements[(i,j)].shape_indices = (i,j);
            elements[(i,j)].pose_a_j = pose_a_j.clone();
            elements[(i,j)].pose_b_j = pose_b_j.clone();
            elements[(i,j)].disp_between_a_and_b_j = disp_between_a_and_b_j;
            elements[(i,j)].raw_distance_j = raw_distance_j;
            elements[(i,j)].closest_point_a_j = closest_point_a_j;
            elements[(i,j)].closest_point_b_j = closest_point_b_j;
        });

        Self {
            elements,
        }
    }
}
impl ProximaCacheTrait<Proxima2bCacheElement> for Proxima2bCache {
    fn elements(&self) -> &DMatrix<Proxima2bCacheElement> {
        &self.elements
    }

    fn elements_mut(&mut self) -> &mut DMatrix<Proxima2bCacheElement> {
        &mut self.elements
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Proxima2bCacheElement {
    pub shape_indices: (usize, usize),
    pub pose_a_j: ISE3q,
    pub pose_b_j: ISE3q,
    pub closest_point_a_j: V3,
    pub closest_point_b_j: V3,
    pub raw_distance_j: f64,
    pub disp_between_a_and_b_j: ISE3q
}
impl Default for Proxima2bCacheElement {
    fn default() -> Self {
        Self {
            shape_indices: (0, 0),
            pose_a_j: Default::default(),
            pose_b_j: Default::default(),
            closest_point_a_j: Default::default(),
            closest_point_b_j: Default::default(),
            raw_distance_j: -100000.0,
            disp_between_a_and_b_j: Default::default(),
        }
    }
}
impl ProximaCacheElementTrait for Proxima2bCacheElement {
    #[inline(always)]
    fn update_element_with_ground_truth(&mut self, shape_a: &OffsetShape, pose_a: &ISE3q, shape_b: &OffsetShape, pose_b: &ISE3q) -> f64 {
        let contact = shape_a.contact(pose_a, shape_b, pose_b, f64::INFINITY).unwrap();

        self.pose_a_j = pose_a.clone();
        self.pose_b_j = pose_b.clone();
        self.raw_distance_j = contact.dist;
        self.disp_between_a_and_b_j = pose_a.displacement(pose_b);
        self.closest_point_a_j = contact.point1.coords;
        self.closest_point_b_j = contact.point2.coords;

        contact.dist
    }
}

#[derive(Clone, Debug)]
pub struct Proxima2b {
    pub cache: Proxima2bCache,
    pub max_distances_from_origin_a: Vec<f64>,
    pub max_distances_from_origin_b: Vec<f64>,
    pub interpolation: f64
}
impl ProximaTrait for Proxima2b {
    type CacheType = Proxima2bCache;
    type CacheElementType = Proxima2bCacheElement;
    type ExtraArgs = Proxima2bExtraArgs;

    #[inline(always)]
    fn get_extra_args(&self, i: usize, j: usize) -> Cow<Self::ExtraArgs> {
        Cow::Owned( Proxima2bExtraArgs {
            max_distance_from_origin_a: self.max_distances_from_origin_a[i],
            max_distance_from_origin_b: self.max_distances_from_origin_b[j],
            interpolation: self.interpolation,
        } )
    }

    #[inline(always)]
    fn get_cache_mut(&mut self) -> &mut Self::CacheType {
        &mut self.cache
    }

    #[inline(always)]
    fn get_cache_immut(&self) -> &Self::CacheType {
        &self.cache
    }

    #[inline(always)]
    fn approximate_distance_and_bounds(cache_element: &Self::CacheElementType, pose_a_k: &ISE3q, pose_b_k: &ISE3q, cutoff_distance: f64, extra_args: &Self::ExtraArgs) -> Option<(f64, f64, f64)> {
        let d = cache_element.disp_between_a_and_b_j.displacement(&pose_a_k.displacement(&pose_b_k));
        let delta_r = d.0.rotation.to_lie_group_h1().ln().vee().norm();

        let h = extra_args.max_distance_from_origin_a.max(extra_args.max_distance_from_origin_b);
        let psi = f64::sqrt(2.0 * h * h * (1.0 - f64::cos(delta_r)));

        let disp_pos_a = &pose_a_k.0.translation.vector - &cache_element.pose_a_j.0.translation.vector;
        let disp_pos_b = &pose_b_k.0.translation.vector - &cache_element.pose_b_j.0.translation.vector;
        let new_pos_a = cache_element.closest_point_a_j + disp_pos_a;
        let new_pos_b = cache_element.closest_point_b_j + disp_pos_b;

        let starting_lower_bound = (new_pos_a - new_pos_b).norm();

        let lower_bound = starting_lower_bound - psi;

        if lower_bound > cutoff_distance { return None; }

        let upper_bound = Self::distance_upper_bound(cache_element, pose_a_k, pose_b_k);
        let upper_bound = upper_bound.min(cache_element.raw_distance_j + psi);

        let approximation_distance = (1.0 - extra_args.interpolation) * lower_bound + extra_args.interpolation * upper_bound;
        Some((approximation_distance, lower_bound, upper_bound))
    }
}
impl Proxima2b {
    pub fn distance_lower_bound(cache_element: &Proxima2bCacheElement, pose_a_k: &ISE3q, pose_b_k: &ISE3q, max_distance_from_origin_a: f64, max_distance_from_origin_b: f64) -> f64 {
        let d = cache_element.disp_between_a_and_b_j.displacement(&pose_a_k.displacement(&pose_b_k));
        // let delta_m = d.0.translation.vector.norm();
        let delta_r = d.0.rotation.to_lie_group_h1().ln().vee().norm();

        let h = max_distance_from_origin_a.max(max_distance_from_origin_b);
        let psi = f64::sqrt(2.0 * h * h * (1.0 - f64::cos(delta_r)));

        // return cache_element.raw_distance_j - delta_m - psi;

        let disp_pos_a = &pose_a_k.0.translation.vector - &cache_element.pose_a_j.0.translation.vector;
        let disp_pos_b = &pose_b_k.0.translation.vector - &cache_element.pose_b_j.0.translation.vector;
        let new_pos_a = cache_element.closest_point_a_j + disp_pos_a;
        let new_pos_b = cache_element.closest_point_b_j + disp_pos_b;

        let starting_lower_bound = (new_pos_a - new_pos_b).norm();

        // let delta_r_a = cache_element.pose_a_j.0.rotation.to_lie_group_h1().displacement(&pose_a_k.0.rotation.to_lie_group_h1()).0.angle();
        // let delta_r_b = cache_element.pose_b_j.0.rotation.to_lie_group_h1().displacement(&pose_b_k.0.rotation.to_lie_group_h1()).0.angle();

        // let psi_a = f64::sqrt(2.0 * max_distance_from_origin_a * max_distance_from_origin_a * (1.0 - f64::cos(delta_r_a)));
        // let psi_b = f64::sqrt(2.0 * max_distance_from_origin_b * max_distance_from_origin_b * (1.0 - f64::cos(delta_r_b)));

        return starting_lower_bound - psi;
    }

    pub fn distance_upper_bound(cache_element: &Proxima2bCacheElement, pose_a_k: &ISE3q, pose_b_k: &ISE3q) -> f64 {
        let closest_point_a_k = pose_a_k.map_point(&cache_element.pose_a_j.inverse().map_point(&cache_element.closest_point_a_j));
        let closest_point_b_k = pose_b_k.map_point(&cache_element.pose_b_j.inverse().map_point(&cache_element.closest_point_b_j));

        return (&closest_point_a_k - &closest_point_b_k).norm()
    }
}

#[derive(Clone, Debug)]
pub struct Proxima2bExtraArgs {
    pub max_distance_from_origin_a: f64,
    pub max_distance_from_origin_b: f64,
    pub interpolation: f64
}
*/