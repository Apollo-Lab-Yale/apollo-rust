use std::time::Instant;
use nalgebra::DMatrix;
use parry3d_f64::query::contact;
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_spatial::lie::h1::ApolloUnitQuaternionH1LieTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use crate::double_group_queries::{DoubleGroupQueryMode, pairwise_group_query_contact};
use crate::offset_shape::OffsetShape;
use crate::proxima::proxima_core::{ProximaBudget, ProximaOutput, ToAverageDistancesProximaOutput};
use crate::{DistanceMode, ProximityLossFunction, ToProximityValue};

#[derive(Clone, Debug)]
pub struct Proxima1Cache {
    pub elements: DMatrix<Proxima1CacheElement>
}
impl Proxima1Cache {
    pub fn new(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupQueryMode, skips: Option<&DMatrix<bool>>) -> Self {
        if let DoubleGroupQueryMode::SubsetOfPairs(_) = query_mode {
            panic!("you probably shouldn't use a subset of pairs to initialize the Proxima cache.");
        }
        assert_eq!(group_a.len(), poses_a.len());
        assert_eq!(group_b.len(), poses_b.len());

        let mut elements = DMatrix::from_vec(group_a.len(), group_b.len(), vec![Proxima1CacheElement::default(); group_a.len()*group_b.len()]);
        let contacts = pairwise_group_query_contact(group_a, poses_a, group_b, poses_b, query_mode, skips, false, f64::INFINITY);
        contacts.iter().for_each(|((i,j),c)| {
            let pose_a_j = &poses_a[*i];
            let pose_b_j = &poses_b[*j];
            let disp_between_a_and_b_j = pose_a_j.displacement(pose_b_j);
            let c = c.unwrap();
            let raw_distance_j = c.dist;
            let closest_point_a_j = c.point1.coords.xyz();
            let closest_point_b_j = c.point2.coords.xyz();

            elements[(*i,*j)].pose_a_j = pose_a_j.clone();
            elements[(*i,*j)].pose_b_j = pose_b_j.clone();
            elements[(*i,*j)].disp_between_a_and_b_j = disp_between_a_and_b_j;
            elements[(*i,*j)].raw_distance_j = raw_distance_j;
            elements[(*i,*j)].closest_point_a_j = closest_point_a_j;
            elements[(*i,*j)].closest_point_b_j = closest_point_b_j;
        });

        Self {
            elements,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Proxima1CacheElement {
    pub pose_a_j: ISE3q,
    pub pose_b_j: ISE3q,
    pub closest_point_a_j: V3,
    pub closest_point_b_j: V3,
    pub raw_distance_j: f64,
    pub disp_between_a_and_b_j: ISE3q
}
impl Default for Proxima1CacheElement {
    fn default() -> Self {
        Self {
            pose_a_j: Default::default(),
            pose_b_j: Default::default(),
            closest_point_a_j: Default::default(),
            closest_point_b_j: Default::default(),
            raw_distance_j: -100000.0,
            disp_between_a_and_b_j: Default::default(),
        }
    }
}

#[inline]
pub fn proxima1_lower_bound(cache_element: &Proxima1CacheElement, pose_a_k: &ISE3q, pose_b_k: &ISE3q, max_distance_from_origin_a: f64, max_distance_from_origin_b: f64) -> f64 {
    let d = cache_element.disp_between_a_and_b_j.displacement(&pose_a_k.displacement(&pose_b_k));
    let delta_m = d.0.translation.vector.norm();
    let delta_r = d.0.rotation.to_lie_group_h1().ln().vee().norm();

    let h = max_distance_from_origin_a.max(max_distance_from_origin_b);
    let psi = f64::sqrt(2.0 * h * h * (1.0 - f64::cos(delta_r)));

    return cache_element.raw_distance_j - delta_m - psi;
}

#[inline]
pub fn proxima1_upper_bound(cache_element: &Proxima1CacheElement, pose_a_k: &ISE3q, pose_b_k: &ISE3q) -> f64 {
    let closest_point_a_k = pose_a_k.map_point(&cache_element.pose_a_j.inverse().map_point(&cache_element.closest_point_a_j));
    let closest_point_b_k = pose_b_k.map_point(&cache_element.pose_b_j.inverse().map_point(&cache_element.closest_point_b_j));

    return (&closest_point_a_k - &closest_point_b_k).norm()
}

/// interpolation from 0 to 1 returns an approximation distance of lower bound for 0 and upper bound for 1
#[inline]
pub fn proxima1_approximate_distance_and_bounds(interpolation: f64, cache_element: &Proxima1CacheElement, pose_a_k: &ISE3q, pose_b_k: &ISE3q, max_distance_from_origin_a: f64, max_distance_from_origin_b: f64) -> (f64, f64, f64) {
    let lower_bound = proxima1_lower_bound(cache_element, pose_a_k, pose_b_k, max_distance_from_origin_a, max_distance_from_origin_b);
    let upper_bound = proxima1_upper_bound(cache_element, pose_a_k, pose_b_k);

    let approximation_distance = (1.0 - interpolation) * lower_bound + interpolation * upper_bound;

    return (approximation_distance, lower_bound, upper_bound);
}

pub fn proxima1_get_all_approximate_distances_and_bounds(cache: &Proxima1Cache, poses_a: &Vec<ISE3q>, poses_b: &Vec<ISE3q>, max_distances_from_origin_a: &Vec<f64>, max_distances_from_origin_b: &Vec<f64>, interpolation: f64, query_mode: &DoubleGroupQueryMode, skips: Option<&DMatrix<bool>>, average_distances: Option<&DMatrix<f64>>, cutoff_distance: f64) -> Vec<ProximaOutput> {
    let mut out = vec![];

    let mut f = |i: usize, pa: &ISE3q, j: usize, pb: &ISE3q| {
        match skips {
            None => {}
            Some(skips) => {
                if skips[(i,j)] { return; }
            }
        }

        let max_distance_from_origin_a = max_distances_from_origin_a[i];
        let max_distance_from_origin_b = max_distances_from_origin_b[j];
        let cache_element = &cache.elements[(i,j)];

        let res = proxima1_approximate_distance_and_bounds(interpolation, cache_element, pa, pb, max_distance_from_origin_a, max_distance_from_origin_b);
        out.push(ProximaOutput {
            shape_indices: (i, j),
            distance_mode: DistanceMode::RawDistance,
            approximate_distance: res.0,
            lower_bound_distance: res.1,
            upper_bound_distance: res.2,
        });
    };

    match query_mode {
        DoubleGroupQueryMode::AllPossiblePairs => {
            for (i, pa) in poses_a.iter().enumerate() {
                for (j, pb) in poses_b.iter().enumerate() {
                    f(i, pa, j, pb);
                }
            }
        }
        DoubleGroupQueryMode::SkipSymmetricalPairs => {
            for (i, pa) in poses_a.iter().enumerate() {
                'l: for (j, pb) in poses_b.iter().enumerate() {
                    if i >= j { continue 'l; }
                    f(i, pa, j, pb);
                }
            }
        }
        DoubleGroupQueryMode::SubsetOfPairs(v) => {
            for (i,j) in v {
                let pa = &poses_a[*i];
                let pb = &poses_b[*j];
                f(*i, pa, *j, pb);
            }
        }
    }

    match average_distances {
        None => {}
        Some(average_distances) => { out = out.to_average_distances(average_distances); }
    }

    let out = out.iter().filter(|x| x.lower_bound_distance < cutoff_distance).map(|x| x.clone()).collect();

    out
}

pub fn proxima1(cache: &mut Proxima1Cache,
                budget: &ProximaBudget,
                group_a: &Vec<OffsetShape>,
                poses_a: &Vec<ISE3q>,
                group_b: &Vec<OffsetShape>,
                poses_b: &Vec<ISE3q>,
                query_mode: &DoubleGroupQueryMode,
                loss_function: &ProximityLossFunction,
                p_norm: f64,
                cutoff_distance: f64,
                max_distances_from_origin_a: &Vec<f64>,
                max_distances_from_origin_b: &Vec<f64>,
                skips: Option<&DMatrix<bool>>,
                average_distances: Option<&DMatrix<f64>>,
                frozen: bool) -> (f64, usize) {

    let start = Instant::now();

    let mut proxima_outputs = proxima1_get_all_approximate_distances_and_bounds(cache, poses_a, poses_b, max_distances_from_origin_a, max_distances_from_origin_b, 0.0, query_mode, skips, average_distances, cutoff_distance);

    if frozen { return ( proxima_outputs.to_proximity_value(p_norm), 0 ) }

    let mut max_possible_loss_errors = vec![];
    proxima_outputs.iter().for_each(|x| {
        let a = loss_function.loss(x.approximate_distance);
        let l = loss_function.loss(x.lower_bound_distance);
        let u = loss_function.loss(x.upper_bound_distance);

        max_possible_loss_errors.push( f64::max((a - u).abs(), (a - l).abs()) );
    });

    let mut indexed_array: Vec<usize> = (0..max_possible_loss_errors.len()).collect();
    indexed_array.sort_by(|x, y| max_possible_loss_errors[*y].partial_cmp(&max_possible_loss_errors[*x]).unwrap());
    proxima_outputs = indexed_array.iter().map(|x| proxima_outputs[*x].clone()).collect();

    let mut num_ground_truth_checks = 0;
    let mut proximity_value = proxima_outputs.to_proximity_value(p_norm);

    'l: for k in 0..proxima_outputs.len() {
        match budget {
            ProximaBudget::TimeBudget(b) => {
                if start.elapsed() > *b { break 'l; }
            }
            ProximaBudget::AccuracyBudget(b) => {
                if proximity_value < *b { break 'l; }
            }
        }

        let (i, j) = proxima_outputs[k].shape_indices;
        let sa = &group_a[i];
        let sb = &group_b[j];
        let pa = &poses_a[i];
        let pb = &poses_b[j];

        let contact = contact(&sa.get_transform(&pa).as_ref().0, &**sa.shape(), &sb.get_transform(&pb).as_ref().0, &**sb.shape(), f64::INFINITY).expect("error").unwrap();

        let element = &mut cache.elements[(i, j)];
        element.pose_a_j = pa.clone();
        element.pose_b_j = pb.clone();
        element.raw_distance_j = contact.dist;
        element.disp_between_a_and_b_j = pa.displacement(pb);
        element.closest_point_a_j = contact.point1.coords.xyz();
        element.closest_point_a_j = contact.point2.coords.xyz();

        let mut new_distance = contact.dist;
        match average_distances {
            None => {}
            Some(average_distances) => { new_distance /= average_distances[(i,j)].max(0.00001) }
        }

        proxima_outputs[k].approximate_distance = new_distance;
        proxima_outputs[k].lower_bound_distance = new_distance;
        proxima_outputs[k].upper_bound_distance = new_distance;

        num_ground_truth_checks += 1;
        proximity_value = proxima_outputs.to_proximity_value(p_norm);
    }

    return (proximity_value, num_ground_truth_checks);
}