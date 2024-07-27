use std::borrow::Cow;
use std::time::{Duration, Instant};
use nalgebra::DMatrix;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::{DistanceMode, ProximityLossFunction, ToIntersectionResult, ToProximityValue};
use crate::double_group_queries::DoubleGroupQueryMode;
use crate::offset_shape::OffsetShape;

pub trait ProximaTrait {
    type CacheType: ProximaCacheTrait<Self::CacheElementType>;
    type CacheElementType;
    type ExtraArgs : Clone;

    fn get_extra_args(&self, i: usize, j: usize) -> Cow<Self::ExtraArgs>;
    fn approximate_distance_and_bounds(cache_element: &Self::CacheElementType, pose_a_k: &ISE3q, pose_b_k: &ISE3q, cutoff_distance: f64, extra_args: &Self::ExtraArgs) -> Option<(f64, f64, f64)>;
    fn f(&self, cache: &Self::CacheType, i: usize, pa: &ISE3q, j: usize, pb: &ISE3q, skips: Option<&DMatrix<bool>>, cutoff_distance: f64) -> Option<ProximaOutput> {
        match skips {
            None => {  }
            Some(skips) => {
                if skips[(i,j)] { return None; }
            }
        }

        let cache_element = &cache.elements()[(i,j)];
        let extra_args = self.get_extra_args(i, j);
        let res = Self::approximate_distance_and_bounds(cache_element, pa, pb, cutoff_distance, extra_args.as_ref());

        match res {
            None => { None }
            Some(res) => {
                Some(ProximaOutput {
                    shape_indices: (i, j),
                    distance_mode: DistanceMode::RawDistance,
                    approximate_distance: res.0,
                    lower_bound_distance: res.1,
                    upper_bound_distance: res.2,
                })
            }
        }
    }
    fn get_all_proxima_outputs_for_proximity(&self, cache: &Self::CacheType, poses_a: &Vec<ISE3q>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupQueryMode, skips: Option<&DMatrix<bool>>, average_distances: Option<&DMatrix<f64>>, cutoff_distance: f64) -> Vec<ProximaOutput> {
        let mut out = vec![];

        match query_mode {
            DoubleGroupQueryMode::AllPossiblePairs => {
                for (i, pa) in poses_a.iter().enumerate() {
                    for (j, pb) in poses_b.iter().enumerate() {
                        let output = self.f(cache, i, pa, j, pb, skips, cutoff_distance);
                        match output {
                            None => {}
                            Some(output) => { out.push(output); }
                        }
                    }
                }
            }
            DoubleGroupQueryMode::SkipSymmetricalPairs => {
                for (i, pa) in poses_a.iter().enumerate() {
                    'l: for (j, pb) in poses_b.iter().enumerate() {
                        if i >= j { continue 'l; }
                        let output = self.f(cache, i, pa, j, pb, skips, cutoff_distance);
                        match output {
                            None => {}
                            Some(output) => { out.push(output); }
                        }
                    }
                }
            }
            DoubleGroupQueryMode::SubsetOfPairs(v) => {
                for (i,j) in v {
                    let pa = &poses_a[*i];
                    let pb = &poses_b[*j];
                    let output = self.f(cache, *i, pa, *j, pb, skips, cutoff_distance);
                    match output {
                        None => {}
                        Some(output) => { out.push(output); }
                    }
                }
            }
        }

        match average_distances {
            None => {  }
            Some(average_distances) => { out = out.to_average_distances(average_distances); }
        }

        out
    }
    fn get_all_proxima_outputs_for_intersection(&self, cache: &Self::CacheType, poses_a: &Vec<ISE3q>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupQueryMode, skips: Option<&DMatrix<bool>>, cutoff_distance: f64) -> Option<Vec<ProximaOutput>> {
        let mut out = vec![];

        match query_mode {
            DoubleGroupQueryMode::AllPossiblePairs => {
                for (i, pa) in poses_a.iter().enumerate() {
                    for (j, pb) in poses_b.iter().enumerate() {
                        let output = self.f(cache, i, pa, j, pb, skips, cutoff_distance);
                        match output {
                            None => { }
                            Some(output) => {
                                if output.upper_bound_distance < 0.0 { return None; }
                                out.push(output);
                            }
                        }
                    }
                }
            }
            DoubleGroupQueryMode::SkipSymmetricalPairs => {
                for (i, pa) in poses_a.iter().enumerate() {
                    'l: for (j, pb) in poses_b.iter().enumerate() {
                        if i >= j { continue 'l; }
                        let output = self.f(cache, i, pa, j, pb, skips, cutoff_distance);
                        match output {
                            None => {}
                            Some(output) => {
                                if output.upper_bound_distance < 0.0 { return None; }
                                out.push(output);
                            }
                        }
                    }
                }
            }
            DoubleGroupQueryMode::SubsetOfPairs(v) => {
                for (i,j) in v {
                    let pa = &poses_a[*i];
                    let pb = &poses_b[*j];
                    let output = self.f(cache, *i, pa, *j, pb, skips, cutoff_distance);
                    match output {
                        None => {}
                        Some(output) => {
                            if output.upper_bound_distance < 0.0 { return None; }
                            out.push(output);
                        }
                    }
                }
            }
        }

        Some(out)
    }
    fn proxima_for_proximity(&self,
                             cache: &mut Self::CacheType,
                             budget: &ProximaBudget,
                             group_a: &Vec<OffsetShape>,
                             poses_a: &Vec<ISE3q>,
                             group_b: &Vec<OffsetShape>,
                             poses_b: &Vec<ISE3q>,
                             query_mode: &DoubleGroupQueryMode,
                             loss_function: &ProximityLossFunction,
                             p_norm: f64,
                             cutoff_distance: f64,
                             skips: Option<&DMatrix<bool>>,
                             average_distances: Option<&DMatrix<f64>>,
                             frozen: bool) -> (f64, usize) {

        let start = Instant::now();
        let mut proxima_outputs = self.get_all_proxima_outputs_for_proximity(cache, poses_a, poses_b, query_mode, skips, average_distances, cutoff_distance);

        if frozen { return ( proxima_outputs.to_proximity_value(p_norm), 0 ) }

        let sorted_idxs = get_sorted_indices_of_maximum_possible_loss_function_error(&proxima_outputs, &loss_function);

        let mut num_ground_truth_checks = 0;
        let mut proximity_value = proxima_outputs.to_proximity_value(p_norm);

        'l: for k in 0..sorted_idxs.len() {
            match budget {
                ProximaBudget::TimeBudget(b) => {
                    if start.elapsed() > *b { break 'l; }
                }
                ProximaBudget::AccuracyBudget(b) => {
                    if proximity_value < *b { break 'l; }
                }
            }

            let idx = sorted_idxs[k];
            let proxima_output = &mut proxima_outputs[idx];
            let (i, j) = proxima_output.shape_indices;
            let sa = &group_a[i];
            let sb = &group_b[j];
            let pa = &poses_a[i];
            let pb = &poses_b[j];

            let mut new_distance = cache.update_element_with_ground_truth(i, j, sa, pa, sb, pb);

            match average_distances {
                None => { }
                Some(average_distances) => { new_distance /= average_distances[(i,j)].max(0.00001) }
            }

            proxima_output.approximate_distance = new_distance;
            proxima_output.lower_bound_distance = new_distance;
            proxima_output.upper_bound_distance = new_distance;

            num_ground_truth_checks += 1;
            proximity_value = proxima_outputs.to_proximity_value(p_norm);
        }

        return (proximity_value, num_ground_truth_checks);
    }
    fn proxima_for_intersection(&self,
                                cache: &mut Self::CacheType,
                                budget: &ProximaBudget,
                                group_a: &Vec<OffsetShape>,
                                poses_a: &Vec<ISE3q>,
                                group_b: &Vec<OffsetShape>,
                                poses_b: &Vec<ISE3q>,
                                query_mode: &DoubleGroupQueryMode,
                                loss_function: &ProximityLossFunction,
                                p_norm: f64,
                                skips: Option<&DMatrix<bool>>,
                                frozen: bool) -> (bool, usize) {
        let start = Instant::now();
        let proxima_outputs = self.get_all_proxima_outputs_for_intersection(cache, poses_a, poses_b, query_mode, skips, 0.0);

        return match proxima_outputs {
            None => { (true, 0) }
            Some(mut proxima_outputs) => {
                if frozen { return (proxima_outputs.to_intersection_result(), 0) }

                let sorted_idxs = get_sorted_indices_of_maximum_possible_loss_function_error(&proxima_outputs, &loss_function);

                let mut num_ground_truth_checks = 0;
                let mut proximity_value = proxima_outputs.to_proximity_value(p_norm);

                'l: for k in 0..sorted_idxs.len() {
                    match budget {
                        ProximaBudget::TimeBudget(b) => {
                            if start.elapsed() > *b { break 'l; }
                        }
                        ProximaBudget::AccuracyBudget(b) => {
                            if proximity_value < *b { break 'l; }
                        }
                    }

                    let idx = sorted_idxs[k];
                    let proxima_output = &mut proxima_outputs[idx];
                    let (i, j) = proxima_output.shape_indices;
                    let sa = &group_a[i];
                    let sb = &group_b[j];
                    let pa = &poses_a[i];
                    let pb = &poses_b[j];

                    let new_distance = cache.update_element_with_ground_truth(i, j, sa, pa, sb, pb);

                    if new_distance <= 0.0 { return (true, num_ground_truth_checks) }

                    proxima_output.approximate_distance = new_distance;
                    proxima_output.lower_bound_distance = new_distance;
                    proxima_output.upper_bound_distance = new_distance;

                    num_ground_truth_checks += 1;
                    proximity_value = proxima_outputs.to_proximity_value(p_norm);
                }

                (false, num_ground_truth_checks)
            }
        }
    }
}

pub trait ProximaCacheTrait<CacheElement> {
    fn elements(&self) -> &DMatrix<CacheElement>;
    fn elements_mut(&mut self) -> &mut DMatrix<CacheElement>;
    fn update_element_with_ground_truth(&mut self, i: usize, j: usize, shape_a: &OffsetShape, pose_a: &ISE3q, shape_b: &OffsetShape, pose_b: &ISE3q) -> f64;
}

#[derive(Clone, Debug)]
pub enum ProximaBudget {
    TimeBudget(Duration),
    AccuracyBudget(f64)
}

#[derive(Clone, Debug)]
pub struct ProximaOutput {
    pub shape_indices: (usize, usize),
    pub distance_mode: DistanceMode,
    pub approximate_distance: f64,
    pub lower_bound_distance: f64,
    pub upper_bound_distance: f64
}

pub trait ToAverageDistancesProximaOutput {
    fn to_average_distances(&self, average_distance: &DMatrix<f64>) -> Vec<ProximaOutput>;
}
impl ToAverageDistancesProximaOutput for Vec<ProximaOutput> {
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Vec<ProximaOutput> {
        let mut out = vec![];

        self.iter().for_each(|x| {
            let average = average_distances[(x.shape_indices.0, x.shape_indices.1)].max(0.00001);
            out.push(ProximaOutput {
                shape_indices: x.shape_indices.clone(),
                distance_mode: DistanceMode::AverageDistance,
                approximate_distance: x.approximate_distance / average,
                lower_bound_distance: x.lower_bound_distance / average,
                upper_bound_distance: x.upper_bound_distance / average,
            });
        });

        out
    }
}

impl ToProximityValue for Vec<ProximaOutput> {
    fn to_proximity_value(&self, p_norm: f64) -> f64 {
        let mut out = 0.0;

        self.iter().for_each(|x| out += x.approximate_distance.powf(p_norm));

        out.powf(1.0/p_norm)
    }
}
impl ToIntersectionResult for Vec<ProximaOutput> {
    fn to_intersection_result(&self) -> bool {
        for x in self.iter() {
            if x.approximate_distance <= 0.0 { return true; }
        }

        return false;
    }
}

pub fn get_sorted_indices_of_maximum_possible_loss_function_error(proxima_outputs: &Vec<ProximaOutput>,
                                                                  loss_function: &ProximityLossFunction) -> Vec<usize> {
    let mut max_possible_loss_errors = vec![];
    proxima_outputs.iter().for_each(|x| {
        let a = loss_function.loss(x.approximate_distance);
        let l = loss_function.loss(x.lower_bound_distance);
        let u = loss_function.loss(x.upper_bound_distance);

        max_possible_loss_errors.push( f64::max((a - u).abs(), (a - l).abs()) );
    });

    let mut indexed_array: Vec<usize> = (0..max_possible_loss_errors.len()).collect();
    indexed_array.sort_by(|x, y| max_possible_loss_errors[*y].partial_cmp(&max_possible_loss_errors[*x]).unwrap());

    indexed_array
}


