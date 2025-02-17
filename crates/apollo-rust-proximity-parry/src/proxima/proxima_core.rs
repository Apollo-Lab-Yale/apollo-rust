use std::borrow::Cow;
use std::fmt::Debug;
use std::time::{Duration, Instant};
use nalgebra::DMatrix;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::{DistanceMode, ProximityLossFunction, ToIntersectionResult, ToProximityValue};
use crate::double_group_queries::DoubleGroupProximityQueryMode;
use crate::offset_shape::OffsetShape;

pub trait ProximaTrait {
    type CacheType: ProximaCacheTrait<Self::CacheElementType>;
    type CacheElementType: ProximaCacheElementTrait;
    type ExtraArgs : Clone;

    fn get_extra_args(&self, i: usize, j: usize) -> Cow<Self::ExtraArgs>;
    fn get_cache_mut(&mut self) -> &mut Self::CacheType;
    fn get_cache_immut(&self) -> &Self::CacheType;
    fn approximate_distance_and_bounds(cache_element: &Self::CacheElementType, pose_a_k: &ISE3q, pose_b_k: &ISE3q, cutoff_distance: f64, extra_args: &Self::ExtraArgs) -> Option<(f64, f64, f64)>;
    fn f(&self, cache: &Self::CacheType, i: usize, pa: &ISE3q, j: usize, pb: &ISE3q, skips: Option<&DMatrix<bool>>, cutoff_distance: f64) -> Option<ProximaPairwiseOutput> {
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
                Some(ProximaPairwiseOutput {
                    shape_indices: (i, j),
                    distance_mode: DistanceMode::RawDistance,
                    approximate_distance: res.0,
                    lower_bound_distance: res.1,
                    upper_bound_distance: res.2,
                })
            }
        }
    }
    fn get_all_proxima_outputs_for_proximity(&self, poses_a: &Vec<ISE3q>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupProximityQueryMode, skips: Option<&DMatrix<bool>>, average_distances: Option<&DMatrix<f64>>, cutoff_distance: f64) -> Vec<ProximaPairwiseOutput> {
        let mut out = vec![];

        let cache = self.get_cache_immut();
        match query_mode {
            DoubleGroupProximityQueryMode::AllPossiblePairs => {
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
            DoubleGroupProximityQueryMode::SkipSymmetricalPairs => {
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
            DoubleGroupProximityQueryMode::SubsetOfPairs(v) => {
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
    fn get_all_proxima_outputs_for_intersection(&self, poses_a: &Vec<ISE3q>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupProximityQueryMode, skips: Option<&DMatrix<bool>>, cutoff_distance: f64) -> Option<Vec<ProximaPairwiseOutput>> {
        let mut out = vec![];

        let cache = self.get_cache_immut();
        match query_mode {
            DoubleGroupProximityQueryMode::AllPossiblePairs => {
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
            DoubleGroupProximityQueryMode::SkipSymmetricalPairs => {
                for (i, pa) in poses_a.iter().enumerate() {
                    'l: for (j, pb) in poses_b.iter().enumerate() {
                        if i >= j { continue 'l; }
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
            DoubleGroupProximityQueryMode::SubsetOfPairs(v) => {
                for (i,j) in v {
                    let pa = &poses_a[*i];
                    let pb = &poses_b[*j];
                    let output = self.f(cache, *i, pa, *j, pb, skips, cutoff_distance);
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

        Some(out)
    }
    fn proxima_for_proximity(&mut self,
                             budget: &ProximaBudget,
                             group_a: &Vec<OffsetShape>,
                             poses_a: &Vec<ISE3q>,
                             group_b: &Vec<OffsetShape>,
                             poses_b: &Vec<ISE3q>,
                             query_mode: &DoubleGroupProximityQueryMode,
                             loss_function: &ProximityLossFunction,
                             p_norm: f64,
                             cutoff_distance: f64,
                             skips: Option<&DMatrix<bool>>,
                             average_distances: Option<&DMatrix<f64>>,
                             frozen: bool) -> ProximaOutput<f64> {
        let start = Instant::now();
        let mut proxima_outputs = self.get_all_proxima_outputs_for_proximity(poses_a, poses_b, query_mode, skips, average_distances, cutoff_distance);

        if frozen { return ProximaOutput { result: proxima_outputs.to_proximity_value(loss_function, p_norm), ground_truth_checks: vec![] } }

        let sorted_idxs = get_sorted_indices_of_maximum_possible_loss_function_error(&proxima_outputs, &loss_function);

        // let mut num_ground_truth_checks = 0;
        let mut ground_truth_checks = vec![];
        let lower_bound_proximity_value = proxima_outputs.to_lower_bound_proximity_value(&loss_function, p_norm);
        let upper_bound_proximity_value = proxima_outputs.to_upper_bound_proximity_value(&loss_function, p_norm);
        let mut max_possible_error = (lower_bound_proximity_value - upper_bound_proximity_value).abs();

        'l: for k in 0..sorted_idxs.len() {
            match budget {
                ProximaBudget::TimeBudget(b) => {
                    if start.elapsed() > *b { break 'l; }
                }
                ProximaBudget::AccuracyBudget(b) => {
                    if max_possible_error < *b { break 'l; }
                }
            }

            let idx = sorted_idxs[k];
            let proxima_output = &mut proxima_outputs[idx];
            let (i, j) = proxima_output.shape_indices;
            let sa = &group_a[i];
            let sb = &group_b[j];
            let pa = &poses_a[i];
            let pb = &poses_b[j];

            let mut new_distance = self.get_cache_mut().update_element_with_ground_truth(i, j, sa, pa, sb, pb);

            match average_distances {
                None => { }
                Some(average_distances) => {
                    let mut average = average_distances[(i, j)];
                    if average < 0.0 { average = 1.0; }
                    new_distance /= average
                }
            }

            proxima_output.approximate_distance = new_distance;
            proxima_output.lower_bound_distance = new_distance;
            proxima_output.upper_bound_distance = new_distance;

            // num_ground_truth_checks += 1;
            ground_truth_checks.push((i, j));
            let lower_bound_proximity_value = proxima_outputs.to_lower_bound_proximity_value(&loss_function, p_norm);
            let upper_bound_proximity_value = proxima_outputs.to_upper_bound_proximity_value(&loss_function, p_norm);
            max_possible_error = (lower_bound_proximity_value - upper_bound_proximity_value).abs();
        }

        // return (proximity_value, num_ground_truth_checks);
        // println!(" >>>> {:?}", proxima_outputs.to_proximity_value(&loss_function, p_norm));
        return ProximaOutput {
            result: proxima_outputs.to_proximity_value(&loss_function, p_norm),
            ground_truth_checks,
        }
    }
    fn proxima_for_intersection(&mut self,
                                group_a: &Vec<OffsetShape>,
                                poses_a: &Vec<ISE3q>,
                                group_b: &Vec<OffsetShape>,
                                poses_b: &Vec<ISE3q>,
                                query_mode: &DoubleGroupProximityQueryMode,
                                skips: Option<&DMatrix<bool>>,
                                frozen: bool) -> ProximaOutput<bool> {
        let proxima_outputs = self.get_all_proxima_outputs_for_intersection(poses_a, poses_b, query_mode, skips, 0.0);

        return match proxima_outputs {
            None => {
                ProximaOutput {
                    result: true,
                    ground_truth_checks: vec![],
                }
            }
            Some(mut proxima_outputs) => {
                // let loss_function = ProximityLossFunction::Hinge { threshold: 0.0};

                /*
                match average_distances {
                    None => { }
                    Some(average_distances) => {
                        proxima_outputs = proxima_outputs.to_average_distances(average_distances);
                    }
                }
                */

                if frozen {
                    return ProximaOutput {
                        result: proxima_outputs.to_intersection_result(),
                        ground_truth_checks: vec![],
                    }
                }

                // let sorted_idxs = get_sorted_indices_of_maximum_possible_loss_function_error(&proxima_outputs, &loss_function);

                // let mut num_ground_truth_checks = 0;
                let mut ground_truth_checks = vec![];
                // let lower_bound_proximity_value = proxima_outputs.to_lower_bound_proximity_value(&loss_function, p_norm);
                // let upper_bound_proximity_value = proxima_outputs.to_upper_bound_proximity_value(&loss_function, p_norm);
                // let mut max_error = (lower_bound_proximity_value - upper_bound_proximity_value).abs();

                // let mut proximity_value = proxima_outputs.to_proximity_value(loss_function, p_norm);

                'l: for proxima_output in &mut proxima_outputs {
                    if proxima_output.approximate_distance > 0.0 { continue 'l; }
                    // match budget {
                    //     ProximaBudget::TimeBudget(b) => {
                    //         if start.elapsed() > *b { break 'l; }
                    //     }
                    //     ProximaBudget::AccuracyBudget(b) => {
                    //         if max_error < *b { break 'l; }
                    //     }
                    // }

                    // let idx = sorted_idxs[k];
                    // let proxima_output = &mut proxima_outputs[idx];
                    let (i, j) = proxima_output.shape_indices;
                    let sa = &group_a[i];
                    let sb = &group_b[j];
                    let pa = &poses_a[i];
                    let pb = &poses_b[j];

                    let new_distance = self.get_cache_mut().update_element_with_ground_truth(i, j, sa, pa, sb, pb);
                    ground_truth_checks.push((i,j));

                    if new_distance <= 0.0 { return ProximaOutput {
                        result: true,
                        ground_truth_checks,
                    } }

                    proxima_output.approximate_distance = new_distance;
                    proxima_output.lower_bound_distance = new_distance;
                    proxima_output.upper_bound_distance = new_distance;

                    // num_ground_truth_checks += 1;
                    // proximity_value = proxima_outputs.to_proximity_value(loss_function, p_norm);
                    // let lower_bound_proximity_value = proxima_outputs.to_lower_bound_proximity_value(&loss_function, p_norm);
                    // let upper_bound_proximity_value = proxima_outputs.to_upper_bound_proximity_value(&loss_function, p_norm);
                    // max_error = (lower_bound_proximity_value - upper_bound_proximity_value).abs();
                }

                for proxima_output in &proxima_outputs {
                    if proxima_output.approximate_distance <= 0.0 { return ProximaOutput {
                        result: true,
                        ground_truth_checks,
                    } }
                }

                ProximaOutput {
                    result: false,
                    ground_truth_checks,
                }
            }
        }
    }
}

pub trait ProximaCacheTrait<CacheElement: ProximaCacheElementTrait> {
    /*
    fn build_all_elements(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupProximityQueryMode, skips: Option<&DMatrix<bool>>) -> DMatrix<CacheElement> {
        if let DoubleGroupProximityQueryMode::SubsetOfPairs(_) = query_mode {
            panic!("you probably shouldn't use a subset of pairs to initialize the Proxima cache.");
        }
        assert_eq!(group_a.len(), poses_a.len());
        assert_eq!(group_b.len(), poses_b.len());

        let mut elements = DMatrix::from_vec(group_a.len(), group_b.len(), vec![CacheElement::default(); group_a.len()*group_b.len()]);

        for (i, (shape_a, pose_a)) in group_a.iter().zip(poses_a.iter()).enumerate() {
            'l: for (j, (shape_b, pose_b)) in group_b.iter().zip(poses_b.iter()).enumerate() {
                match skips {
                    None => {}
                    Some(skips) => {
                        if skips[(i,j)] { continue 'l; }
                    }
                }
                elements[(i,j)].update_element_with_ground_truth(shape_a, pose_a, shape_b, pose_b);
            }
        }

        elements
    }
    fn new_from_elements(elements: DMatrix<CacheElement>) -> Self;
    */
    fn elements(&self) -> &DMatrix<CacheElement>;
    fn elements_mut(&mut self) -> &mut DMatrix<CacheElement>;
    fn update_element_with_ground_truth(&mut self, i: usize, j: usize, shape_a: &OffsetShape, pose_a: &ISE3q, shape_b: &OffsetShape, pose_b: &ISE3q) -> f64 {
        let element = &mut self.elements_mut()[(i,j)];
        element.update_element_with_ground_truth(shape_a, pose_a, shape_b, pose_b)
    }
}

pub trait ProximaCacheElementTrait: Clone + Debug {
    fn update_element_with_ground_truth(&mut self, shape_a: &OffsetShape, pose_a: &ISE3q, shape_b: &OffsetShape, pose_b: &ISE3q) -> f64;
}

#[derive(Clone, Debug)]
pub enum ProximaBudget {
    TimeBudget(Duration),
    AccuracyBudget(f64)
}

#[derive(Clone, Debug)]
pub struct ProximaPairwiseOutput {
    pub shape_indices: (usize, usize),
    pub distance_mode: DistanceMode,
    pub approximate_distance: f64,
    pub lower_bound_distance: f64,
    pub upper_bound_distance: f64
}

#[derive(Clone, Debug)]
pub struct ProximaOutput<T: Clone + Debug> {
    pub result: T,
    pub ground_truth_checks: Vec<(usize, usize)>
}

pub trait ToAverageDistancesProximaOutput {
    fn to_average_distances(&self, average_distance: &DMatrix<f64>) -> Vec<ProximaPairwiseOutput>;
}
impl ToAverageDistancesProximaOutput for Vec<ProximaPairwiseOutput> {
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Vec<ProximaPairwiseOutput> {
        let mut out = vec![];

        self.iter().for_each(|x| {
            let mut average = average_distances[(x.shape_indices.0, x.shape_indices.1)];
            if average < 0.0 { average = 1.0; }

            out.push(ProximaPairwiseOutput {
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

impl ToProximityValue for Vec<ProximaPairwiseOutput> {
    fn to_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64 {
        let mut out = 0.0;

        self.iter().for_each(|x| out += loss.loss(x.approximate_distance).powf(p_norm));

        out.powf(1.0/p_norm)
    }
}
impl ToIntersectionResult for Vec<ProximaPairwiseOutput> {
    fn to_intersection_result(&self) -> bool {
        for x in self.iter() {
            if x.approximate_distance <= 0.0 { return true; }
        }

        return false;
    }
}

pub trait ToUpperAndLowerBoundProximityValue {
    fn to_upper_bound_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64;

    fn to_lower_bound_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64;
}
impl ToUpperAndLowerBoundProximityValue for Vec<ProximaPairwiseOutput> {
    fn to_upper_bound_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64 {
        let mut out = 0.0;

        self.iter().for_each(|x| out += loss.loss(x.upper_bound_distance).powf(p_norm));

        out.powf(1.0/p_norm)
    }

    fn to_lower_bound_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64 {
        let mut out = 0.0;

        self.iter().for_each(|x| out += loss.loss(x.lower_bound_distance).powf(p_norm));

        out.powf(1.0/p_norm)
    }
}

pub fn get_sorted_indices_of_maximum_possible_loss_function_error(proxima_outputs: &Vec<ProximaPairwiseOutput>,
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


