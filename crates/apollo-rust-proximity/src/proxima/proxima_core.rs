use std::time::Duration;
use nalgebra::DMatrix;
use crate::{DistanceMode, ProximityLossFunction, ToProximityValue};

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


