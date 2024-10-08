use std::fmt::Debug;

pub mod double_group_queries;
pub mod offset_shape;
pub mod single_group_queries;
pub mod proxima;
pub mod bvh;

pub enum ProximityLossFunction {
    Hinge { threshold: f64 }
}
impl ProximityLossFunction {
    pub fn loss(&self, value: f64) ->  f64 {
        return match self {
            ProximityLossFunction::Hinge { threshold } => {
                if value <= *threshold { *threshold - value } else { 0.0 }
            }
        }
    }
}


pub trait ToProximityValue {
    fn to_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64;
}

pub trait ToIntersectionResult {
    fn to_intersection_result(&self) -> bool;
}

#[derive(Clone, Debug, PartialEq, Eq, Copy)]
pub enum DistanceMode {
    RawDistance,
    AverageDistance
}

#[derive(Clone, Debug)]
pub struct ProximitySimpleOutput<T: Clone + Debug> {
    pub result: T,
    pub num_ground_truth_checks: usize
}