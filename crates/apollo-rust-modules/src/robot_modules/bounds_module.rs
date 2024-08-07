use rand::Rng;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloBoundsModule {
    pub bounds: Vec<(f64, f64)>,
    pub dof_lower_bounds: Vec<f64>,
    pub dof_upper_bounds: Vec<f64>
}

impl ApolloBoundsModule {
    pub fn sample_random_state(&self) -> Vec<f64> {
        let mut rng = rand::thread_rng();
        self.bounds.iter().map(|&(lower, upper)| rng.gen_range(lower..upper)).collect()
    }
}