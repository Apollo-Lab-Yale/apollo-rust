use rand::Rng;
use serde::{Deserialize, Serialize};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloBoundsModule {
    pub bounds: Vec<(f64, f64)>,
    pub dof_lower_bounds: Vec<f64>,
    pub dof_upper_bounds: Vec<f64>
}
impl ApolloBoundsModule {
    pub fn sample_random_state(&self) -> V {
        let mut rng = rand::thread_rng();
        V::new(&self.bounds.iter().map(|&(lower, upper)| rng.gen_range(lower..upper)).collect::<Vec<f64>>())
    }
}