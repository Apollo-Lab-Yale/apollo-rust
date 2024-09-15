use rand::Rng;
use serde::{Deserialize, Serialize};

/// Struct representing a module for defining and handling bounds on degrees of freedom (DOF).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloBoundsModule {
    /// A vector of tuples representing the lower and upper bounds for each degree of freedom (DOF).
    pub bounds: Vec<(f64, f64)>,

    /// A vector representing the lower bounds for each DOF.
    pub dof_lower_bounds: Vec<f64>,

    /// A vector representing the upper bounds for each DOF.
    pub dof_upper_bounds: Vec<f64>,
}

impl ApolloBoundsModule {
    /// Samples a random state within the specified bounds.
    ///
    /// For each DOF, a random value is generated between the corresponding lower and upper bounds.
    ///
    /// Returns a vector of sampled values for each DOF.
    pub fn sample_random_state(&self) -> Vec<f64> {
        let mut rng = rand::thread_rng();
        self.bounds.iter().map(|&(lower, upper)| rng.gen_range(lower..upper)).collect()
    }
}
