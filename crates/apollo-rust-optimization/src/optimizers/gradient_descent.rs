use apollo_rust_differentiation::FunctionEngine;
use apollo_rust_linalg::V;
use crate::{IterativeOptimizerTrait, SimpleOptimizerOutput};

#[derive(Clone)]
pub struct SimpleGradientDescent {
    pub lambda: f64
}
impl SimpleGradientDescent {
    pub fn new(lambda: f64) -> Self {
        Self { lambda }
    }
}
impl IterativeOptimizerTrait for SimpleGradientDescent {
    type OutputType = SimpleOptimizerOutput;

    fn optimize(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        assert!(equality_constraint.is_none());
        assert!(inequality_constraint.is_none());

        let mut x_k = init_condition.clone();
        let mut num_iters = 0;

        loop {
            let (f_k, g_k) = objective_function.derivative(&x_k);
            let norm = g_k.norm();
            if norm < 0.01 {
                return SimpleOptimizerOutput {
                    x_star: x_k,
                    f_star: f_k[0],
                    num_iters,
                }
            }
            x_k = &x_k - self.lambda*&g_k;

            num_iters += 1;
        }
    }
}
