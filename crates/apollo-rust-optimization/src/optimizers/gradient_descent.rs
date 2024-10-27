use apollo_rust_differentiation::FunctionEngine;
use apollo_rust_linalg::V;
use crate::{OptimizerTrait, SimpleOptimizerOutput};

#[derive(Clone)]
pub struct OptimizerSimpleGradientDescent {
    pub objective_function: FunctionEngine,
    pub init_condition: V,
    pub lambda: f64
}
impl OptimizerSimpleGradientDescent {
    pub fn new(objective_function: FunctionEngine, init_condition: V, lambda: f64) -> Self {
        Self { objective_function, init_condition, lambda }
    }
}
impl OptimizerTrait for OptimizerSimpleGradientDescent {
    type OutputType = SimpleOptimizerOutput;

    fn optimize(&self) -> Self::OutputType {
        let mut x_k = self.init_condition.clone();
        let mut f_k_1 = self.objective_function.call(&x_k);
        let mut num_iters = 0;

        loop {
            let grad = self.objective_function.derivative(&x_k);
            let norm = grad.norm();
            if norm < 0.01 {
                todo!()
            }
            num_iters += 1;
        }
    }

    fn objective_function(&self) -> &FunctionEngine {
        &self.objective_function
    }

    fn equality_constraint_function(&self) -> Option<&FunctionEngine> {
        None
    }

    fn inequality_constraint_function(&self) -> Option<&FunctionEngine> {
        None
    }
}
