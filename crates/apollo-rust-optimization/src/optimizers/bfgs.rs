use std::sync::Arc;
use apollo_rust_differentiation::FunctionEngine;
use apollo_rust_linalg::V;
use crate::{IterativeOptimizerTrait, LineSearchTrait, SimpleOptimizerOutput};

pub struct BFGS {
    pub line_search: Arc<dyn LineSearchTrait>
}
impl BFGS {

}
impl IterativeOptimizerTrait for BFGS {
    type OutputType = SimpleOptimizerOutput;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        todo!()
    }
}

pub struct LBFGS {
    pub line_search: Arc<dyn LineSearchTrait>
}
impl LBFGS {

}
impl IterativeOptimizerTrait for LBFGS {
    type OutputType = SimpleOptimizerOutput;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        todo!()
    }
}