pub mod optimizers;

use apollo_rust_differentiation::{DifferentiableFunctionEngineNalgebraTrait, FunctionEngine};
use apollo_rust_linalg::V;

pub trait OptimizerOutputTrait {
    /// the argument that is a (local) optimizer
    fn x_star(&self) -> &V;

    /// the objective function output at x_star
    fn f_star(&self) -> f64;
}

#[derive(Clone, Debug)]
pub struct SimpleOptimizerOutput {
    pub x_star: V,
    pub f_star: f64,
    pub num_iters: usize
}
impl OptimizerOutputTrait for SimpleOptimizerOutput {
    #[inline(always)]
    fn x_star(&self) -> &V {
        &self.x_star
    }

    #[inline(always)]
    fn f_star(&self) -> f64 {
        self.f_star
    }
}

pub trait IterativeOptimizerTrait {
    type OutputType : OptimizerOutputTrait;

    fn optimize(&self,
                init_condition: &V,
                objective_function: &FunctionEngine,
                equality_constraint: Option<&FunctionEngine>,
                inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType;

    fn optimize_unconstrained(&self, init_condition: &V, objective_function: &FunctionEngine) -> Self::OutputType {
        self.optimize(init_condition, objective_function, None, None)
    }

    fn optimize_equality_constrained(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: &FunctionEngine) -> Self::OutputType {
        self.optimize(init_condition, objective_function, Some(equality_constraint), None)
    }

    fn optimize_inequality_constrained(&self, init_condition: &V,
                                       objective_function: &FunctionEngine,
                                       inequality_constraint: &FunctionEngine) -> Self::OutputType {
        self.optimize(init_condition, objective_function, None, Some(inequality_constraint))
    }

    fn optimize_fully_constrained(&self, init_condition: &V,
                                  objective_function: &FunctionEngine,
                                  equality_constraint: &FunctionEngine,
                                  inequality_constraint: &FunctionEngine) -> Self::OutputType {
        self.optimize(init_condition, objective_function, Some(equality_constraint), Some(inequality_constraint))
    }
}

pub trait LineSearchTrait {
    fn line_search(&self, function: &impl DifferentiableFunctionEngineNalgebraTrait, x_k: &V, dir: &V) -> f64;
}
