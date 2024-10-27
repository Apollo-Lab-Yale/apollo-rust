mod optimizers;

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

/*
pub trait GradientBasedOptimizerTrait {
    type OutputType : OptimizerOutputTrait;

    fn optimize(&self,
                objective_function: &impl DifferentiableFunctionEngineNalgebraTrait,
                equality_constraint: Option<&impl DifferentiableFunctionEngineNalgebraTrait>,
                inequality_constraint: Option<&impl DifferentiableFunctionEngineNalgebraTrait>) -> Self::OutputType;

    fn optimize_unconstrained(&self, objective_function: &impl DifferentiableFunctionEngineNalgebraTrait) -> Self::OutputType {
        self.optimize(objective_function, None::<&DummyDifferentiableFunction>, None::<&DummyDifferentiableFunction>)
    }

    fn optimize_equality_constrained(&self, objective_function: &impl DifferentiableFunctionEngineNalgebraTrait,
                                     equality_constraint: &impl DifferentiableFunctionEngineNalgebraTrait) -> Self::OutputType {
        self.optimize(objective_function, Some(equality_constraint), None::<&DummyDifferentiableFunction>)
    }

    fn optimize_inequality_constrained(&self, objective_function: &impl DifferentiableFunctionEngineNalgebraTrait,
                                       inequality_constraint: &impl DifferentiableFunctionEngineNalgebraTrait) -> Self::OutputType {
        self.optimize(objective_function, None::<&DummyDifferentiableFunction>, Some(inequality_constraint))
    }

    fn optimize_fully_constrained(&self, objective_function: &impl DifferentiableFunctionEngineNalgebraTrait,
                                  equality_constraint: &impl DifferentiableFunctionEngineNalgebraTrait,
                                  inequality_constraint: &impl DifferentiableFunctionEngineNalgebraTrait) -> Self::OutputType {
        self.optimize(objective_function, Some(equality_constraint), Some(inequality_constraint))
    }
}
*/

pub trait OptimizerTrait {
    type OutputType : OptimizerOutputTrait;

    fn optimize(&self) -> Self::OutputType;

    fn objective_function(&self) -> &FunctionEngine;

    fn equality_constraint_function(&self) -> Option<&FunctionEngine>;

    fn inequality_constraint_function(&self) -> Option<&FunctionEngine>;
}

pub trait LineSearchTrait {
    fn line_search(&self, function: &impl DifferentiableFunctionEngineNalgebraTrait, x_k: &V, dir: &V) -> f64;
}
