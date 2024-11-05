pub mod optimizers;
pub mod line_searches;

use std::ops::Deref;
use std::sync::{Arc, Mutex, RwLock};
use apollo_rust_differentiation::{FunctionEngine};
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

    fn optimize_raw(&self,
                    init_condition: &V,
                    objective_function: &FunctionEngine,
                    equality_constraint: Option<&FunctionEngine>,
                    inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType;

    fn optimize(&self,
                init_condition: &V,
                objective_function: &FunctionEngine,
                equality_constraint: Option<&FunctionEngine>,
                inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        assert_eq!(objective_function.output_dim(), 1);
        self.optimize_raw(init_condition, objective_function, equality_constraint, inequality_constraint)
    }

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
impl<T: IterativeOptimizerTrait> IterativeOptimizerTrait for Arc<T> {
    type OutputType = T::OutputType;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        self.deref().optimize(init_condition, objective_function, equality_constraint, inequality_constraint)
    }
}
impl<T: IterativeOptimizerTrait> IterativeOptimizerTrait for RwLock<T> {
    type OutputType = T::OutputType;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        let tmp = self.read().unwrap();
        tmp.optimize(init_condition, objective_function, equality_constraint, inequality_constraint)
    }
}
impl<T: IterativeOptimizerTrait> IterativeOptimizerTrait for Mutex<T> {
    type OutputType = T::OutputType;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        let tmp = self.lock().unwrap();
        tmp.optimize(init_condition, objective_function, equality_constraint, inequality_constraint)
    }
}

pub trait LineSearchTrait {
    fn line_search(&self, function: &FunctionEngine, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64;
}
impl<T: LineSearchTrait> LineSearchTrait for Arc<T> {
    fn line_search(&self, function: &FunctionEngine, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64 {
        self.deref().line_search(function, x_k, f_k, dir, g_k)
    }
}
impl<T: LineSearchTrait> LineSearchTrait for RwLock<T> {
    fn line_search(&self, function: &FunctionEngine, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64 {
        let tmp = self.read().unwrap();
        tmp.line_search(function, x_k, f_k, dir, g_k)
    }
}
impl<T: LineSearchTrait> LineSearchTrait for Mutex<T> {
    fn line_search(&self, function: &FunctionEngine, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64 {
        let tmp = self.lock().unwrap();
        tmp.line_search(function, x_k, f_k, dir, g_k)
    }
}
