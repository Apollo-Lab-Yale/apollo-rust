pub mod optimizers;
pub mod line_searches;

use std::ops::Deref;
use std::sync::{Arc, Mutex, RwLock};
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use ad_trait::function_engine::FunctionEngine;
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

    fn optimize_raw<F1O, F2O, EO, F1E, F2E, EE, F1I, F2I, EI>(&self,
                    max_iterations: usize,
                    init_condition: &V,
                    objective_function: &FunctionEngine<F1O, F2O, EO>,
                    equality_constraint: Option<&FunctionEngine<F1E, F2E, EE>>,
                    inequality_constraint: Option<&FunctionEngine<F1I, F2I, EI>>) -> Self::OutputType
        where F1O: DifferentiableFunctionTrait<f64>,
              F2O: DifferentiableFunctionTrait<EO::T>,
              EO: DerivativeMethodTrait,
              F1E: DifferentiableFunctionTrait<f64>,
              F2E: DifferentiableFunctionTrait<EE::T>,
              EE: DerivativeMethodTrait,
              F1I: DifferentiableFunctionTrait<f64>,
              F2I: DifferentiableFunctionTrait<EI::T>,
              EI: DerivativeMethodTrait,
    ;

    fn optimize<F1O, F2O, EO, F1E, F2E, EE, F1I, F2I, EI>(&self,
                max_iterations: usize,
                init_condition: &V,
                objective_function: &FunctionEngine<F1O, F2O, EO>,
                equality_constraint: Option<&FunctionEngine<F1E, F2E, EE>>,
                inequality_constraint: Option<&FunctionEngine<F1I, F2I, EI>>) -> Self::OutputType
        where F1O: DifferentiableFunctionTrait<f64>,
          F2O: DifferentiableFunctionTrait<EO::T>,
          EO: DerivativeMethodTrait,
          F1E: DifferentiableFunctionTrait<f64>,
          F2E: DifferentiableFunctionTrait<EE::T>,
          EE: DerivativeMethodTrait,
          F1I: DifferentiableFunctionTrait<f64>,
          F2I: DifferentiableFunctionTrait<EI::T>,
          EI: DerivativeMethodTrait,
    {
        assert_eq!(objective_function.num_outputs(), 1);
        self.optimize_raw(max_iterations, init_condition, objective_function, equality_constraint, inequality_constraint)
    }

    fn optimize_unconstrained<F1O, F2O, EO>(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine<F1O, F2O, EO>) -> Self::OutputType
        where F1O: DifferentiableFunctionTrait<f64>,
              F2O: DifferentiableFunctionTrait<EO::T>,
              EO: DerivativeMethodTrait,
    {
        self.optimize::<F1O, F2O, EO, (), (), (), (), (), ()>(max_iterations, init_condition, objective_function, None, None)
    }

    fn optimize_equality_constrained<F1O, F2O, EO, F1E, F2E, EE>(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine<F1O, F2O, EO>, equality_constraint: &FunctionEngine<F1E, F2E, EE>) -> Self::OutputType
        where
            F1O: DifferentiableFunctionTrait<f64>,
            F2O: DifferentiableFunctionTrait<EO::T>,
            EO: DerivativeMethodTrait,
            F1E: DifferentiableFunctionTrait<f64>,
            F2E: DifferentiableFunctionTrait<EE::T>,
            EE: DerivativeMethodTrait,
    {
        self.optimize::<F1O, F2O, EO, F1E, F2E, EE, (), (), ()>(max_iterations, init_condition, objective_function, Some(equality_constraint), None)
    }

    fn optimize_inequality_constrained<F1O, F2O, EO, F1I, F2I, EI>(&self, max_iterations: usize, init_condition: &V,
                                       objective_function: &FunctionEngine<F1O, F2O, EO>,
                                       inequality_constraint: &FunctionEngine<F1I, F2I, EI>) -> Self::OutputType
        where
            F1O: DifferentiableFunctionTrait<f64>,
            F2O: DifferentiableFunctionTrait<EO::T>,
            EO: DerivativeMethodTrait,
            F1I: DifferentiableFunctionTrait<f64>,
            F2I: DifferentiableFunctionTrait<EI::T>,
            EI: DerivativeMethodTrait,
    {
        self.optimize::<F1O, F2O, EO, (), (), (), F1I, F2I, EI>(max_iterations, init_condition, objective_function, None, Some(inequality_constraint))
    }
}

impl<T: IterativeOptimizerTrait> IterativeOptimizerTrait for Arc<T> {
    type OutputType = T::OutputType;

    fn optimize_raw<F1O, F2O, EO, F1E, F2E, EE, F1I, F2I, EI>(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine<F1O, F2O, EO>, equality_constraint: Option<&FunctionEngine<F1E, F2E, EE>>, inequality_constraint: Option<&FunctionEngine<F1I, F2I, EI>>) -> Self::OutputType
        where F1O: DifferentiableFunctionTrait<f64>,
              F2O: DifferentiableFunctionTrait<EO::T>,
              EO: DerivativeMethodTrait,
              F1E: DifferentiableFunctionTrait<f64>,
              F2E: DifferentiableFunctionTrait<EE::T>,
              EE: DerivativeMethodTrait,
              F1I: DifferentiableFunctionTrait<f64>,
              F2I: DifferentiableFunctionTrait<EI::T>,
              EI: DerivativeMethodTrait,
    {
        self.deref().optimize(max_iterations, init_condition, objective_function, equality_constraint, inequality_constraint)
    }
}
impl<T: IterativeOptimizerTrait> IterativeOptimizerTrait for RwLock<T> {
    type OutputType = T::OutputType;

    fn optimize_raw<F1O, F2O, EO, F1E, F2E, EE, F1I, F2I, EI>(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine<F1O, F2O, EO>, equality_constraint: Option<&FunctionEngine<F1E, F2E, EE>>, inequality_constraint: Option<&FunctionEngine<F1I, F2I, EI>>) -> Self::OutputType
        where F1O: DifferentiableFunctionTrait<f64>,
              F2O: DifferentiableFunctionTrait<EO::T>,
              EO: DerivativeMethodTrait,
              F1E: DifferentiableFunctionTrait<f64>,
              F2E: DifferentiableFunctionTrait<EE::T>,
              EE: DerivativeMethodTrait,
              F1I: DifferentiableFunctionTrait<f64>,
              F2I: DifferentiableFunctionTrait<EI::T>,
              EI: DerivativeMethodTrait,
    {
        let tmp = self.read().unwrap();
        tmp.optimize(max_iterations, init_condition, objective_function, equality_constraint, inequality_constraint)
    }
}
impl<T: IterativeOptimizerTrait> IterativeOptimizerTrait for Mutex<T> {
    type OutputType = T::OutputType;

    fn optimize_raw<F1O, F2O, EO, F1E, F2E, EE, F1I, F2I, EI>(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine<F1O, F2O, EO>, equality_constraint: Option<&FunctionEngine<F1E, F2E, EE>>, inequality_constraint: Option<&FunctionEngine<F1I, F2I, EI>>) -> Self::OutputType
        where F1O: DifferentiableFunctionTrait<f64>,
              F2O: DifferentiableFunctionTrait<EO::T>,
              EO: DerivativeMethodTrait,
              F1E: DifferentiableFunctionTrait<f64>,
              F2E: DifferentiableFunctionTrait<EE::T>,
              EE: DerivativeMethodTrait,
              F1I: DifferentiableFunctionTrait<f64>,
              F2I: DifferentiableFunctionTrait<EI::T>,
              EI: DerivativeMethodTrait,
    {
        let tmp = self.lock().unwrap();
        tmp.optimize(max_iterations, init_condition, objective_function, equality_constraint, inequality_constraint)
    }
}

pub trait LineSearchTrait {
    fn line_search<F1, F2, E>(&self, function: &FunctionEngine<F1, F2, E>, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64
        where F1: DifferentiableFunctionTrait<f64>,
              F2: DifferentiableFunctionTrait<E::T>,
              E: DerivativeMethodTrait
    ;
}
impl<T: LineSearchTrait> LineSearchTrait for Arc<T> {
    fn line_search<F1, F2, E>(&self, function: &FunctionEngine<F1, F2, E>, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64
        where F1: DifferentiableFunctionTrait<f64>,
              F2: DifferentiableFunctionTrait<E::T>,
              E: DerivativeMethodTrait
    {
        self.deref().line_search(function, x_k, f_k, dir, g_k)
    }
}
impl<T: LineSearchTrait> LineSearchTrait for RwLock<T> {
    fn line_search<F1, F2, E>(&self, function: &FunctionEngine<F1, F2, E>, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64
        where F1: DifferentiableFunctionTrait<f64>,
              F2: DifferentiableFunctionTrait<E::T>,
              E: DerivativeMethodTrait
    {
        let tmp = self.read().unwrap();
        tmp.line_search(function, x_k, f_k, dir, g_k)
    }
}
impl<T: LineSearchTrait> LineSearchTrait for Mutex<T> {
    fn line_search<F1, F2, E>(&self, function: &FunctionEngine<F1, F2, E>, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64
        where F1: DifferentiableFunctionTrait<f64>,
              F2: DifferentiableFunctionTrait<E::T>,
              E: DerivativeMethodTrait
    {
        let tmp = self.lock().unwrap();
        tmp.line_search(function, x_k, f_k, dir, g_k)
    }
}