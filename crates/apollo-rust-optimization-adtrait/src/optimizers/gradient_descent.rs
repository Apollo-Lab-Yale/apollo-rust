
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use ad_trait::function_engine::FunctionEngine;
use apollo_rust_linalg::V;
use crate::{IterativeOptimizerTrait, LineSearchTrait, SimpleOptimizerOutput};

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

    fn optimize_raw<F1O, F2O, EO, F1E, F2E, EE, F1I, F2I, EI>(&self,
                                                              max_iterations: usize,
                                                              init_condition: &V,
                                                              objective_function: &FunctionEngine<F1O, F2O, EO>,
                                                              equality_constraint: Option<&FunctionEngine<F1E, F2E, EE>>,
                                                              inequality_constraint: Option<&FunctionEngine<F1I, F2I, EI>>) -> Self::OutputType
        where
            F1O: DifferentiableFunctionTrait<f64>,
            F2O: DifferentiableFunctionTrait<EO::T>,
            EO: DerivativeMethodTrait,
            F1E: DifferentiableFunctionTrait<f64>,
            F2E: DifferentiableFunctionTrait<EE::T>,
            EE: DerivativeMethodTrait,
            F1I: DifferentiableFunctionTrait<f64>,
            F2I: DifferentiableFunctionTrait<EI::T>,
            EI: DerivativeMethodTrait,
    {
        assert!(equality_constraint.is_none());
        assert!(inequality_constraint.is_none());

        let mut x_k = init_condition.clone();
        let mut num_iters = 0;

        loop {
            let (f_k, g_k) = objective_function.derivative(x_k.as_slice());
            let norm = g_k.norm();
            if num_iters>=max_iterations || norm < 0.01 {
                return SimpleOptimizerOutput {
                    x_star: x_k,
                    f_star: f_k[0],
                    num_iters,
                }
            }
            let g_k = V::from_column_slice(g_k.as_slice());
            x_k = &x_k - self.lambda*&g_k;
            num_iters += 1;
        }
    }
}

pub struct GradientDescent<L: LineSearchTrait> {
    pub line_search: L
}
impl<L: LineSearchTrait> GradientDescent<L> {
    pub fn new(line_search: L) -> Self {
        Self {
            line_search,
        }
    }
}
impl<L: LineSearchTrait> IterativeOptimizerTrait for GradientDescent<L> {
    type OutputType = SimpleOptimizerOutput;

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
    {
        assert!(equality_constraint.is_none());
        assert!(inequality_constraint.is_none());

        let mut x_k = init_condition.clone();
        let mut num_iters = 0;

        loop {
            let (f_k, g_k) = objective_function.derivative(x_k.as_slice());
            let norm = g_k.norm();
            if num_iters>=max_iterations || norm < 0.01 {
                return SimpleOptimizerOutput {
                    x_star: x_k,
                    f_star: f_k[0],
                    num_iters,
                }
            }
            let g_k = V::from_column_slice(g_k.as_slice());
            let lambda = self.line_search.line_search(objective_function, &x_k, f_k[0], &-&g_k, &g_k);
            x_k = &x_k - lambda*&g_k;

            num_iters += 1;
        }
    }
}