use std::sync::RwLock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use ad_trait::function_engine::FunctionEngine;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use crate::{IterativeOptimizerTrait, SimpleOptimizerOutput};

pub struct OpENUnconstrained {
    n: usize,
    panoc_cache: RwLock<PANOCCache>,
    lower_bounds: Vec<f64>,
    upper_bounds: Vec<f64>
}
impl OpENUnconstrained {
    pub fn new(n: usize, lower_bounds: Vec<f64>, upper_bounds: Vec<f64>) -> Self {
        assert_eq!(lower_bounds.len(), n);
        assert_eq!(upper_bounds.len(), n);

        Self {
            n,
            panoc_cache: RwLock::new(PANOCCache::new(n, 1e-6, 10)),
            lower_bounds,
            upper_bounds
        }
    }
}
impl IterativeOptimizerTrait for OpENUnconstrained {
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

        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let res = objective_function.derivative(u);
            let grad_as_slice = res.1.as_slice();
            assert_eq!(grad_as_slice.len(), grad.len());
            grad.iter_mut().zip(grad_as_slice.iter()).for_each(|(x, y)| *x = *y );
            Ok(())
        };

        let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            let res= objective_function.call(u);
            assert_eq!(res.len(), 1);
            *cost = res[0];
            Ok(())
        };

        let binding = constraints::Rectangle::new(Some(self.lower_bounds.as_slice()), Some(self.upper_bounds.as_slice()));
        let problem = Problem::new(&binding, df, f);
        let mut binding = self.panoc_cache.write();
        let cache = binding.as_mut().unwrap();
        let mut panoc = PANOCOptimizer::new(problem, cache);
        panoc = panoc.with_max_iter(max_iterations);

        let mut x = init_condition + V::new_random_with_range(self.n, -0.00000001, 0.00000001);
        let solver_status = panoc.solve(x.as_mut_slice()).expect("error");

        SimpleOptimizerOutput {
            x_star: x.clone(),
            f_star: solver_status.cost_value(),
            num_iters: solver_status.iterations(),
        }
    }
}