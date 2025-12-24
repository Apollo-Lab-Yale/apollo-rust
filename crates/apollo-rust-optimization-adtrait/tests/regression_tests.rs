use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardADMulti};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::function_engine::FunctionEngine;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization_adtrait::IterativeOptimizerTrait;
use apollo_rust_optimization_adtrait::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization_adtrait::optimizers::gradient_descent::GradientDescent;
use apollo_rust_optimization_adtrait::optimizers::open::OpENUnconstrained;
use std::sync::Arc;

#[derive(Clone)]
pub struct QuadraticAD;

impl<T: AD> DifferentiableFunctionTrait<T> for QuadraticAD {
    const NAME: &'static str = "QuadraticAD";

    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
        vec![inputs[0].powi(2)]
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

#[test]
fn test_open_optimizer_ad() {
    let f = FunctionEngine::new(QuadraticAD, QuadraticAD, ForwardADMulti::<adfn<1>>::new());
    // OpENUnconstrained::new(dim, lower_bounds, upper_bounds)
    let optimizer = OpENUnconstrained::new(1, vec![-100.0], vec![100.0]);
    let init_condition = V::new(&[10.0]);

    let result = optimizer.optimize_unconstrained(100, &init_condition, &f);

    println!("OpEN AD Result: {:?}", result);
    assert!(result.f_star < 1e-4, "Objective should be close to 0");
    assert!(result.x_star[0].abs() < 1e-2, "x should be close to 0");
}

#[test]
fn test_gradient_descent_ad() {
    let f = FunctionEngine::new(QuadraticAD, QuadraticAD, ForwardADMulti::<adfn<1>>::new());
    let line_search = Arc::new(BacktrackingLineSearch::default());
    let optimizer = GradientDescent::new(line_search);
    let init_condition = V::new(&[10.0]);

    let result = optimizer.optimize_unconstrained(100, &init_condition, &f);

    println!("Gradient Descent AD Result: {:?}", result);
    assert!(result.f_star < 1e-4, "Objective should be close to 0");
    assert!(result.x_star[0].abs() < 1e-2, "x should be close to 0");
}
