use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization::optimizers::gradient_descent::{
    GradientDescent, SimpleGradientDescent,
};
use apollo_rust_optimization::IterativeOptimizerTrait;
use std::sync::Arc;

pub struct Quadratic;
// Simple f(x) = x^2, min at x=0
impl FunctionNalgebraTrait for Quadratic {
    fn call_raw(&self, x: &V) -> V {
        V::new(&[x[0].powi(2)])
    }

    fn input_dim(&self) -> usize {
        1
    }

    fn output_dim(&self) -> usize {
        1
    }
}

#[test]
fn test_simple_gradient_descent() {
    let f = FunctionEngine::new(Quadratic, DerivativeMethodFD::default());
    let optimizer = SimpleGradientDescent::new(0.1);
    let init_condition = V::new(&[10.0]);

    let result = optimizer.optimize_unconstrained(100, &init_condition, &f);

    println!("Simple GD Result: {:?}", result);
    assert!(result.f_star < 1e-4, "Objective should be close to 0");
    assert!(result.x_star[0].abs() < 1e-2, "x should be close to 0");
}

#[test]
fn test_gradient_descent_backtracking() {
    let f = FunctionEngine::new(Quadratic, DerivativeMethodFD::default());
    let line_search = Arc::new(BacktrackingLineSearch::default());
    let optimizer = GradientDescent::new(line_search);
    let init_condition = V::new(&[10.0]);

    let result = optimizer.optimize_unconstrained(100, &init_condition, &f);

    println!("GD Backtracking Result: {:?}", result);
    assert!(result.f_star < 1e-4, "Objective should be close to 0");
    assert!(result.x_star[0].abs() < 1e-2, "x should be close to 0");
}
