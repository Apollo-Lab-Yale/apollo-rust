use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardAD};
use ad_trait::function_engine::FunctionEngine;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization_adtrait::IterativeOptimizerTrait;
use apollo_rust_optimization_adtrait::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization_adtrait::optimizers::gradient_descent::{GradientDescent};

#[derive(Clone)]
pub struct Foo;
impl<A: AD> DifferentiableFunctionTrait<A> for Foo {
    const NAME: &'static str = "Foo";

    fn call(&self, inputs: &[A], _freeze: bool) -> Vec<A> {
        vec![ inputs[0].sin() + inputs[1].cos() ]
    }

    fn num_inputs(&self) -> usize {
        2
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

fn main() {
    let f = FunctionEngine::new(Foo, Foo, ForwardAD::new());

    let o = GradientDescent::new(BacktrackingLineSearch::default());

    let res = o.optimize_unconstrained(100, &V::new(&[1., 2.]), &f);
    println!("{:?}", res);
}