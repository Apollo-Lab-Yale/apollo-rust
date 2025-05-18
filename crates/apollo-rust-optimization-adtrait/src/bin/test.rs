use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, FiniteDifferencing, ForwardAD, ForwardADMulti, ReverseAD};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::function_engine::FunctionEngine;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization_adtrait::IterativeOptimizerTrait;
use apollo_rust_optimization_adtrait::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization_adtrait::optimizers::bfgs::{BFGS, LBFGS};
use apollo_rust_optimization_adtrait::optimizers::gradient_descent::GradientDescent;
use apollo_rust_optimization_adtrait::optimizers::open::OpENUnconstrained;

#[derive(Clone)]
pub struct Foo;
impl<T: AD> DifferentiableFunctionTrait<T> for Foo {
    const NAME: &'static str = "Foo";

    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
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
    let f = FunctionEngine::new(Foo, Foo, ForwardADMulti::<adfn<2>>::new());

    let o = OpENUnconstrained::new(2, vec![-100.0, -100.0], vec![100.0, 100.0]);

    let res = o.optimize_unconstrained(100, &V::new(&[1., 2.]), &f);

    println!("{:?}", res);
}