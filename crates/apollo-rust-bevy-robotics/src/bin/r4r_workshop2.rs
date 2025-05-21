use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardAD};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::function_engine::FunctionEngine;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization_adtrait::IterativeOptimizerTrait;
use apollo_rust_optimization_adtrait::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization_adtrait::optimizers::bfgs::BFGS;

#[derive(Clone)]
pub struct Foo<A: AD> {
    pub coeff: A
}
impl<A: AD> DifferentiableFunctionTrait<A> for Foo<A> {
    const NAME: &'static str = "Foo";

    fn call(&self, inputs: &[A], _freeze: bool) -> Vec<A> {
        vec![ self.coeff * inputs[0].sin() + inputs[1].cos() ]
    }

    fn num_inputs(&self) -> usize {
        2
    }

    fn num_outputs(&self) -> usize {
        1
    }
}
impl<A: AD> Foo<A> {
    pub fn to_other_ad_type<A2: AD>(&self) -> Foo<A2> {
        Foo {
            coeff: self.coeff.to_other_ad_type::<A2>(),
        }
    }
}

fn main() {
    let f1 = Foo::<f64> { coeff: 2.0 };
    let f2 = f1.to_other_ad_type::<adfn<1>>();

    let f = FunctionEngine::new(f1, f2, ForwardAD::new());

    let o = BFGS::new(BacktrackingLineSearch::default(), None);

    let res = o.optimize_unconstrained(100, &V::new(&[1., 2.]), &f);
    println!("{:?}", res);
}