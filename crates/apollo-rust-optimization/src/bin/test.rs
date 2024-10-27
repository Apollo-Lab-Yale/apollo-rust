
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization::IterativeOptimizerTrait;
use apollo_rust_optimization::optimizers::gradient_descent::SimpleGradientDescent;

pub struct Test;
impl FunctionNalgebraTrait for Test {
    fn call_raw(&self, x: &V) -> V {
        V::new(&[x[0].sin()])
    }

    fn input_dim(&self) -> usize {
        1
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let f = FunctionEngine::new(Test, DerivativeMethodFD::default());

    let o = SimpleGradientDescent::new(0.1);

    let res = o.optimize_unconstrained(&V::new(&[1.0]), &f);
    println!("{:?}", res);
}