
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization::IterativeOptimizerTrait;
use apollo_rust_optimization::optimizers::gradient_descent::SimpleGradientDescent;
use apollo_rust_optimization::optimizers::open::OpENUnconstrained;

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

    let o = SimpleGradientDescent::new(1.0);

    let res = o.optimize_unconstrained(&V::new(&[3.0]), &f);
    println!("{:?}", res);

    let open = OpENUnconstrained::new(1, 10, vec![-100.0], vec![100.0]);
    let res = open.optimize_unconstrained(&V::new(&[3.0]), &f);

    println!("{:?}", res);
}