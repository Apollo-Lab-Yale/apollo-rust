
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_linalg::{ApolloDVectorTrait, V};

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


}