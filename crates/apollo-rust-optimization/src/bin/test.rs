use std::sync::{Arc, RwLock};
use apollo_rust_differentiation::{DifferentiableFunctionEngineNalgebraTrait, FunctionNalgebraConversionTrait, FunctionNalgebraTrait};
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
    let handle = Arc::new(RwLock::new(Test));
    let res = handle.clone().to_wrapper_differentiable_function();
    println!("{:?}", res.output_dim());
}