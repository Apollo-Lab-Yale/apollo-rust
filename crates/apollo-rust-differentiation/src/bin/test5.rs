use std::sync::Arc;
use apollo_rust_differentiation::derivative_methods::{DerivativeMethodFD, DerivativeMethodWASP};
use apollo_rust_differentiation::{DerivativeMethodNalgebraTrait, FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

pub struct Test;
impl FunctionNalgebraTrait for Test {
    fn call_raw(&self, x: &V) -> V {
        return V::new(&[x[0].sin()])
    }

    fn input_dim(&self) -> usize {
        1
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let f = Arc::new(Test);

    let d = Arc::new(DerivativeMethodWASP::new_default(1, 1));
    let fe = FunctionEngine::new(f.clone(), d.clone());
    println!("{:?}", fe.derivative(&V::new(&[1.0])));
    println!("{:?}", d.num_f_calls());
    println!("{:?}", fe.derivative(&V::new(&[1.0])));
    println!("{:?}", d.num_f_calls());
    println!("{:?}", fe.derivative(&V::new(&[1.1])));
    println!("{:?}", d.num_f_calls());

    let fe = FunctionEngine::new(f.clone(), DerivativeMethodFD::default());
    println!("{:?}", fe.derivative(&V::new(&[1.1])));
}