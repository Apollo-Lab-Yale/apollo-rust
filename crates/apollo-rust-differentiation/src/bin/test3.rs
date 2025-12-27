use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use std::sync::{Arc, RwLock};

pub struct Test {
    pub a: f64,
}
impl FunctionNalgebraTrait for Test {
    fn call_raw(&self, x: &V) -> V {
        V::new(&[x[0] * self.a])
    }

    fn input_dim(&self) -> usize {
        1
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let t = Test { a: 1.0 };
    let handle = Arc::new(RwLock::new(t));

    let d = FunctionEngine::new(handle.clone(), DerivativeMethodFD::new(0.0000001));

    println!("{}", d.derivative(&V::new(&[1.0])).1);

    let mut tmp = handle.write().unwrap();
    tmp.a = 5.0;
    drop(tmp);

    println!("{}", d.derivative(&V::new(&[1.0])).1);
}
