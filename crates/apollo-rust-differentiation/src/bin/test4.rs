use std::sync::{Arc, RwLock};
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_linalg::{ApolloDVectorTrait, V};

pub struct Test {
    pub a: f64
}
impl FunctionNalgebraTrait for Test {
    fn call_raw(&self, x: &V) -> V {
        V::new(&[x[0]* self.a])
    }

    fn input_dim(&self) -> usize {
        1
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let handle = Arc::new(RwLock::new(Test { a: 2.0 }));
    let f = FunctionEngine::new(handle.clone(), DerivativeMethodFD::default());

    println!("{}", f.call(&V::new(&[1.0])));

    let mut tmp = handle.write().unwrap();
    tmp.a = 3.0;
    drop(tmp);

    println!("{}", f.call(&V::new(&[1.0])));
}