use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

#[derive(Clone, Debug)]
pub struct TestFunction {
    pub a: f64,
    pub time: Arc<Mutex<Duration>>
}
impl Default for TestFunction {
    fn default() -> Self {
        Self {
            a: 1.0,
            time: Arc::new(Mutex::new(Default::default())),
        }
    }
}
impl FunctionNalgebraTrait for TestFunction {
    fn call_raw(&self, x: &V) -> V {
        let start = Instant::now();
        let res = V::from_column_slice(&[self.a * x[0].sin()]);
        let duration = start.elapsed();
        *self.time.lock().unwrap() = duration;
        return res;
    }

    fn input_dim(&self) -> usize {
        1
    }

    fn output_dim(&self) -> usize {
        1
    }
}


fn main() {
    let f = Arc::new(Mutex::new(TestFunction::default()));
    let d = DerivativeMethodFD::default();
    let fe = FunctionEngine::new(f.clone(), d);

    let res = fe.call(&V::new(&[2.0]));
    println!("{}", res);

    let time = f.lock().unwrap().time.clone();
    println!("{:?}", time.lock().unwrap());

    f.lock().unwrap().a = 2.0;

    let res = fe.call(&V::new(&[2.0]));
    println!("{}", res);
}