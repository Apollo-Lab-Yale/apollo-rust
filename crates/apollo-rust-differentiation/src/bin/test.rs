use apollo_rust_differentiation::{DerivativeNalgebraTrait, FunctionNalgebraTrait, WASPDerivativeEngine};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

struct Test;
impl FunctionNalgebraTrait for Test {
    fn call(&self, x: &V) -> V {
        V::new(&[x[0]*x[1], x[0]+x[1], x[0].powi(2)])
    }

    fn input_dim(&self) -> usize {
        2
    }

    fn output_dim(&self) -> usize {
        3
    }
}

fn main() {
    let mut w = WASPDerivativeEngine::new(Test, 0.1);
    let d = w.derivative(&V::new(&[1.,2.]));
    println!("{}", d);

    let d = w.derivative(&V::new(&[1.001,2.002]));
    println!("{}", d);

    let d = w.derivative(&V::new(&[1.03,2.04]));
    println!("{}", d);

    let d = w.derivative(&V::new(&[1.03,2.04]));
    println!("{}", d);

    let d = w.derivative(&V::new(&[1.03,2.04]));
    println!("{}", d);

    let d = w.derivative(&V::new(&[1.03,2.04]));
    println!("{}", d);
}