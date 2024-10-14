use apollo_rust_differentiation::{DifferentiableFunctionEngineNalgebraTrait, FDFunctionAndDerivativeEngine, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

pub struct Test;
impl FunctionNalgebraTrait for Test {
    fn call_raw(&self, x: &V) -> V {
        let out = x[0].sin() + x[1].cos();
        return V::from_column_slice(&[out]);
    }

    fn input_dim(&self) -> usize {
        2
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let mut fd = FDFunctionAndDerivativeEngine::new(Test);
    let res = fd.derivative(&V::new(&[1.,1.]));
    println!("{}", res);
}