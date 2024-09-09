use apollo_rust_differentiation::{DerivativeNalgebraTrait, FunctionNalgebraTrait, WASPDerivativeEngine};
use apollo_rust_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};

#[allow(unused)]
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

#[derive(Clone)]
struct Test2 {
    pub m: M
}
impl Test2 {
    pub fn new(n: usize, m: usize) -> Self {
        Self {
            m: M::new_random_with_range(m, n, -1.0, 1.0),
        }
    }
}
impl FunctionNalgebraTrait for Test2 {
    fn call(&self, x: &V) -> V {
        return &self.m * x;
    }

    fn input_dim(&self) -> usize {
        self.m.ncols()
    }

    fn output_dim(&self) -> usize {
        self.m.nrows()
    }
}

fn main() {
    let f = Test2::new(5, 5);
    println!("{}", f.m);
    println!("---");
    let mut w = WASPDerivativeEngine::new(f, 0.0001);
    let mut v = V::new_random_with_range(5, -1.0, 1.0);

    for _ in 0..10 {
        v += V::new_random_with_range(5, -0.1, 0.1);
        let d = w.derivative(&v);
        println!("{}", d);
        println!("{:?}", w.num_f_calls);
    }
}