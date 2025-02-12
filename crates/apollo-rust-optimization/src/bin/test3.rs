use std::sync::Arc;
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization::IterativeOptimizerTrait;
use apollo_rust_optimization::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization::optimizers::bfgs::{BFGS};

pub struct TestObjective {
    pub a: f64
}
impl FunctionNalgebraTrait for TestObjective {
    fn call_raw(&self, x: &V) -> V {
        V::new(
            &[ self.a * (x[0].sin() + x[1].cos() + x[2].tan()) ]
        )
    }

    fn input_dim(&self) -> usize {
        3
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let t = TestObjective {
        a: 2.0,
    };

    let res = t.call(&V::new(&[1., 2., 3.]));
    println!("{}", res[0]);

    let fe = FunctionEngine::new(t, DerivativeMethodFD::default());
    let res = fe.derivative(&V::new(&[1., 2., 3.]));
    println!("{}", res.1);

    let line_search = Arc::new(BacktrackingLineSearch::default());
    let optimizer = BFGS::new(line_search.clone(), None);

    let res = optimizer.optimize_unconstrained(1000, &V::new(&[1., 2., 3.]), &fe);
    println!("{:?}", res.x_star);
    println!("{:?}", res.f_star);
}