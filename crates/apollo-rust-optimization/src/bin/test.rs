use std::f64::consts::PI;
use std::sync::Arc;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization::{IterativeOptimizerTrait, OptimizerOutputTrait, SimpleOptimizerOutput};
use apollo_rust_optimization::optimizers::gradient_descent::{GradientDescent, SimpleGradientDescent};
use apollo_rust_optimization::optimizers::open::OpENUnconstrained;
use apollo_rust_optimization::optimizers::bfgs::{BFGS, LBFGS};
use std::time::Instant;
use apollo_rust_optimization::line_searches::backtracking_line_search::BacktrackingLineSearch;

pub struct Sine;
impl FunctionNalgebraTrait for Sine {
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

pub struct Rosenbrock;
impl FunctionNalgebraTrait for Rosenbrock {
    fn call_raw(&self, x: &V) -> V {
        let a = 1.0;
        let b=100.0;
        let ret = (a-x[0]).powf(2.)+b*((x[1]-x[0]*x[0]).powf(2.));
        V::new(&[ret])
    }

    fn input_dim(&self) -> usize {
        2
    }

    fn output_dim(&self) -> usize {
        1
    }
}

pub struct NDRosenbrock;
impl FunctionNalgebraTrait for NDRosenbrock {
    fn call_raw(&self, x: &V) -> V {
        let a = 1.0;
        let b=100.0;
        let mut ret = 0.;
        for i in 0..x.len()/2{
            ret += (a-x[2*i]).powf(2.)+b*(x[2*i+1]-x[2*i]*x[2*i]).powf(2.);
        }
        V::new(&[ret])
    }

    fn input_dim(&self) -> usize {
        20
    }

    fn output_dim(&self) -> usize {
        1
    }
}


fn benchmark_gradient_descent(f: &FunctionEngine, init:&V, dim: usize) {
    let o1 = SimpleGradientDescent::new(1.0);
    let o2 = GradientDescent::new(Arc::new(BacktrackingLineSearch::default()));
    let open = OpENUnconstrained::new(dim, 100, vec![-100.0;dim], vec![100.0;dim]);

    // simple gradient without line search
    let o1_start = Instant::now();
    let res = o1.optimize_unconstrained(init, f);
    let o1_elapsed = o1_start.elapsed();
    println!("Simple Gradient: res={:?},time_elapsed={:?}", res, o1_elapsed);

    // gradient descent
    let o2_start = Instant::now();
    let res = o2.optimize_unconstrained(init, f);
    let o2_elapsed = o2_start.elapsed();
    println!("Gradient Descent: res={:?},time_elapsed={:?}", res, o2_elapsed);

    // OpEn
    let open_start=Instant::now();
    let res = open.optimize_unconstrained(init, f);
    let open_elapsed = open_start.elapsed();
    println!("Open Engine: res={:?},time_elapsed={:?}", res, open_elapsed);
}

fn benchmark_bfgs(f: &FunctionEngine, init: &V, dim: usize) {
    let o1=BFGS::new(Arc::new(BacktrackingLineSearch::default()), None);
    let o2 = LBFGS::new(Arc::new(BacktrackingLineSearch::default()), 5);
    let open = OpENUnconstrained::new(dim, 100, vec![-100.0;dim], vec![100.0;dim]);

    // BFGS
    let o1_start = Instant::now();
    let res = o1.optimize_unconstrained(init, f);
    let o1_elapsed = o1_start.elapsed();
    println!("BFGS: res={:?},time_elapsed={:?}", res, o1_elapsed);

    // LBFGS
    let o2_start = Instant::now();
    let res = o2.optimize_unconstrained(init, f);
    let o2_elapsed = o2_start.elapsed();
    println!("LBFGS: res={:?},time_elapsed={:?}", res, o2_elapsed);

    // OpEn
    let open_start=Instant::now();
    let res = open.optimize_unconstrained(init, f);
    let open_elapsed = open_start.elapsed();
    println!("Open Engine: res={:?},time_elapsed={:?}", res, open_elapsed);

}
fn main() {
    let sine = FunctionEngine::new(Sine, DerivativeMethodFD::default());
    let rosenbrock = FunctionEngine::new(Rosenbrock, DerivativeMethodFD::default());
    let ndrosenbrock = FunctionEngine::new(NDRosenbrock, DerivativeMethodFD::default());
    //benchmark_gradient_descent(&sine,&V::new(&[3.0]),1);
    //benchmark_gradient_descent(&rosenbrock,&V::new(&vec![0.; 2]),2);
     benchmark_bfgs(&ndrosenbrock, &V::new(&vec![0.; 20]), 20);
}