use apollo_rust_differentiation::derivative_methods::{DummyDifferentiableFunction, FDDifferentiableFunctionEngine};
use apollo_rust_differentiation::{DifferentiableFunctionEngineNalgebraTrait, FunctionNalgebraTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_optimization::GradientBasedOptimizerTrait;

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

pub struct Test2;
impl GradientBasedOptimizerTrait for Test2 {
    type OutputType = ();

    fn optimize(&self, objective_function: &impl DifferentiableFunctionEngineNalgebraTrait, equality_constraint: Option<&impl DifferentiableFunctionEngineNalgebraTrait>, inequality_constraint: Option<&impl DifferentiableFunctionEngineNalgebraTrait>) -> Self::OutputType {
        ()
    }
}

fn main() {
    let fd = FDDifferentiableFunctionEngine::new_default(Test);

    let o = Test2;
    o.optimize(&fd, None::<&DummyDifferentiableFunction>, None::<&DummyDifferentiableFunction>);
}