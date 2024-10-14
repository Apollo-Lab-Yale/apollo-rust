pub mod derivative_methods;
pub mod functions;

use std::sync::Arc;
use apollo_rust_linalg::{M, V};

pub trait FunctionNalgebraTrait {
    fn call(&self, x: &V) -> V {
        assert_eq!(x.len(), self.input_dim());
        let out = self.call_raw(x);
        assert_eq!(out.len(), self.output_dim());
        return out;
    }

    fn call_raw(&self, x: &V) -> V;

    fn input_dim(&self) -> usize;

    fn output_dim(&self) -> usize;
}

pub trait DifferentiableFunctionEngineNalgebraTrait : Clone {
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait>;
    fn derivative(&mut self, x: &V) -> M;
    #[inline(always)]
    fn call(&self, x: &V) -> V {
        self.function().call(x)
    }
    #[inline(always)]
    fn input_dim(&self) -> usize {
        self.function().input_dim()
    }
    #[inline(always)]
    fn output_dim(&self) -> usize {
        self.function().output_dim()
    }
}






