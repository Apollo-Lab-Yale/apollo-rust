use std::sync::{Arc};
use apollo_rust_linalg::{ApolloDMatrixTrait, M, V};
use crate::{DerivativeMethodNalgebraTrait, DifferentiableFunctionEngineNalgebraTrait, FunctionNalgebraTrait};

/// Soon to be deprecated
#[derive(Clone)]
pub struct DummyDifferentiableFunction;
impl DifferentiableFunctionEngineNalgebraTrait for DummyDifferentiableFunction {
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait> {
        unimplemented!("function call not supported in DummyFunctionEngine")
    }

    fn derivative(&mut self, _x: &V) -> M {
        unimplemented!("derivative call not supported in DummyFunctionEngine")
    }
}

/// Soon to be deprecated
#[derive(Clone)]
pub struct WrapperDifferentiableFunction {
    pub f: Arc<dyn FunctionNalgebraTrait>
}
impl WrapperDifferentiableFunction {
    pub fn new<F: FunctionNalgebraTrait + 'static>(f: F) -> Self {
        Self { f: Arc::new(f) }
    }
}
impl DifferentiableFunctionEngineNalgebraTrait for WrapperDifferentiableFunction {
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait> {
        &self.f
    }

    fn derivative(&mut self, _x: &V) -> M {
        unimplemented!("derivative call not supported in DummyFunctionEngine")
    }
}

/// Soon to be deprecated
#[derive(Clone)]
pub struct FDDifferentiableFunctionEngine {
    pub f: Arc<dyn FunctionNalgebraTrait>,
    pub epsilon: f64
}
impl FDDifferentiableFunctionEngine {
    pub fn new<F: FunctionNalgebraTrait + 'static>(f: F, epsilon: f64) -> Self {
        Self { f: Arc::new(f), epsilon }
    }
    pub fn new_default<F: FunctionNalgebraTrait + 'static>(f: F) -> Self {
        Self::new(f, 0.0000001)
    }
}
impl DifferentiableFunctionEngineNalgebraTrait for FDDifferentiableFunctionEngine {
    #[inline(always)]
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait> { &self.f }

    /// Derivative for finite differencing method
    #[inline]
    fn derivative(&mut self, x: &V) -> M {
        let mut out = M::zeros(self.f.output_dim(), self.f.input_dim());

        let f0 = self.f.call(x);

        for i in 0..x.len() {
            let mut xh = x.clone();
            xh[i] += self.epsilon;
            let fh = self.f.call(&xh);
            let jvp = (fh - &f0) / self.epsilon;
            out.set_column_from_slice(i, jvp.as_slice());
        }

        out
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct DerivativeMethodDummy;
impl DerivativeMethodNalgebraTrait for DerivativeMethodDummy {
    fn derivative(&self, _f: &Arc<dyn FunctionNalgebraTrait>, _x: &V) -> M {
        unimplemented!("derivative call not supported in DummyDerivative")
    }
}

pub struct DerivativeMethodFD {
    pub epsilon: f64
}
impl DerivativeMethodFD {
    pub fn new(epsilon: f64) -> Self {
        Self { epsilon }
    }
}
impl Default for DerivativeMethodFD {
    fn default() -> Self {
        Self::new(0.000001)
    }
}
impl DerivativeMethodNalgebraTrait for DerivativeMethodFD {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> M {
        let mut out = M::zeros(f.output_dim(), f.input_dim());

        let f0 = f.call(x);

        for i in 0..x.len() {
            let mut xh = x.clone();
            xh[i] += self.epsilon;
            let fh = f.call(&xh);
            let jvp = (fh - &f0) / self.epsilon;
            out.set_column_from_slice(i, jvp.as_slice());
        }

        out
    }
}