pub mod derivative_methods;
pub mod functions;

use std::ops::{Deref};
use std::sync::{Arc, Mutex, RwLock};
use apollo_rust_linalg::{M, V};

pub trait FunctionNalgebraTrait  {
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
impl<T: FunctionNalgebraTrait> FunctionNalgebraTrait for Arc<T> {
    #[inline(always)]
    fn call_raw(&self, x: &V) -> V {
        self.deref().call(x)
    }

    #[inline(always)]
    fn input_dim(&self) -> usize {
        self.deref().input_dim()
    }

    #[inline(always)]
    fn output_dim(&self) -> usize {
        self.deref().output_dim()
    }

}
impl<T: FunctionNalgebraTrait> FunctionNalgebraTrait for Mutex<T> {
    #[inline(always)]
    fn call_raw(&self, x: &V) -> V {
        let tmp = self.lock().unwrap();
        tmp.call_raw(x)
    }

    #[inline(always)]
    fn input_dim(&self) -> usize {
        let tmp = self.lock().unwrap();
        tmp.input_dim()
    }

    #[inline(always)]
    fn output_dim(&self) -> usize {
        let tmp = self.lock().unwrap();
        tmp.output_dim()
    }


}
impl<T: FunctionNalgebraTrait> FunctionNalgebraTrait for RwLock<T> {
    #[inline(always)]
    fn call_raw(&self, x: &V) -> V {
        let tmp = self.read().unwrap();
        tmp.call_raw(x)
    }

    #[inline(always)]
    fn input_dim(&self) -> usize {
        let tmp = self.read().unwrap();
        tmp.input_dim()
    }

    #[inline(always)]
    fn output_dim(&self) -> usize {
        let tmp = self.read().unwrap();
        tmp.output_dim()
    }


}


pub trait DerivativeMethodNalgebraTrait {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M);
}
impl<T: DerivativeMethodNalgebraTrait> DerivativeMethodNalgebraTrait for Arc<T> {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M) {
        self.deref().derivative(f, x)
    }
}
impl<T: DerivativeMethodNalgebraTrait> DerivativeMethodNalgebraTrait for Mutex<T> {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M) {
        let tmp = self.lock().unwrap();
        tmp.derivative(f, x)
    }
}
impl<T: DerivativeMethodNalgebraTrait> DerivativeMethodNalgebraTrait for RwLock<T> {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M) {
        let tmp = self.read().unwrap();
        tmp.derivative(f, x)
    }
}


/// Soon to be deprecated
pub trait DifferentiableFunctionEngineNalgebraTrait: Clone {
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

#[derive(Clone)]
pub struct FunctionEngine {
    function: Arc<dyn FunctionNalgebraTrait>,
    derivative_method: Arc<dyn DerivativeMethodNalgebraTrait>
}
impl FunctionEngine {
    pub fn new<F: FunctionNalgebraTrait + 'static, D: DerivativeMethodNalgebraTrait + 'static>(f: F, d: D) -> Self {
        Self {
            function: Arc::new(f),
            derivative_method: Arc::new(d),
        }
    }

    #[inline(always)]
    pub fn call(&self, x: &V) -> V {
        self.function.call(x)
    }

    #[inline(always)]
    pub fn derivative(&self, x: &V) -> (V, M) {
        self.derivative_method.derivative(&self.function, x)
    }

    #[inline(always)]
    pub fn input_dim(&self) -> usize {
        self.function.input_dim()
    }

    #[inline(always)]
    pub fn output_dim(&self) -> usize {
        self.function.output_dim()
    }

}






