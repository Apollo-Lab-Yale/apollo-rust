pub mod derivative_methods;
pub mod functions;

use std::ops::Deref;
use std::sync::{Arc, Mutex, RwLock};
use apollo_rust_linalg::{M, V};
use crate::derivative_methods::{FDDifferentiableFunctionEngine, WrapperDifferentiableFunction};

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

pub trait FunctionNalgebraConversionTrait : FunctionNalgebraTrait + Sized + 'static {
    fn to_wrapper_differentiable_function(self) -> WrapperDifferentiableFunction {
        WrapperDifferentiableFunction::new(self)
    }

    fn to_fd_differentiable_function(self) -> FDDifferentiableFunctionEngine {
        FDDifferentiableFunctionEngine::new_default(self)
    }
}
impl<T: FunctionNalgebraTrait + Sized + 'static> FunctionNalgebraConversionTrait for T { }

/// If you need to mutate part of the function at run-time, make sure those fields are wrapped
/// in a Mutex or something similar, then mutate the original from the outer loop in between calls.
/// Alternatively, you can wrap the whole function in an RwLock or Mutex and do a similar thing
/// ```
/// use apollo_rust_differentiation::derivative_methods::FDDifferentiableFunctionEngine;
/// use apollo_rust_differentiation::{DifferentiableFunctionEngineNalgebraTrait, FunctionNalgebraTrait};
/// use apollo_rust_linalg::{ApolloDVectorTrait, V};
/// use std::sync::{Arc, RwLock};
///
/// pub struct Test {
///     pub a: f64
/// }
/// impl FunctionNalgebraTrait for Test {
///     fn call_raw(&self, x: &V) -> V {
///         V::new(&[x[0]* self.a])
///     }
///
///     fn input_dim(&self) -> usize {
///         1
///     }
///
///     fn output_dim(&self) -> usize {
///         1
///     }
/// }
///
/// fn main() {
///     let t = Test { a: 1.0 };
///     let handle = Arc::new(RwLock::new(t));
///
///     let mut d = FDDifferentiableFunctionEngine::new(handle.clone(), 0.0000001);
///
///     println!("{}", d.derivative(&V::new(&[1.0])));
///
///     let mut tmp = handle.write().unwrap();
///     tmp.a = 5.0;
///     drop(tmp);
///
///     println!("{}", d.derivative(&V::new(&[1.0])));
/// }
/// ```
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






