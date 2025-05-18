use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use ad_trait::function_engine::FunctionEngine;
use apollo_rust_linalg::V;
use crate::LineSearchTrait;
pub struct BacktrackingLineSearch {
    alpha: f64,
    alpha_min: f64,
    rho: f64,
    c: f64
}

impl BacktrackingLineSearch {
    pub fn new(alpha: f64, alpha_min: f64, rho:f64, c:f64) -> Self {
        Self {alpha, alpha_min, rho, c }
    }
    pub fn default() -> Self {
        Self::new(1.0, 1e-10, 0.5, 1e-4)
    }
}

impl LineSearchTrait for BacktrackingLineSearch {
    fn line_search<F1, F2, E>(&self, function: &FunctionEngine<F1, F2, E>, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64
        where F1: DifferentiableFunctionTrait<f64>,
              F2: DifferentiableFunctionTrait<E::T>,
              E: DerivativeMethodTrait
    {
        let mut alpha=self.alpha;
        while alpha > self.alpha_min {
            let x = x_k + alpha*dir;
            let f = function.call(x.as_slice())[0];
            if f-f_k < self.c*alpha*g_k.dot(dir) {break;}
            alpha *= self.rho;
        }
        alpha
    }
}