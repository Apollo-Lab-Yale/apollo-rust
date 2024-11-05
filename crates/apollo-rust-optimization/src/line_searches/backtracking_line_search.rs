use apollo_rust_differentiation::FunctionEngine;
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
    fn line_search(&self, function: &FunctionEngine, x_k: &V, f_k: f64, dir: &V, g_k: &V) -> f64{
        let mut alpha=self.alpha;
        while alpha > self.alpha_min {
            let x = x_k + alpha*dir;
            let f = function.call(&x)[0];
            if f-f_k < self.c*alpha*g_k.dot(dir) {break;}
            alpha *= self.rho;
        }
        alpha
    }
}