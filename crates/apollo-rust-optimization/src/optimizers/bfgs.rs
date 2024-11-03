use std::sync::Arc;
use apollo_rust_differentiation::FunctionEngine;
use apollo_rust_linalg::{V,M};
use crate::{IterativeOptimizerTrait, LineSearchTrait, SimpleOptimizerOutput};

pub struct BFGS {
    pub line_search: Arc<dyn LineSearchTrait>,
    pub init_h: Option<M>
}

impl BFGS {
    pub fn new<L: LineSearchTrait + 'static>(line_search: L, init_h: Option<M>) -> Self {
        Self {
            line_search: Arc::new(line_search),
            init_h
        }
    }
}

impl IterativeOptimizerTrait for BFGS {
    type OutputType = SimpleOptimizerOutput;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        assert!(equality_constraint.is_none());
        assert!(inequality_constraint.is_none());

        // initialize
        let mut x_k = init_condition.clone();
        let (f_k,g_k) =  objective_function.derivative(&x_k);
        let mut f_k = f_k[0];
        let mut g_k = V::from_column_slice(g_k.as_slice());
        let mut h_k: M;
        if self.init_h.is_some() {
           h_k = self.init_h.as_ref().unwrap().clone();
        }
        else {
            h_k = M::identity(x_k.len(), x_k.len());
        }
        let id = M::identity(h_k.nrows(), h_k.ncols());
        let mut num_iters = 0;
        // optimize
        loop{
            let norm = g_k.norm();
            if norm < 0.01 {
                return SimpleOptimizerOutput {
                    x_star: x_k,
                    f_star: f_k,
                    num_iters,
                }
            }
            // compute descending step
            let p_k = -&h_k*&g_k;
            let lambda = self.line_search.line_search(objective_function, &x_k, &f_k[0], &p_k, &g_k);
            let s_k = lambda * p_k;
            // update
            x_k = x_k + &s_k;
            let (f_k_next, g_k_next) = objective_function.derivative(&x_k);
            let g_k_next = V::from_column_slice(g_k_next.as_slice());
            let y_k = &g_k_next-g_k;
            let rho_k  = 1.0/s_k.dot(&y_k);
            h_k = (&id-rho_k*&s_k*y_k.transpose())*h_k*(&id-rho_k*y_k*s_k.transpose())+rho_k*&s_k*s_k.transpose();
            g_k = g_k_next;
            f_k = f_k_next[0];
            num_iters += 1;
        }
    }
}

pub struct LBFGS {
    pub line_search: Arc<dyn LineSearchTrait>
}
impl LBFGS {

}
impl IterativeOptimizerTrait for LBFGS {
    type OutputType = SimpleOptimizerOutput;

    fn optimize_raw(&self, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        todo!()
    }
}