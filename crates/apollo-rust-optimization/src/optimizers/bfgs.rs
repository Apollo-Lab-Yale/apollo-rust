use std::sync::Arc;
use apollo_rust_differentiation::FunctionEngine;
use apollo_rust_linalg::{V,M};
use crate::{IterativeOptimizerTrait, LineSearchTrait, SimpleOptimizerOutput};
use std::collections::VecDeque;

pub struct BFGS {
    pub line_search: Arc<dyn LineSearchTrait>,
    init_h: Option<M>
}

impl BFGS {
    pub fn new (line_search: Arc<dyn LineSearchTrait>, init_h: Option<M>) -> Self {
        Self {
            line_search,
            init_h
        }
    }
}

impl IterativeOptimizerTrait for BFGS {
    type OutputType = SimpleOptimizerOutput;

    fn optimize_raw(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
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
            if num_iters>=max_iterations || norm < 0.01 {
                return SimpleOptimizerOutput {
                    x_star: x_k,
                    f_star: f_k,
                    num_iters,
                }
            }
            // compute descending step
            let p_k = -&h_k*&g_k;
            let lambda = self.line_search.line_search(objective_function, &x_k, f_k, &p_k, &g_k);
            let s_k = lambda * p_k;
            // update
            x_k = x_k + &s_k;
            let (f_k_next, g_k_next) = objective_function.derivative(&x_k);
            let g_k_next = V::from_column_slice(g_k_next.as_slice());
            let y_k = &g_k_next-g_k;
            g_k = g_k_next;
            f_k = f_k_next[0];
            // in case Wolfe's condition is not satisfied
            let inv_rho_k = s_k.dot(&y_k);
            if inv_rho_k > 1e-6 {
                let rho_k = 1.0 / inv_rho_k;
                h_k = (&id - rho_k * &s_k * y_k.transpose()) * h_k * (&id - rho_k * y_k * s_k.transpose()) + rho_k * &s_k * s_k.transpose();
            }
            num_iters += 1;
        }
    }
}

pub struct LBFGS {
    pub line_search: Arc<dyn LineSearchTrait>,
    m: usize,
}
impl LBFGS {
    pub fn new(line_search: Arc<dyn LineSearchTrait>, m: usize) -> Self {
        Self{
            line_search,
            m
        }
    }
}
impl IterativeOptimizerTrait for LBFGS {
    type OutputType = SimpleOptimizerOutput;

    fn optimize_raw(&self, max_iterations: usize, init_condition: &V, objective_function: &FunctionEngine, equality_constraint: Option<&FunctionEngine>, inequality_constraint: Option<&FunctionEngine>) -> Self::OutputType {
        assert!(equality_constraint.is_none());
        assert!(inequality_constraint.is_none());

        // initialize
        let mut x_k = init_condition.clone();
        let (f_k,g_k) =  objective_function.derivative(&x_k);
        let mut f_k = f_k[0];
        let mut g_k = V::from_column_slice(g_k.as_slice());
        let mut s_queue:VecDeque<V>=VecDeque::with_capacity(self.m);
        let mut y_queue:VecDeque<V>=VecDeque::with_capacity(self.m);
        let mut rho_queue:VecDeque<f64>=VecDeque::with_capacity(self.m);
        let mut alpha_queue:VecDeque<f64>=VecDeque::with_capacity(self.m);
        let mut num_iters = 0;
        // optimize
        loop{
            let norm = g_k.norm();
            if num_iters>=max_iterations || norm < 0.01 {
                return SimpleOptimizerOutput {
                    x_star: x_k,
                    f_star: f_k,
                    num_iters,
                }
            }
            let mut p_k = -&g_k;
            // two-loop recursion
            if s_queue.len()>0 {
                alpha_queue.clear();
                for ((s, y), rho) in s_queue.iter().rev().zip(y_queue.iter().rev()).zip(rho_queue.iter().rev()) {
                    let alpha = rho * s.dot(&p_k);
                    p_k = p_k - alpha * y;
                    alpha_queue.push_back(alpha);
                }
                let s_rear = s_queue.back().unwrap();
                let y_rear = y_queue.back().unwrap();
                p_k *= s_rear.dot(y_rear) / y_rear.dot(y_rear);
                for (((s, y), rho), alpha) in s_queue.iter().zip(y_queue.iter()).zip(rho_queue.iter()).zip(alpha_queue.iter().rev()) {
                    let beta = rho * y.dot(&p_k);
                    p_k = p_k + (alpha-beta)*s;
                }
            }
            let lambda = self.line_search.line_search(objective_function, &x_k, f_k, &p_k, &g_k);
            let s_k = lambda * p_k;
            // update
            x_k = x_k + &s_k;
            let (f_k_next, g_k_next) = objective_function.derivative(&x_k);
            let g_k_next = V::from_column_slice(g_k_next.as_slice());
            let y_k = &g_k_next - g_k;
            g_k = g_k_next;
            f_k = f_k_next[0];
            // in case Wolfe's condition is not satisfied
            let inv_rho_k = s_k.dot(&y_k);
            if inv_rho_k > 1e-6 {
                if num_iters >= self.m {
                    s_queue.pop_front();
                    y_queue.pop_front();
                    rho_queue.pop_front();
                }
                rho_queue.push_back(1.0 /inv_rho_k);
                s_queue.push_back(s_k);
                y_queue.push_back(y_k);
            }
            num_iters+=1;
        }
    }
}