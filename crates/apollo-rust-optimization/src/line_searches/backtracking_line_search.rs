use apollo_rust_differentiation::FunctionEngine;
use apollo_rust_linalg::V;
use crate::LineSearchTrait;
pub struct BacktrackingLineSearch {

}

impl LineSearchTrait for BacktrackingLineSearch {
    fn line_search(&self, function: &FunctionEngine, x_k: &V, dir: &V) -> f64{
        0.
    }
}