use apollo_rust_linalg::V;

pub trait FeasibilityChecker {
    fn is_feasible_state(&self, state: &V) -> bool;
}