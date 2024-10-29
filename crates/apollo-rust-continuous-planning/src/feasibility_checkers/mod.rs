pub mod robot_feasibility_checkers;

use apollo_rust_linalg::V;

pub trait FeasibilityCheckerTrait {
    fn is_feasible_state(&self, state: &V) -> bool;
}