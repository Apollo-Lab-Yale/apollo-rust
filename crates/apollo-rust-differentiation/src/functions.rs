use std::sync::Arc;
use apollo_rust_linalg::V;
use crate::FunctionNalgebraTrait;

#[derive(Clone)]
pub struct FunctionNalgebraWeightedSum {
    terms: Vec<Arc<dyn FunctionNalgebraTrait>>,
    weights: Vec<f64>,
    num_inputs: Option<usize>,
    num_outputs: Option<usize>
}
impl FunctionNalgebraWeightedSum {
    pub fn new_empty() -> Self {
        Self {
            terms: vec![],
            weights: vec![],
            num_inputs: None,
            num_outputs: None,
        }
    }
    pub fn insert_function<F: FunctionNalgebraTrait + 'static>(self, f: F, weight: f64) -> Self {
        let mut out = self.clone();

        match out.num_inputs {
            None => { out.num_inputs = Some(f.input_dim()) }
            Some(n) => { assert_eq!(n, f.input_dim()) }
        }

        match out.num_outputs {
            None => { out.num_outputs = Some(f.output_dim()) }
            Some(n) => { assert_eq!(n, f.output_dim()) }
        }

        out.terms.push(Arc::new(f));
        out.weights.push(weight);

        out
    }
}
impl FunctionNalgebraTrait for FunctionNalgebraWeightedSum {
    fn call_raw(&self, x: &V) -> V {
        let mut out = V::zeros(self.num_outputs.expect("error"));

        self.terms.iter().enumerate().for_each(|(i, f)| {
            out += self.weights[i] * f.call(x);
        });

        out
    }

    fn input_dim(&self) -> usize {
        self.num_inputs.expect("error")
    }

    fn output_dim(&self) -> usize {
        self.num_outputs.expect("error")
    }
}