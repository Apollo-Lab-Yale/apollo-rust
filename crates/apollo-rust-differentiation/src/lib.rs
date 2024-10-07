use apollo_rust_linalg::{ApolloDMatrixTrait, M, V};

pub trait FunctionNalgebraTrait {
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

pub trait DerivativeNalgebraTrait<F: FunctionNalgebraTrait> {
    fn derivative(&mut self, x: &V) -> M;
}

pub struct FDDerivativeEngine<F: FunctionNalgebraTrait> {
    pub f: F,
    pub epsilon: f64
}
impl<F: FunctionNalgebraTrait> FDDerivativeEngine<F> {
    pub fn new(f: F, epsilon: f64) -> Self {
        Self { f, epsilon }
    }
}
impl<F: FunctionNalgebraTrait> DerivativeNalgebraTrait<F> for FDDerivativeEngine<F> {
    /// Derivative for finite differencing method
    #[inline]
    fn derivative(&mut self, x: &V) -> M {
        let mut out = M::zeros(self.f.output_dim(), self.f.input_dim());

        let f0 = self.f.call(x);

        for i in 0..x.len() {
            let mut xh = x.clone();
            xh[i] += self.epsilon;
            let fh = self.f.call(&xh);
            let jvp = (fh - &f0) / self.epsilon;
            out.set_column_from_slice(i, jvp.as_slice());
        }

        out
    }
}

