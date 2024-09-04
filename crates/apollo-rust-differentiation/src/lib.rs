use apollo_rust_linalg::{ApolloDMatrixTrait, M, V};

pub trait FunctionNalgebraTrait {
    fn call(&self, x: &V) -> V;

    fn input_dim(&self) -> usize;

    fn output_dim(&self) -> usize;
}

pub trait DerivativeNalgebraTrait {
    fn derivative(&mut self, x: &V) -> M;
}

pub struct WASPDerivativeEngine<F: FunctionNalgebraTrait> {
    f: F,
    lagrange_multiplier_inf_norm_cutoff: f64,
    p_matrices: Vec<M>,
    delta_x_mat: M,
    delta_x_mat_t: M,
    delta_f_hat_mat_t: M,
    r: usize,
    i: usize,
    pub num_f_calls: usize
}
impl<F: FunctionNalgebraTrait> WASPDerivativeEngine<F> {
    pub fn new(f: F, lagrange_multiplier_inf_norm_cutoff: f64) -> Self {
        let n = f.input_dim();
        let m = f.output_dim();

        let r = n + 1;
        let delta_x_mat = M::new_random_with_range(n, r, -1.0, 1.0);
        let delta_x_mat_t = delta_x_mat.transpose();
        let delta_f_hat_mat_t = M::zeros(r, m);

        let tmp = 2.0 * (&delta_x_mat * &delta_x_mat_t);
        let mut p_matrices = vec![];
        for i in 0..r {
            let mut p = M::zeros(n+1, n+1);
            p.view_mut((0,0), (n,n)).copy_from(&tmp);
            let delta_x_i = delta_x_mat.column(i);
            p.view_mut((n,0), (1,n)).copy_from_slice(delta_x_i.as_slice());
            p.view_mut((0,n), (n,1)).copy_from_slice((-delta_x_i).as_slice());
            p_matrices.push(p.try_inverse().expect("error"));
        }

        Self {
            f,
            lagrange_multiplier_inf_norm_cutoff,
            p_matrices,
            delta_x_mat,
            delta_x_mat_t,
            delta_f_hat_mat_t,
            r,
            i: 0,
            num_f_calls: 0,
        }
    }

    fn derivative_internal(&mut self, x: &V, recursive_call: bool, f0: Option<V>) -> M {
        let n = self.f.input_dim();
        let m = self.f.output_dim();

        let i = self.i;
        let delta_x_i = self.delta_x_mat.column(i);

        let f0 = match f0 {
            None => { self.num_f_calls += 1; self.f.call(x)  }
            Some(f0) => { f0 }
        };

        let p = 0.00001;
        let xh = x + (p * delta_x_i);
        let fh = self.f.call(&xh);
        self.num_f_calls += 1;
        let delta_f_i = (fh - &f0) / p;
        self.delta_f_hat_mat_t.view_mut((i, 0), (1, m)).copy_from_slice(delta_f_i.as_slice());

        let a_mat = 2.0 * &self.delta_x_mat * &self.delta_f_hat_mat_t;

        let mut b_mat = M::zeros(n+1, m);
        b_mat.view_mut((0,0), (n, m)).copy_from(&a_mat);
        b_mat.view_mut((n,0), (1, m)).copy_from_slice(delta_f_i.as_slice());

        let p_mat_i = &self.p_matrices[i];
        let c_mat = p_mat_i * b_mat;
        let d_mat_t = c_mat.view((0,0), (n, m));
        let lagrange_multiplier_row = c_mat.view((n,0), (1, m));
        let inf_norm = lagrange_multiplier_row.iter().max_by(|x, y| x.abs().partial_cmp(&y.abs()).unwrap()).expect("error").abs();

        if !recursive_call {
            self.delta_f_hat_mat_t = &self.delta_x_mat_t * d_mat_t;
        }

        self.i = (i + 1) % self.p_matrices.len();

        return if inf_norm > self.lagrange_multiplier_inf_norm_cutoff {
            self.derivative_internal(x, true, Some(f0.clone()))
        } else {
            d_mat_t.transpose()
        }
    }
}
impl<F: FunctionNalgebraTrait> DerivativeNalgebraTrait for WASPDerivativeEngine<F> {
    fn derivative(&mut self, x: &V) -> M {
        self.num_f_calls = 0;
        self.derivative_internal(x, false, None)
    }
}