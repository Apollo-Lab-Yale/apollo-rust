use std::sync::{Arc, RwLock};
use apollo_rust_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};
use crate::{DerivativeMethodNalgebraTrait, DifferentiableFunctionEngineNalgebraTrait, FunctionNalgebraTrait};

/// Soon to be deprecated
#[derive(Clone)]
pub struct DummyDifferentiableFunction;
impl DifferentiableFunctionEngineNalgebraTrait for DummyDifferentiableFunction {
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait> {
        unimplemented!("function call not supported in DummyFunctionEngine")
    }

    fn derivative(&mut self, _x: &V) -> M {
        unimplemented!("derivative call not supported in DummyFunctionEngine")
    }
}

/// Soon to be deprecated
#[derive(Clone)]
pub struct WrapperDifferentiableFunction {
    pub f: Arc<dyn FunctionNalgebraTrait>
}
impl WrapperDifferentiableFunction {
    pub fn new<F: FunctionNalgebraTrait + 'static>(f: F) -> Self {
        Self { f: Arc::new(f) }
    }
}
impl DifferentiableFunctionEngineNalgebraTrait for WrapperDifferentiableFunction {
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait> {
        &self.f
    }

    fn derivative(&mut self, _x: &V) -> M {
        unimplemented!("derivative call not supported in WrapperDifferentiableFunction")
    }
}

/// Soon to be deprecated
#[derive(Clone)]
pub struct FDDifferentiableFunctionEngine {
    pub f: Arc<dyn FunctionNalgebraTrait>,
    pub epsilon: f64
}
impl FDDifferentiableFunctionEngine {
    pub fn new<F: FunctionNalgebraTrait + 'static>(f: F, epsilon: f64) -> Self {
        Self { f: Arc::new(f), epsilon }
    }
    pub fn new_default<F: FunctionNalgebraTrait + 'static>(f: F) -> Self {
        Self::new(f, 0.0000001)
    }
}
impl DifferentiableFunctionEngineNalgebraTrait for FDDifferentiableFunctionEngine {
    #[inline(always)]
    fn function(&self) -> &Arc<dyn FunctionNalgebraTrait> { &self.f }

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

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct DerivativeMethodDummy;
impl DerivativeMethodNalgebraTrait for DerivativeMethodDummy {
    fn derivative(&self, _f: &Arc<dyn FunctionNalgebraTrait>, _x: &V) -> (V, M) {
        unimplemented!("derivative call not supported in DummyDerivative")
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct DerivativeMethodFD {
    pub epsilon: f64
}
impl DerivativeMethodFD {
    pub fn new(epsilon: f64) -> Self {
        Self { epsilon }
    }
}
impl Default for DerivativeMethodFD {
    fn default() -> Self {
        Self::new(0.000001)
    }
}
impl DerivativeMethodNalgebraTrait for DerivativeMethodFD {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M) {
        let mut out = M::zeros(f.output_dim(), f.input_dim());

        let f0 = f.call(x);

        for i in 0..x.len() {
            let mut xh = x.clone();
            xh[i] += self.epsilon;
            let fh = f.call(&xh);
            let jvp = (fh - &f0) / self.epsilon;
            out.set_column_from_slice(i, jvp.as_slice());
        }

        (f0, out)
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct DerivativeMethodWASP {
    cache: RwLock<WASPCache>,
    num_f_calls: RwLock<usize>,
    d_theta: f64,
    d_ell: f64
}
impl DerivativeMethodWASP {
    pub fn new(n: usize, m: usize, orthonormal_delta_x: bool, d_theta: f64, d_ell: f64) -> Self {
        Self {
            cache: RwLock::new(WASPCache::new(n, m, orthonormal_delta_x)),
            num_f_calls: RwLock::new(0),
            d_theta,
            d_ell,
        }
    }
    pub fn new_default(n: usize, m: usize) -> Self {
        Self::new(n, m, true, 0.3, 0.3)
    }
    pub fn num_f_calls(&self) -> usize {
        return self.num_f_calls.read().unwrap().clone()
    }
}
impl DerivativeMethodNalgebraTrait for DerivativeMethodWASP {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M) {
        let mut num_f_calls = 0;
        let f_k = f.call(x);
        num_f_calls += 1;
        let epsilon = 0.000001;

        let mut cache = self.cache.write().unwrap();
        let n = x.len();

        loop {
            let i = cache.i.clone();

            let delta_x_i = cache.delta_x.get_column(i);

            let x_k_plus_delta_x_i = x + epsilon*&delta_x_i;
            let f_k_plus_delta_x_i = f.call(&x_k_plus_delta_x_i);
            num_f_calls += 1;
            let delta_f_i = (&f_k_plus_delta_x_i - &f_k) / epsilon;
            let delta_f_i_hat = cache.delta_f_t.get_row(i);
            let return_result = close_enough(&delta_f_i, &delta_f_i_hat, self.d_theta, self.d_ell);

            cache.delta_f_t.set_row(i, &delta_f_i.transpose());
            let c_1_mat = &cache.c_1[i];
            let c_2_mat = &cache.c_2[i];
            let delta_f_t = &cache.delta_f_t;

            let d_t_star = c_1_mat*delta_f_t + c_2_mat*delta_f_i.transpose();
            let d_star = d_t_star.transpose();

            let tmp = &d_star * &cache.delta_x;
            cache.delta_f_t = tmp.transpose();

            let mut new_i = i + 1;
            if new_i >= n { new_i = 0; }
            cache.i = new_i;

            if return_result {
                *self.num_f_calls.write().unwrap() = num_f_calls;
                return (f_k, d_star);
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct WASPCache {
    pub i: usize,
    pub delta_f_t: M,
    pub delta_x: M,
    pub c_1: Vec<M>,
    pub c_2: Vec<V>
}
impl WASPCache {
    pub fn new(n: usize, m: usize, orthonormal_delta_x: bool) -> Self {
        let delta_f_t = M::identity(n, m);
        let delta_x = get_tangent_matrix(n, orthonormal_delta_x);
        let mut c_1 = vec![];
        let mut c_2 = vec![];

        let a_mat = 2.0 * &delta_x * &delta_x.transpose();
        let a_inv_mat = a_mat.try_inverse().unwrap();

        for i in 0..n {
            let delta_x_i = V::new(delta_x.column(i).as_slice());
            let s_i = (delta_x_i.transpose() * &a_inv_mat * &delta_x_i)[(0,0)];
            let s_i_inv = 1.0 / s_i;
            let c_1_mat = &a_inv_mat * (M::identity(n, n) - s_i_inv * &delta_x_i * delta_x_i.transpose() * &a_inv_mat) * 2.0 * &delta_x;
            let c_2_mat = s_i_inv * &a_inv_mat * delta_x_i;
            c_1.push(c_1_mat);
            c_2.push(c_2_mat);
        }

        return Self {
            i: 0,
            delta_f_t,
            delta_x,
            c_1,
            c_2,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct DerivativeMethodWASP2 {
    cache: RwLock<WASPCache2>,
    num_f_calls: RwLock<usize>,
    d_theta: f64,
    d_ell: f64
}
impl DerivativeMethodWASP2 {
    pub fn new(n: usize, m: usize, alpha: f64, orthonormal_delta_x: bool, d_theta: f64, d_ell: f64) -> Self {
        Self {
            cache: RwLock::new(WASPCache2::new(n, m, alpha, orthonormal_delta_x)),
            num_f_calls: RwLock::new(0),
            d_theta,
            d_ell,
        }
    }
    pub fn new_default(n: usize, m: usize) -> Self {
        Self::new(n, m, 0.98, true, 0.3, 0.3)
    }
    pub fn num_f_calls(&self) -> usize {
        return self.num_f_calls.read().unwrap().clone()
    }
}
impl DerivativeMethodNalgebraTrait for DerivativeMethodWASP2 {
    fn derivative(&self, f: &Arc<dyn FunctionNalgebraTrait>, x: &V) -> (V, M) {
        let mut num_f_calls = 0;
        let f_k = f.call(x);
        num_f_calls += 1;
        let epsilon = 0.000001;

        let mut cache = self.cache.write().unwrap();
        let n = x.len();

        loop {
            let i = cache.i.clone();

            let delta_x_i = cache.delta_x.get_column(i);

            let x_k_plus_delta_x_i = x + epsilon*&delta_x_i;
            let f_k_plus_delta_x_i = f.call(&x_k_plus_delta_x_i);
            num_f_calls += 1;
            let delta_f_i = (&f_k_plus_delta_x_i - &f_k) / epsilon;
            let delta_f_i_hat = &cache.curr_d * &delta_x_i;
            let return_result = close_enough(&delta_f_i, &delta_f_i_hat, self.d_theta, self.d_ell);

            cache.delta_f_t.set_row(i, &delta_f_i.transpose());
            let c_1_mat = &cache.c_1[i];
            let c_2_mat = &cache.c_2[i];
            let delta_f_t = &cache.delta_f_t;

            let d_t_star = c_1_mat*delta_f_t + c_2_mat*delta_f_i.transpose();
            let d_star = d_t_star.transpose();
            cache.curr_d = d_star.clone();

            let mut new_i = i + 1;
            if new_i >= n { new_i = 0; }
            cache.i = new_i;

            if return_result {
                *self.num_f_calls.write().unwrap() = num_f_calls;
                return (f_k, d_star);
            }
        }
    }
}

pub struct WASPCache2 {
    pub i: usize,
    pub curr_d: M,
    pub delta_f_t: M,
    pub delta_x: M,
    pub c_1: Vec<M>,
    pub c_2: Vec<V>
}
impl WASPCache2 {
    pub fn new(n: usize, m: usize, alpha: f64, orthonormal_delta_x: bool) -> Self {
        assert!(alpha > 0.0 && alpha < 1.0);

        let curr_d = M::identity(m, n);
        let delta_f_t = M::identity(n, m);
        let delta_x = get_tangent_matrix(n, orthonormal_delta_x);
        let mut c_1 = vec![];
        let mut c_2 = vec![];

        for i in 0..n {
            let delta_x_i = V::new(delta_x.column(i).as_slice());
            let mut w_i = M::zeros(n, n);
            for j in 0..n {
                let exponent = math_mod(i as i32 - j as i32, n as i32) as f64 / (n as i32 - 1) as f64;
                w_i[(j, j)] = alpha * (1.0 - alpha).powf(exponent);
            }
            let w_i_2 = &w_i * &w_i;

            let a_i = 2.0 * &delta_x * &w_i_2 * &delta_x.transpose();
            let a_i_inv = a_i.try_inverse().unwrap();

            let s_i = (delta_x_i.transpose() * &a_i_inv * &delta_x_i)[(0, 0)];
            let s_i_inv = 1.0 / s_i;
            let c_1_mat = &a_i_inv * (M::identity(n, n) - s_i_inv * &delta_x_i * delta_x_i.transpose() * &a_i_inv) * 2.0 * &delta_x * &w_i_2;
            let c_2_mat = s_i_inv * &a_i_inv * delta_x_i;
            c_1.push(c_1_mat);
            c_2.push(c_2_mat);
        }

        return Self {
            i: 0,
            curr_d,
            delta_f_t,
            delta_x,
            c_1,
            c_2,
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn math_mod(a: i32, b: i32) -> i32 {
    return ((a % b) + b) % b;
}

pub (crate) fn get_tangent_matrix(n: usize, orthonormal: bool) -> M {
    let t = M::new_random_with_range(n, n, -1.0, 1.0);
    return if orthonormal {
        let svd = t.svd(true, true);
        let u = svd.u.unwrap();
        let v_t = svd.v_t.unwrap();
        let delta_x = u * v_t;
        delta_x
    } else {
        t
    }
}

pub (crate) fn close_enough(a: &V, b: &V, d_theta: f64, d_ell: f64) -> bool {
    let a_n = a.norm();
    let b_n = b.norm();

    let tmp = ((a.dot(&b) / ( a_n*b_n )) - 1.0).abs();
    if tmp > d_theta { return false; }

    let tmp1 = if b_n != 0.0 {
        ((a_n / b_n) - 1.0).abs()
    } else {
        f64::MAX
    };
    let tmp2 = if a_n != 0.0 {
        ((b_n / a_n) - 1.0).abs()
    } else {
        f64::MAX
    };

    if f64::min(tmp1, tmp2) > d_ell { return false; }

    return true;
}