use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};
use apollo_rust_optimization::IterativeOptimizerTrait;
use apollo_rust_optimization::optimizers::open::OpENUnconstrained;

fn reflect(x: &V, y: &V) -> V {
    let tmp = 2.0*(x.dot(y) / y.dot(y)) * y;
    return x - tmp;
}
fn scale(x: &V, singular_vals: &[f64], n: usize, m: usize) -> V {
    assert_eq!( singular_vals.len(), n.min(m) );

    let singular_vals: Vec<f64> = singular_vals.iter().map(|x| x.abs()).collect();
    let mut d = M::from_diagonal(&V::new(&singular_vals));

    if n > m {
        let z = M::zeros(m, n - m);
        d = d.hstack(&z).unwrap();
    } else if m > n {
        let z = M::zeros(m - n, n);
        d = d.vstack(&z).unwrap();
    }

    return d * x;
}
fn reflect_map(state: &V, x: &V, n: usize, m: usize) -> V {
    let (u1, u2, s, v1, v2) = split(state, n, m);

    let x = reflect(x, &v1);
    let x = reflect(&x, &v2);
    let x = scale(&x, &s.as_slice(), n, m);
    let x = reflect(&x, &u1);
    let x = reflect(&x, &u2);

    return x;
}
fn split(state: &V, n: usize, m: usize) -> (V, V, V, V, V) {
    let r = n.min(m);
    let mut curr = 0;

    let u1 = V::new(&state.view((curr,0), (m, 1)).iter().map(|x| *x).collect::<Vec<f64>>());
    curr += m;
    let u2 = V::new(&state.view((curr,0), (m, 1)).iter().map(|x| *x).collect::<Vec<f64>>());
    curr += m;
    let s = V::new(&state.view((curr,0), (r, 1)).iter().map(|x| *x).collect::<Vec<f64>>());
    curr += r;
    let v1 = V::new(&state.view((curr,0), (n, 1)).iter().map(|x| *x).collect::<Vec<f64>>());
    curr += n;
    let v2 = V::new(&state.view((curr,0), (n, 1)).iter().map(|x| *x).collect::<Vec<f64>>());

    return (u1, u2, s, v1, v2)
}

fn to_matrix(state: &V, n: usize, m: usize) -> M {
    let (u1, u2, s, v1, v2) = split(state, n, m);

    // let singular_vals: Vec<f64> = s.iter().map(|x| x.abs()).collect();
    let mut d = M::from_diagonal(&s);

    if n > m {
        let z = M::zeros(m, n - m);
        d = d.hstack(&z).unwrap();
    } else if m > n {
        let z = M::zeros(m - n, n);
        d = d.vstack(&z).unwrap();
    }

    let vt = (M::identity(n,n) - &v1*v1.transpose()) * (M::identity(n,n) - &v2*v2.transpose());
    let u = (M::identity(m,m) - &u1*u1.transpose()) * (M::identity(m,m) - &u2*u2.transpose());

    return u*d*vt;
}

pub struct F1 {
    pub x: V,
    pub target: V,
    pub n: usize,
    pub m: usize
}
impl FunctionNalgebraTrait for F1 {
    fn call_raw(&self, x: &V) -> V {
       //  let res = reflect_map(x, &self.x, self.n, self.m);
        let mm = to_matrix(x, self.n, self.m);
        let val = (mm*&self.x - &self.target).norm();
        return V::new(&[val]);
    }

    fn input_dim(&self) -> usize {
        2*self.n + 2*self.m + self.n.min(self.m)
    }

    fn output_dim(&self) -> usize {
        1
    }
}

pub struct F2 {
    pub x: V,
    pub target: V,
    pub n: usize,
    pub m: usize
}
impl FunctionNalgebraTrait for F2 {
    fn call_raw(&self, x: &V) -> V {
        let m = M::from_column_slice(self.m, self.n, x.as_slice());
        let val = m*&self.x;
        let val2 = (val - &self.target).norm();
        return V::new(&[val2]);
    }

    fn input_dim(&self) -> usize {
        self.m*self.n
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn main() {
    let n = 10;
    let m = 10;
    let nn = 2*n + 2*m + n.min(m);

    let x = V::new_random_with_range(n, -1.0, 1.0);

    let target = V::new_random_with_range(m, -1.0, 1.0);

    let f1 = F1 {
        x: x.clone(),
        target: target.clone(),
        n,
        m,
    };
    let fe1 = FunctionEngine::new(f1, DerivativeMethodFD::new(0.0000001));

    let f2 = F2 {
        x: x.clone(),
        target: target.clone(),
        n,
        m,
    };
    let fe2 = FunctionEngine::new(f2, DerivativeMethodFD::new(0.0000001));

    let o = OpENUnconstrained::new(nn, 1000, vec![-10000.0; nn], vec![10000.0; nn]);
    let res = o.optimize_unconstrained(&V::new(&vec![0.001; nn]), &fe1);

    let (u1, u2, s, v1, v2) = split(&res.x_star, n, m);
    println!("{}", res.f_star);
    let mm = to_matrix(&res.x_star, n, m);
    println!("{}", mm);
    println!("{}", mm*x.clone());
    println!("---");

    let o = OpENUnconstrained::new(m*n, 1000, vec![-10000.0; m*n], vec![10000.0; m*n]);
    let res = o.optimize_unconstrained(&V::new(&vec![0.001; m*n]), &fe2);

    println!("{}", res.f_star);
    let mm = M::from_column_slice(m, n, &res.x_star.as_slice());
    println!("{}", mm);
    println!("{}", mm*x.clone());
    println!("{}", target);

}