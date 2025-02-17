/*

use std::borrow::Cow;
use nalgebra::DMatrix;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{Optimizer, Problem, SolverError};
use optimization_engine::constraints::NoConstraints;
use optimization_engine::core::SolverStatus;
use rand::Rng;
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_spatial::lie::se3_implicit_quaternion::{ApolloLieAlgPackIse3qTrait, ApolloPseudoLieAlgPackIse3qTrait, ISE3q};
use apollo_rust_spatial::vectors::{ApolloVector6Trait, V6};
use crate::double_group_queries::DoubleGroupProximityQueryMode;
use crate::offset_shape::OffsetShape;
use crate::proxima::proxima_core::{ProximaCacheElementTrait, ProximaCacheTrait, ProximaTrait};

#[derive(Clone, Debug)]
pub struct Proxima2Cache {
    pub elements: DMatrix<Proxima2CacheElement>
}
impl Proxima2Cache {
    pub fn new(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupProximityQueryMode, skips: Option<&DMatrix<bool>>, lie_alg_mode: LieAlgMode) -> Self {
        if let DoubleGroupProximityQueryMode::SubsetOfPairs(_) = query_mode {
            panic!("you probably shouldn't use a subset of pairs to initialize the Proxima cache.");
        }
        assert_eq!(group_a.len(), poses_a.len());
        assert_eq!(group_b.len(), poses_b.len());

        let mut elements = DMatrix::from_vec(group_a.len(), group_b.len(), vec![Proxima2CacheElement::new_default(lie_alg_mode); group_a.len()*group_b.len()]);

        for (i, (shape_a, pose_a)) in group_a.iter().zip(poses_a.iter()).enumerate() {
            'l: for (j, (shape_b, pose_b)) in group_b.iter().zip(poses_b.iter()).enumerate() {
                match skips {
                    None => {}
                    Some(skips) => {
                        if skips[(i,j)] { continue 'l; }
                    }
                }
                elements[(i,j)].update_element_with_ground_truth(shape_a, pose_a, shape_b, pose_b);
            }
        }

        Self { elements }
    }
}
impl ProximaCacheTrait<Proxima2CacheElement> for Proxima2Cache {
    fn elements(&self) -> &DMatrix<Proxima2CacheElement> {
        &self.elements
    }

    fn elements_mut(&mut self) -> &mut DMatrix<Proxima2CacheElement> {
        &mut self.elements
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct Proxima2CacheElement {
    pub gradient: V6,
    pub raw_distance_j: f64,
    pub disp_between_a_and_b_j: ISE3q,
    pub lie_alg_mode: LieAlgMode
}
impl Proxima2CacheElement {
    pub fn new_default(lie_alg_mode: LieAlgMode) -> Self {
        Self {
            gradient: Default::default(),
            raw_distance_j: 0.0,
            disp_between_a_and_b_j: Default::default(),
            lie_alg_mode,
        }
    }
}
impl ProximaCacheElementTrait for Proxima2CacheElement {
    #[inline(always)]
    fn update_element_with_ground_truth(&mut self, shape_a: &OffsetShape, pose_a: &ISE3q, shape_b: &OffsetShape, pose_b: &ISE3q) -> f64 {
        let disp = pose_a.displacement(&pose_b);
        let t = get_t(&disp, self.lie_alg_mode);
        let gradient_res = lladis_gradient(shape_a, shape_b, &t, self.lie_alg_mode);
        self.disp_between_a_and_b_j = disp;
        self.raw_distance_j = gradient_res.0;
        self.gradient = gradient_res.1;

        gradient_res.0
    }
}

#[derive(Clone, Debug)]
pub struct Proxima2 {
    pub cache: Proxima2Cache,
    pub lie_alg_mode: LieAlgMode
}
impl ProximaTrait for Proxima2 {
    type CacheType = Proxima2Cache;
    type CacheElementType = Proxima2CacheElement;
    type ExtraArgs = Proxima2ExtraArgs;

    #[inline(always)]
    fn get_extra_args(&self, _i: usize, _j: usize) -> Cow<Self::ExtraArgs> {
        Cow::Owned(Proxima2ExtraArgs {
            lie_alg_mode: self.lie_alg_mode,
        })
    }

    #[inline(always)]
    fn get_cache_mut(&mut self) -> &mut Self::CacheType {
        &mut self.cache
    }

    #[inline(always)]
    fn get_cache_immut(&self) -> &Self::CacheType {
        &self.cache
    }

    #[inline(always)]
    fn approximate_distance_and_bounds(cache_element: &Self::CacheElementType, pose_a_k: &ISE3q, pose_b_k: &ISE3q, _cutoff_distance: f64, extra_args: &Self::ExtraArgs) -> Option<(f64, f64, f64)> {
        let delta_t = get_delta_t(&cache_element.disp_between_a_and_b_j, &pose_a_k.displacement(&pose_b_k), extra_args.lie_alg_mode);
        let norm = delta_t.norm();
        let approximate_distance = cache_element.raw_distance_j + cache_element.gradient.dot(&delta_t);
        let lower_bound_delta = match extra_args.lie_alg_mode {
            LieAlgMode::Standard => { -0.3244893534761935*norm }
            LieAlgMode::Pseudo => { -0.33611397097921175*norm }
            // LieAlgMode::Standard => { -0.1*norm }
            // LieAlgMode::Pseudo => { -0.1*norm }
        };
        let upper_bound_delta = match extra_args.lie_alg_mode {
            LieAlgMode::Standard => { 1.561538215120098*norm }
            LieAlgMode::Pseudo => { 1.5894634675072785*norm }
            // LieAlgMode::Standard => { 0.1*norm }
            // LieAlgMode::Pseudo => { 0.1*norm }
        };

        let lower_bound = approximate_distance + lower_bound_delta;
        let upper_bound = approximate_distance + upper_bound_delta;

        // println!("{:?}", (approximate_distance, lower_bound, upper_bound));

        Some((approximate_distance, lower_bound, upper_bound))
    }
}

#[derive(Clone, Debug)]
pub struct Proxima2ExtraArgs {
    pub lie_alg_mode: LieAlgMode
}

#[inline(always)]
pub fn get_t(disp_between_a_and_b_j: &ISE3q, lie_alg_mode: LieAlgMode) -> V6 {
    return match lie_alg_mode {
        LieAlgMode::Standard => {
            disp_between_a_and_b_j.ln().vee()
        }
        LieAlgMode::Pseudo => {
            disp_between_a_and_b_j.pseudo_ln().vee()
        }
    }
}

#[inline(always)]
pub fn get_delta_t(disp_between_a_and_b_j: &ISE3q, disp_between_a_and_b_k: &ISE3q, lie_alg_mode: LieAlgMode) -> V6 {
    let disp = disp_between_a_and_b_j.displacement(disp_between_a_and_b_k);
    return match lie_alg_mode {
        LieAlgMode::Standard => {
            disp.ln().vee()
        }
        LieAlgMode::Pseudo => {
            disp.pseudo_ln().vee()
        }
    }
}

#[inline(always)]
pub fn lladis(shape_a: &OffsetShape, shape_b: &OffsetShape, t: &V6, lie_alg_mode: LieAlgMode) -> f64 {
    let pose = match lie_alg_mode {
        LieAlgMode::Standard => {
            t.to_lie_alg_ise3q().exp()
        }
        LieAlgMode::Pseudo => {
            t.to_pseudo_lie_alg_ise3q().exp()
        }
    };
    shape_a.contact(&ISE3q::identity(), shape_b, &pose, f64::INFINITY).unwrap().dist
}

#[inline(always)]
pub fn lladis_gradient(shape_a: &OffsetShape, shape_b: &OffsetShape, t: &V6, lie_alg_mode: LieAlgMode) -> (f64, V6) {
    let mut out = V6::zeros();

    let p = 0.00001;
    let f0 = lladis(shape_a, shape_b, t, lie_alg_mode);
    for i in 0..6 {
        let mut th = t.clone();
        th[i] += p;
        let fh = lladis(shape_a, shape_b, &th, lie_alg_mode);
        out[i] = (fh - f0) / p;
    }

    (f0, out)
}

pub fn get_lladis_taylor_series_error_dataset(shapes: &Vec<OffsetShape>, lie_alg_mode: LieAlgMode, num_random_pairs: usize, num_gradient_samples: usize, num_samples_per_gradient: usize, t_norm_max: f64, delta_t_norm_max: f64) -> Vec<(f64, f64)> {
    let mut out = vec![];

    assert!(shapes.len() >= 2);

    for _ in 0..num_random_pairs {
        let mut rng = rand::thread_rng();
        let mut i = 0;
        let mut j = 0;
        while i == j {
            i = rng.gen_range(0..shapes.len());
            j = rng.gen_range(0..shapes.len());
        }
        let shape_a = &shapes[i];
        let shape_b = &shapes[j];
        let ds = get_lladis_taylor_series_error_dataset_shape_pair(shape_a, shape_b, lie_alg_mode, num_gradient_samples, num_samples_per_gradient, t_norm_max, delta_t_norm_max);
        out.extend(ds);
    }

    out
}

/// the outputs here will be pairs (x,y), where x is the norm of delta_t, and y is the signed error between the ground truth and taylor series approximation
pub fn get_lladis_taylor_series_error_dataset_shape_pair(shape_a: &OffsetShape, shape_b: &OffsetShape, lie_alg_mode: LieAlgMode, num_gradient_samples: usize, num_samples_per_gradient: usize, t_norm_max: f64, delta_t_norm_max: f64) -> Vec<(f64, f64)> {
    let mut out = vec![];

    for _ in 0..num_gradient_samples {
        let mut t = V6::new_random_with_range(-1.0, 1.0);
        let s = V::new_random_with_range(1, 0.0, t_norm_max)[0];
        t = (t / t.norm()) * s;
        let gradient = lladis_gradient(shape_a, shape_b, &t, lie_alg_mode).1;
        for _ in 0..num_samples_per_gradient {
            let mut delta_t = V6::new_random_with_range(-1.0, 1.0);
            let s = V::new_random_with_range(1, 0.0, delta_t_norm_max)[0];
            delta_t = (delta_t / delta_t.norm()) * s;
            let gt = lladis(&shape_a, &shape_b, &(t + delta_t), lie_alg_mode);
            let ts = lladis(&shape_a, &shape_b, &t, lie_alg_mode) + gradient.dot(&delta_t);
            out.push( (delta_t.norm(), gt - ts) );
        }
    }

    out
}

#[derive(Clone, Debug, Copy)]
pub enum PolynomialFit {
    LinearNoIntercept,
    Linear,
    Quadratic,
    Cubic,
    Quartic
}
impl PolynomialFit {
    #[inline(always)]
    pub fn call(&self, coefficients: &[f64], x: f64) -> f64 {
        return match self {
            PolynomialFit::LinearNoIntercept => {
                coefficients[0] * x
            }
            PolynomialFit::Linear => {
                coefficients[0] + coefficients[1] * x
            }
            PolynomialFit::Quadratic => {
                coefficients[0] + coefficients[1] * x + coefficients[2] * x.powi(2)
            }
            PolynomialFit::Cubic => {
                coefficients[0] + coefficients[1] * x + coefficients[2] * x.powi(2) + coefficients[3] * x.powi(3)
            }
            PolynomialFit::Quartic => {
                coefficients[0] + coefficients[1] * x + coefficients[2] * x.powi(2) + coefficients[3] * x.powi(3) + coefficients[4] * x.powi(4)
            }
        }
    }

    pub fn num_dofs(&self) -> usize {
        match self {
            PolynomialFit::LinearNoIntercept => { 1 }
            PolynomialFit::Linear => { 2 }
            PolynomialFit::Quadratic => { 3 }
            PolynomialFit::Cubic => { 4 }
            PolynomialFit::Quartic => { 5 }
        }
    }
}

pub fn quantile_optimization(q: f64, dataset: &Vec<(f64, f64)>, polynomial_fit: &PolynomialFit) -> (Vec<f64>, SolverStatus) {
    assert!(0.0 < q && q < 1.0);

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        *c = quantile_optimization_objective(u, q, dataset, polynomial_fit);
        Ok(())
    };

    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        let g = quantile_optimization_objective_gradient(u, q, dataset, polynomial_fit);
        grad.iter_mut().enumerate().for_each(|(i, x)| { *x = g[i]; });
        Ok(())
    };

    let binding = NoConstraints::new();
    let problem = Problem::new(&binding, df, f);

    let n = polynomial_fit.num_dofs();
    let lbfgs_memory = 10;
    let tolerance = 1e-6;
    let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);

    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_tolerance(0.00005).with_max_iter(1000);

    let init_condition_v = V::new_random_with_range(n, -1.0, 1.0);
    let mut u = init_condition_v.data.as_slice().to_vec();

    let status = panoc.solve(&mut u).unwrap();

    (u, status)
}

pub fn quantile_optimization_objective(u: &[f64], q: f64, dataset: &Vec<(f64, f64)>, polynomial_fit: &PolynomialFit) -> f64 {
    let mut ff = 0.0;
    for d in dataset {
        let x = d.0;
        let y = d.1;
        let y_hat = polynomial_fit.call(u, x);
        if y >= y_hat {
            ff += q * (y - y_hat);
        } else {
            ff += (q - 1.0) * (y - y_hat);
        }
    }

    ff
}

pub fn quantile_optimization_objective_gradient(u: &[f64], q: f64, dataset: &Vec<(f64, f64)>, polynomial_fit: &PolynomialFit) -> Vec<f64> {
    let u = u.to_vec();

    let mut out = vec![0.0; u.len()];

    let p = 0.00001;
    let f0 = quantile_optimization_objective(&u, q, dataset, polynomial_fit);
    for i in 0..u.len() {
        let mut uh = u.clone();
        uh[i] += p;
        let fh = quantile_optimization_objective(&uh, q, dataset, polynomial_fit);
        out[i] = (fh - f0) / p;
    }

    out
}

#[derive(Clone, Debug, PartialEq, Eq, Copy)]
pub enum LieAlgMode {
    Standard,
    Pseudo
}

*/