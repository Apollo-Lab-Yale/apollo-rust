use std::cmp::Reverse;
use std::time::Instant;
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionClass, DifferentiableFunctionTrait, FiniteDifferencing, ForwardAD, ForwardADMulti, ReverseAD};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::reverse_ad::adr::{adr, GlobalComputationGraph};
use apollo_rust_linalg_adtrait::{ApolloDMatrixTrait, ApolloDVectorTrait, V};
use apollo_rust_spatial_adtrait::vectors::{V3, V6};
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_robotics_core_adtrait::ChainNalgebraADTrait;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_linalg_adtrait::{M, SVDType, SVDResult};
use apollo_rust_lie_adtrait::{LieGroupElement, LieAlgebraElement};
use apollo_rust_robotics_adtrait::ToChainNalgebraADTrait;
use apollo_rust_spatial_adtrait::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial_adtrait::lie::h1::ApolloUnitQuaternionH1LieTrait;

#[derive(Clone)]
pub struct BenchmarkIK<T:AD>{
    chain: ChainNalgebraADTrait<T>,
    target_foot1_pos: V3<T>,
    target_foot2_pos: V3<T>,
    target_foot3_pos: V3<T>,
    target_foot4_pos: V3<T>,
    target_ee_pos: LieGroupISE3q<T>,
}

impl <T:AD> BenchmarkIK<T>{
    pub fn new()->Self{
        // let dir=ResourcesRootDirectory::new_from_default_apollo_robots_dir();
        // let chain:ChainNalgebraADTrait<T> = dir.get_subdirectory("b1z1").to_chain_nalgebra_adtrait();
        let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
        let s = r.get_subdirectory("b1z1");
        let chain = s.to_chain_nalgebra_adtrait::<f64>().to_other_ad_type::<T>();
        Self{
            chain,
            target_foot1_pos: V3::new(0.45.into(), (-0.2).into(), 0.0.into()),
            target_foot2_pos: V3::new(0.45.into()  , 0.2.into()  , 0.0.into()  ),
            target_foot3_pos: V3::new((-0.2).into()   , (-0.2).into()  , 0.0.into()  ),
            target_foot4_pos: V3::new((-0.2).into()  , 0.2.into()  , 0.0.into()  ),
            // target_ee_pos: LieGroupISE3q::from_exponential_coordinates(&V6::new(0.0.into()  , 0.0.into()  , 0.0.into()  ,
            //                                                                     0.3.into()  , 0.0.into()  , 1.15.into()  ))
            target_ee_pos: LieGroupISE3q::new(I3::from_slices_euler_angles(&[0.3.into(), 0.0.into(), 1.15.into()], &[0.0.into(), 0.0.into(), 0.0.into()]))
        }
    }
}

impl <T:AD> DifferentiableFunctionTrait<T> for BenchmarkIK<T>{
    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
        // let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
        // let s = r.get_subdirectory("b1z1");
        // let c1 = s.to_chain_nalgebra_adtrait::<f64>().to_other_ad_type::<T>();
        let res = self.chain.fk(&V::from_column_slice(inputs));
        let t1 = (res[10].0.translation.vector - self.target_foot1_pos).norm();
        let t2 = (res[17].0.translation.vector - self.target_foot2_pos).norm();
        let t3 = (res[24].0.translation.vector - self.target_foot3_pos).norm();
        let t4 = (res[31].0.translation.vector - self.target_foot4_pos).norm();
        // let t5 = LieGroupISE3q::new(res[39].0.inverse()*self.target_ee_pos.0).ln().vee().norm();
        let t5 = (res[39].0.translation.vector - self.target_ee_pos.0.translation.vector).norm();
        let t6 = res[39].0.rotation.to_lie_group_h1().displacement(&self.target_ee_pos.0.rotation.to_lie_group_h1()).ln().vee().norm();

        vec![t1*t1, t2*t2, t3*t3, t4*t4, t5*t5, t6*t6]
    }

    fn num_inputs(&self) -> usize {
        self.chain.num_dofs()
    }

    fn num_outputs(&self) -> usize {
        6
    }
}

pub struct DCBenchmarkIK;
impl DifferentiableFunctionClass for DCBenchmarkIK {
    type FunctionType<T: AD> = BenchmarkIK<T>;
}

fn pseudo_inverse<T: AD>(mat: &M<T>, eps: T) -> M<T> {
    let svd = mat.singular_value_decomposition(SVDType::Full);
    let s = svd.singular_values();
    let u = svd.u();
    let vt = svd.vt();
    let mut s_pinv = M::<T>::zeros(vt.nrows(), u.ncols());
    let min_dim = std::cmp::min(s_pinv.nrows(), s_pinv.ncols());
    for i in 0..std::cmp::min(s.len(), min_dim) {
        if s[i] > eps {
            s_pinv[(i, i)] = T::one().div(s[i]);
        } else {
            s_pinv[(i, i)] = T::zero();
        }
    }

    vt.transpose() * s_pinv * u.transpose()
}

pub fn simple_pseudoinverse_newtons_method_ik<E:DerivativeMethodTrait>(ad_method: E, init_cond: V<f64>, max_iter: usize, step_length: f64, threshold: f64) -> f64{
    let ik = BenchmarkIK::<f64>::new();
    let ik_d = BenchmarkIK::<E::T>::new();
    let ad_engine = DifferentiableBlock::new_with_tag(DCBenchmarkIK, ad_method, ik, ik_d);
    let mut q = init_cond;
    let start = Instant::now();
    let mut y = V::<f64>::from(ad_engine.call(q.as_slice()));
    for _ in 0..max_iter {
        let (_, jacobian) = ad_engine.derivative(q.as_slice());
        let delta_q = pseudo_inverse(&jacobian, 1e-10)*y;
        q = q - step_length*delta_q;
        y = V::<f64>::from(ad_engine.call(q.as_slice()));
        if y.norm() < threshold {break;}
    }
    let duration = start.elapsed().as_secs_f64();
    duration
}

pub fn benchmark_eval2(){
    let passes = 1;
    let ik = BenchmarkIK::<f64>::new();
    let mut durs_fad =Vec::<f64>::new();
    let mut durs_rad =Vec::<f64>::new();
    let mut durs_fd =Vec::<f64>::new();
    let mut durs_mcfad =Vec::<f64>::new();
    for i in 0..passes {
        println!("Pass {i} running...");
        let init_cond = V::<f64>::new_random_with_range(ik.num_inputs(),-0.2,0.2);
        durs_fad.push(simple_pseudoinverse_newtons_method_ik(ForwardAD::new(), init_cond.clone(), 10000,0.01, 0.01));
        GlobalComputationGraph::get().reset();
        durs_rad.push(simple_pseudoinverse_newtons_method_ik(ReverseAD::new(), init_cond.clone(), 10000,0.01, 0.01));
        // durs_fd.push(simple_pseudoinverse_newtons_method_ik(FiniteDifferencing::new(), init_cond.clone(), 10000,0.01, 0.01));
        // durs_mcfad.push(simple_pseudoinverse_newtons_method_ik(ForwardADMulti::<adfn<24>>::new(), init_cond.clone(), 10000,0.01, 0.01));
    }
    // println!("Forward AD:, (avg_time, std_time)={:?}", calculate_stats(&durs_fad));
    // println!("Reverse AD: (avg_time, std_time)={:?}", calculate_stats(&durs_rad));
    //println!("Finite Diff:, (avg_time, std_time)={:?}", calculate_stats(&durs_fd));
    //println!("Multi Channel Forward AD:,  (avg_time, std_time)={:?}", calculate_stats(&durs_mcfad));
}

fn main() {
    // let start = Instant::now();
    // benchmark_eval2();
    // println!("{:?}", start.elapsed());

    let d1 = DifferentiableBlock::new_with_tag(DCBenchmarkIK, ReverseAD::new(), BenchmarkIK::<f64>::new(), BenchmarkIK::<adr>::new());
    let d2 = DifferentiableBlock::new_with_tag(DCBenchmarkIK, ForwardAD::new(), BenchmarkIK::<f64>::new(), BenchmarkIK::<adfn<1>>::new());
    let d3 = DifferentiableBlock::new_with_tag(DCBenchmarkIK, FiniteDifferencing::new(), BenchmarkIK::<f64>::new(), BenchmarkIK::<f64>::new());

    GlobalComputationGraph::get().reset();
    let res1 = d1.derivative(&[0.1; 24]);
    let res2 = d2.derivative(&[0.1; 24]);
    let res3 = d3.derivative(&[0.1; 24]);

    println!("{:?}", res1.1);
    println!();
    println!("{:?}", res2.1);
    println!();
    println!("{:?}", res3.1);
}