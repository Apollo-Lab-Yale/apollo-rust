use ad_trait::AD;
// use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardAD, ReverseAD};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::reverse_ad::adr::adr;
use apollo_rust_lie_adtrait::{EuclideanSpaceElement, LieAlgebraElement, LieGroupElement};
use apollo_rust_linalg_adtrait::V;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics_adtrait::ToChainNalgebraADTrait;
use apollo_rust_robotics_core_adtrait::ChainNalgebraADTrait;
use apollo_rust_spatial_adtrait::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial_adtrait::lie::h1::ApolloUnitQuaternionH1LieTrait;
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::{ISE3q, LieGroupISE3q};
use apollo_rust_spatial_adtrait::quaternions::UQ;
use apollo_rust_spatial_adtrait::vectors::V3;

pub struct Test<A: AD> {
    pub robot: ChainNalgebraADTrait<A>,
}
impl<A: AD> DifferentiableFunctionTrait<A> for Test<A> {
    const NAME: &'static str = "Test";

    fn call(&self, inputs: &[A], _freeze: bool) -> Vec<A> {
        let res = self.robot.fk(&V::from_column_slice(inputs));
        let t1 =
            (res[10].0.translation.vector - V3::new(0.45.into(), (-0.2).into(), 0.0.into())).norm();
        let t2 =
            (res[17].0.translation.vector - V3::new(0.45.into(), 0.2.into(), 0.0.into())).norm();
        let t3 = (res[24].0.translation.vector - V3::new((-0.2).into(), (-0.2).into(), 0.0.into()))
            .norm();
        let t4 =
            (res[31].0.translation.vector - V3::new((-0.2).into(), 0.2.into(), 0.0.into())).norm();
        let target_pose = LieGroupISE3q::new(I3::from_slices_euler_angles(
            &[0.3.into(), 0.0.into(), 1.15.into()],
            &[0.0.into(), 0.0.into(), 0.0.into()],
        ));
        let t5 = (res[39].0.translation.vector - target_pose.0.translation.vector).norm();
        let t6 = res[39]
            .0
            .rotation
            .to_lie_group_h1()
            .displacement(&target_pose.0.rotation.to_lie_group_h1())
            .ln()
            .vee()
            .norm();

        vec![t1 * t1, t2 * t2, t3 * t3, t4 * t4, t5 * t5, t6 * t6]
    }

    fn num_inputs(&self) -> usize {
        self.robot.num_dofs()
    }

    fn num_outputs(&self) -> usize {
        6
    }
}

/*
pub struct TestDC;
impl DifferentiableFunctionClass for TestDC {
    type FunctionType<T: AD> = Test<T>;
}
*/

/*
fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1z1");
    let c1 = s.to_chain_nalgebra_adtrait::<f64>();
    let c2 = s.to_chain_nalgebra_adtrait::<adr>();
    let c3 = s.to_chain_nalgebra_adtrait::<adfn<1>>();

    let db1 = DifferentiableBlock::new_with_tag(
        TestDC,
        ReverseAD::new(),
        Test { robot: c1.clone() },
        Test { robot: c2.clone() },
    );
    let db2 = DifferentiableBlock::new_with_tag(
        TestDC,
        ForwardAD::new(),
        Test { robot: c1.clone() },
        Test { robot: c3.clone() },
    );

    println!("{:?}", db1.derivative(&[0.1; 24]).1);
    println!();
    println!("{:?}", db2.derivative(&[0.1; 24]).1);
}
*/
fn main() {}
