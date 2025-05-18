use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DifferentiableFunctionClass, DifferentiableFunctionTrait, FiniteDifferencing, ForwardAD, ReverseAD};
use apollo_rust_lie_adtrait::{EuclideanSpaceElement, LieAlgebraElement};
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::ApolloLieAlgPackIse3qTrait;
use apollo_rust_spatial_adtrait::vectors::{V3, V6};

pub struct Test;
impl<A: AD> DifferentiableFunctionTrait<A> for Test {
    fn call(&self, inputs: &[A], _freeze: bool) -> Vec<A> {
        // let v = V6::from_column_slice(inputs);
        // let res = v.to_lie_alg_ise3q().exp();
        return vec![inputs[0].powi(3)]
    }

    fn num_inputs(&self) -> usize {
        6
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

pub struct TestDC;
impl DifferentiableFunctionClass for TestDC {
    type FunctionType<T: AD> = Test;
}


fn main() {
    let db1 = DifferentiableBlock::new_with_tag(TestDC, ReverseAD::new(), Test, Test);
    let db2 = DifferentiableBlock::new_with_tag(TestDC, ForwardAD::new(), Test, Test);
    let db3 = DifferentiableBlock::new_with_tag(TestDC, FiniteDifferencing::new(), Test, Test);

    let state = [2., 2., 3., 4., 5., 6.];

    let res1 = db1.derivative(&state);
    let res2 = db2.derivative(&state);
    let res3 = db3.derivative(&state);

    println!("{:?}", res1.1);
    println!();
    println!("{:?}", res2.1);
    println!();
    println!("{:?}", res3.1);
}