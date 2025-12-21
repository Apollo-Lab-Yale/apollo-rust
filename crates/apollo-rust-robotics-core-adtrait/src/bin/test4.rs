use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardAD, ReverseAD};
use ad_trait::function_engine::FunctionEngine;
use nalgebra::{Matrix3, Rotation3};
// use nalgebra::{Matrix3, Rotation3};
use apollo_rust_linalg_adtrait::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};
use apollo_rust_spatial_adtrait::vectors::V3;

pub struct SVDFunc;
impl<T: AD> DifferentiableFunctionTrait<T> for SVDFunc {
    const NAME: &'static str = "SVDFunc";

    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
        let m = M::new(inputs, 3, 3);
        let svd = m.svd(true, true);
        let r = svd.u.unwrap() * svd.v_t.unwrap();
        return r.data.as_slice().to_vec()
    }

    fn num_inputs(&self) -> usize {
        9
    }

    fn num_outputs(&self) -> usize {
        9
    }
}

pub struct Orth6DFunc;
impl<T: AD> DifferentiableFunctionTrait<T> for Orth6DFunc {
    const NAME: &'static str = "Orth6DFunc";

    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
        let mut v1 = V3::new(inputs[0], inputs[1], inputs[2]);
        let mut v2 = V3::new(inputs[3], inputs[4], inputs[5]);

        v1 = v1 / v1.norm();
        v2 = v2 / v2.norm();

        let x = v1.clone();
        let mut y = x.cross(&v2);
        y = y / y.norm();
        let mut z = x.cross(&y);
        z = z / z.norm();

        vec![ x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2] ]
    }

    fn num_inputs(&self) -> usize {
        6
    }

    fn num_outputs(&self) -> usize {
        9
    }
}

pub struct EulerFunc;
impl<T: AD> DifferentiableFunctionTrait<T> for EulerFunc {
    const NAME: &'static str = "EulerFunc";

    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
        let r = Rotation3::from_euler_angles(inputs[0], inputs[1], inputs[2]);
        return r.matrix().data.as_slice().to_vec()
    }

    fn num_inputs(&self) -> usize {
        3
    }

    fn num_outputs(&self) -> usize {
        9
    }
}

fn main() {

    let de = FunctionEngine::new(SVDFunc, SVDFunc, ForwardAD::new());

    let mut largest = -100000000.0;
    let mut smallest = 100000000.0;

    for _ in 0..1000000 {
        let v = V::<f64>::new_random(9);
        // let v = V::<f64>::new(&[0.0, 1.54, 0.0]);

        let res = de.derivative(v.as_slice());

        // let dv = V::<f64>::new_random_with_range(3, -1.0, 1.0);
        // let dr = res.1.clone() * dv;
        // let drm = Matrix3::from_row_slice(&dr.as_slice());
        // let rm = Matrix3::from_row_slice(&res.0.as_slice());

        let svd = res.1.svd(true, true);
        // println!("{}", svd.u.unwrap());
        // println!("{}", svd.v_t.unwrap());
        // println!("{}", svd.singular_values);
        let c = svd.singular_values[0] / svd.singular_values[2];
        // println!("{:?}", c);
        if c < smallest { smallest = c; }
        if c > largest { largest = c; }
    }

    println!("smallest: {:?}", smallest);
    println!("largest: {:?}", largest);

}