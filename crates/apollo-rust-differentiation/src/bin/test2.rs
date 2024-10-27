use apollo_rust_differentiation::{DifferentiableFunctionEngineNalgebraTrait, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::FDDifferentiableFunctionEngine;
use apollo_rust_linalg::{ApolloDVectorTrait, V};

pub struct Test;
impl FunctionNalgebraTrait for Test {
    fn call_raw(&self, x: &V) -> V {
        V::new(&[ x[0].sin() + x[1].cos()*x[2].sin() ])
    }

    fn input_dim(&self) -> usize {
        3
    }

    fn output_dim(&self) -> usize {
        1
    }
}

fn directional_derivative<F: DifferentiableFunctionEngineNalgebraTrait>(x_k: &V, dir: &V, f: &F) -> V {
    let epsilon = 0.00001;
    let f_x_k = f.call(x_k);
    let f_x_k_delta = f.call(&(x_k + epsilon*dir));
    return (f_x_k_delta - f_x_k) / epsilon;
}

fn main() {
    let f = FDDifferentiableFunctionEngine::new(Test, 0.00001);
    let x_k = V::new(&[0.1,0.2,0.3]);

    let delta_x_1 = V::new_random_with_range(3, -1.0, 1.0);
    let delta_x_2 = V::new_random_with_range(3, -1.0, 1.0);

    let res1 = directional_derivative(&x_k, &delta_x_1, &f);
    let res2 = directional_derivative(&x_k, &delta_x_2, &f);

    println!("{}", delta_x_1);
    println!("{}", delta_x_2);

    println!("{}", res1);
    println!("{}", res2);

    println!("{:?}", delta_x_1.dot(&delta_x_2));
    println!("{:?}", res1.dot(&res2));
}