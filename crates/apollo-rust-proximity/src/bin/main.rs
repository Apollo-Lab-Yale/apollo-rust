use parry3d_f64::shape::Cuboid;
use apollo_rust_proximity::offset_shape::OffsetShape;
use apollo_rust_proximity::proxima::proxima2::{get_lladis_taylor_series_error_dataset, LieAlgMode, PolynomialFit, quantile_optimization};
use apollo_rust_spatial::vectors::{ V3,};

fn main() {
    let shape_a = OffsetShape::new(Cuboid::new(V3::new(0.1, 0.1, 0.1)), None);
    let shape_b = OffsetShape::new(Cuboid::new(V3::new(0.1, 0.1, 0.1)), None);

    // let pose_a = ISE3q::new_random();
    // let pose_b = ISE3q::new_random();

    let ds = get_lladis_taylor_series_error_dataset(&shape_a, &shape_b, LieAlgMode::Pseudo, 100, 100, 1.0, 2.0);

    let res = quantile_optimization(0.5, &ds, &PolynomialFit::Linear);
    println!("{:?}", res);

    let res = quantile_optimization(0.5, &ds, &PolynomialFit::Quadratic);
    println!("{:?}", res);
}

