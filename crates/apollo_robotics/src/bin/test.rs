use std::time::Instant;
use nalgebra::{Complex, Isometry2, Isometry3, IsometryMatrix3, Matrix3, Quaternion, Rotation3, SMatrix, UnitDualQuaternion, UnitQuaternion};

fn main() {
    // let a = Rotation3::from_euler_angles(1.,2.,3.);
    // let b = SMatrix::<f64, 3, 3>::identity();
    // let mut c = SMatrix::<f64, 3, 3>::identity();
    // let mut c = Rotation3::from_euler_angles(1.,2.,3.);

    let a = UnitQuaternion::from_euler_angles(1.,2.,3.);
    let b = UnitQuaternion::from_euler_angles(1.,2.,3.);
    let mut c = UnitQuaternion::from_euler_angles(1.,2.,3.);
    let mut d = Quaternion::new(1.,2.,3.,4.);

    let start = Instant::now();
    for _ in 0..1000 {
        // c += a + 2.0*(a + b) + Matrix3::new(1.,2.,3.,4.,5.,6.,7.,8.,9.);
        // c += (c.try_inverse().unwrap() + d);
        // c = c*a;
        d = c.ln();
    }
    println!("{:?}", start.elapsed());
    println!("{}", d);
}