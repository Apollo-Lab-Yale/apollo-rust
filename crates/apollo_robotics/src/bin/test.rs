use std::ops::Mul;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

fn main() {
    let q = UnitQuaternion::from_euler_angles(1.,2.,3.);
    let bw = Vector3::new(1.,4.,5.);
    let bx = Vector3::new(4.,3.,2.);

    let res1 = q*(bw.cross(&bx));
    println!("{}", res1);

    let c = bw.cross(&bx);
    println!("{:?}", c);

    // let qx = Quaternion::new(0.0, 4.,3.,2.);
    // let qb = Quaternion::new(0.0, 1.,4.,5.);
    // println!("{}", qx*qb);
    // let qq = Quaternion::new(0.0, tt.x, tt.y, tt.z);
    // println!("{}", q.quaternion().mul(&qq)*q.quaternion().try_inverse().unwrap());

    let qdot = q.quaternion()*Quaternion::new(0.0, bw.x, bw.y, bw.z);
    println!("{}", qdot);

    let res2 = qdot*Quaternion::new(0.0, bx.x, bx.y, bx.z);
    println!("{}", res2);
}