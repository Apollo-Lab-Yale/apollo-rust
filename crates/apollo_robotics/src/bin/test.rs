use std::time::Instant;
use nalgebra::{Isometry3, Vector3};
use parry3d::query::{contact, distance};
use parry3d::shape::{Ball, Cuboid};

fn main() {
    // let a = Cuboid::new(Vector3::new(0.2,0.5,0.1));
    // let b = Cuboid::new(Vector3::new(0.2,0.7,0.3));
    let a = Ball::new(0.5);
    let b = Ball::new(0.3);

    let v = Vector3::new(1.,2.,3.);

    let start = Instant::now();
    for i in 0..1000 {
        // let d = contact(&Isometry3::new(Vector3::new(1.,2.,3.), Vector3::new(3.,2.,1.)), &a, &Isometry3::new(Vector3::new(3.,2.,1.), Vector3::new(1.,2.,3.)), &b, 1000.0);
        // let d = Isometry3::new(Vector3::new(1.,2.,3.), Vector3::new(3.,2.,1.))*v;
    }
    println!("{:?}", start.elapsed());

    // println!("{:?}", d);
}