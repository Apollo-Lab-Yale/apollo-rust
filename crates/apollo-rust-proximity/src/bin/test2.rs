use apollo_rust_proximity::{gjk_contact, Cuboid, ShapeTrait};
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::vectors::V3;
use nalgebra::{Quaternion, UnitQuaternion};
use parry3d_f64::query::{contact as parry_contact, distance as parry_distance};
use parry3d_f64::shape::{Cuboid as ParryCuboid, SupportMap};
use std::ops::Neg;

fn main() {
    let s1 = Cuboid::new(1.075098098538974, 1.096634535128007, 1.500321486525829);
    let s2 = Cuboid::new(0.5215194241920614, 0.6700215901060109, 1.837100818514518);

    /*
    let s1 = Cuboid::new(0.9224848726422733,
                         0.6450843912586752 ,
                         1.7245976540032735);
    let s2 =  Cuboid::new( 0.9518835153328179, 1.4275913280581842, 1.9763082564025916);

     */
    let mut p1 = LieGroupISE3q::identity();
    p1.0.rotation = UnitQuaternion::new_normalize(Quaternion::new(
        0.12521312636263338,
        0.05320068964272308,
        -0.9479505940465489,
        -0.2878906578109627,
    ));
    p1.0.translation = nalgebra::Translation3::new(
        -0.20384870881084893,
        1.4837239790061565,
        -1.9428668424691602,
    );
    let mut p2 = LieGroupISE3q::identity();
    p2.0.rotation = UnitQuaternion::new_normalize(Quaternion::new(
        0.3202799675421666,
        0.5046631055433424,
        -0.32184338669003126,
        -0.7342702000887882,
    ));
    p2.0.translation =
        nalgebra::Translation3::new(-1.985337327415627, 0.20894124790838386, 0.20201679374059456);
    let (dir, dist) = gjk_contact(&s1, &p1, &s2, &p2);
    let _collided = dist <= 0.0;
    let s1_parry = ParryCuboid::new(nalgebra::Vector3::new(
        s1.half_extents.x,
        s1.half_extents.y,
        s1.half_extents.z,
    ));
    let s2_parry = ParryCuboid::new(nalgebra::Vector3::new(
        s2.half_extents.x,
        s2.half_extents.y,
        s2.half_extents.z,
    ));
    let mut dist0 = parry_distance(&p1.0, &s1_parry, &p2.0, &s2_parry).unwrap();
    let collided0 = dist0 <= 0.0;
    let mut dir0 = V3::zeros();
    if collided0 {
        let contact_res = parry_contact(&p1.0, &s1_parry, &p2.0, &s2_parry, 0.0).unwrap();
        if let Some(c) = contact_res {
            dist0 = c.dist;
            dir0 = V3::from_array_storage(c.normal1.data);
        }
    }
    let my_support1 = s1.support(&dir0, &p1);
    let my_support2 = s2.support(&dir0.neg(), &p2);
    let parry_dir_vec = nalgebra::Vector3::new(dir0.x, dir0.y, dir0.z);
    let parry_support1 = s1_parry.support_point(&p1.0, &parry_dir_vec);
    let parry_support2 = s2_parry.support_point(&p2.0, &parry_dir_vec.neg());
    println!(
        "My support1: {:?}, support2: {:?}",
        my_support1.transpose(),
        my_support2.transpose()
    );
    println!(
        "Parry support1: {:?}, support2: {:?}",
        parry_support1.coords.transpose(),
        parry_support2.coords.transpose()
    );
    let my_support_dist = (my_support1 - my_support2).dot(&dir0);
    let parry_support_dist = (parry_support1.coords - parry_support2.coords).dot(&parry_dir_vec);
    println!(
        "My support dist in dir0: {}, Parry support dist in dir0: {}",
        my_support_dist, parry_support_dist
    );
    let diff = (dist - dist0).abs();
    println!(
        "my = {} vs parry = {} (diff = {}), my_dir={:?}, parry_dir={:?}",
        dist,
        dist0,
        diff,
        dir.transpose(),
        dir0.transpose(),
    );
}
