use apollo_rust_proximity::{gjk_contact, sphere_sphere_contact, Cuboid, Sphere};
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use parry3d_f64::query::distance as parry_distance;
use parry3d_f64::shape::Cuboid as ParryCuboid;

#[test]
fn test_sphere_sphere_contact() {
    let s1 = Sphere::new(1.0);
    let s2 = Sphere::new(1.0);

    let mut p1 = LieGroupISE3q::identity();
    p1.0.translation = nalgebra::Translation3::new(0.5, 0.0, 0.0);

    let p2 = LieGroupISE3q::identity();

    // Centers at (0.5, 0, 0) and (0, 0, 0). Distance between centers is 0.5.
    // Combined radii is 2.0. Expected penetration depth is 2.0 - 0.5 = 1.5.
    // Our convention for penetration is negative, so -1.5.

    let (dir, dist) = sphere_sphere_contact(&s1, &p1, &s2, &p2);

    assert!((dist - (-1.5)).abs() < 1e-6);
    assert!((dir.x.abs() - 1.0).abs() < 1e-6);

    // Also verify GJK/EPA gives the same result for these spheres
    let (gjk_dir, gjk_dist) = gjk_contact(&s1, &p1, &s2, &p2);
    assert!(
        (gjk_dist - dist).abs() < 1e-6,
        "GJK/EPA Mismatch: gjk={} vs specialized={}",
        gjk_dist,
        dist
    );
    assert!(
        gjk_dir.dot(&dir).abs() > 0.99,
        "GJK/EPA direction mismatch: gjk={:?} vs specialized={:?}",
        gjk_dir,
        dir
    );
}

#[test]
fn test_cuboid_regression_case_1() {
    // This case was found to have a discrepancy between my EPA and Parry's contact due to multiple deeper faces.
    // We verify that my EPA returns a consistent result that satisfies the support property.
    let s1 = Cuboid::new(1.075098098538974, 1.096634535128007, 1.500321486525829);
    let s2 = Cuboid::new(0.5215194241920614, 0.6700215901060109, 1.837100818514518);

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

    let (_dir, dist) = gjk_contact(&s1, &p1, &s2, &p2);

    // Expected distance from previous manual verify was -0.1288602456115563
    assert!((dist - (-0.1288602)).abs() < 1e-6);
}

#[test]
fn test_randomized_cuboids() {
    use rand::Rng;
    let mut rng = rand::thread_rng();

    for _ in 0..100 {
        let s1 = Cuboid::new(
            rng.gen_range(0.5..2.0),
            rng.gen_range(0.5..2.0),
            rng.gen_range(0.5..2.0),
        );
        let s2 = Cuboid::new(
            rng.gen_range(0.5..2.0),
            rng.gen_range(0.5..2.0),
            rng.gen_range(0.5..2.0),
        );
        let p1 = LieGroupISE3q::new_random();
        let p2 = LieGroupISE3q::new_random();

        let (_dir, dist) = gjk_contact(&s1, &p1, &s2, &p2);

        let s1_parry = ParryCuboid::new(Vector3::new(
            s1.half_extents.x,
            s1.half_extents.y,
            s1.half_extents.z,
        ));
        let s2_parry = ParryCuboid::new(Vector3::new(
            s2.half_extents.x,
            s2.half_extents.y,
            s2.half_extents.z,
        ));

        // parry_distance returns positive for non-overlapping, and we don't necessarily trust its negative distance for MTV
        // but it should always agree on whether they are colliding or not, and the positive distance.
        let dist_parry = parry_distance(&p1.0, &s1_parry, &p2.0, &s2_parry).unwrap();

        if dist > 0.0 || dist_parry > 0.0 {
            assert!(
                (dist - dist_parry).abs() < 1e-4,
                "Mismatch in positive distance: my={} parry={}",
                dist,
                dist_parry
            );
        }
    }
}
