use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use crate::shape::ShapeTrait;



pub fn gjk_intersection(
    shape_a: &impl ShapeTrait,
    shape_b: &impl ShapeTrait,
    pose_a: &ISE3q,
    pose_b: &ISE3q
) -> bool {
    let mut d = pose_b.0.translation.vector - pose_a.0.translation.vector;
    if d.norm() < 1e-12 {
        d = V3::new(1.0, 0.0, 0.0);
    }

    let mut simplex = Vec::new();
    simplex.push(support(shape_a, shape_b, pose_a, pose_b, d));

    d = -simplex[0];

    let mut iterations = 0;
    const MAX_ITER: usize = 100;

    loop {
        iterations += 1;
        if iterations > MAX_ITER {
            return true;
        }

        let new_point = support(shape_a, shape_b, pose_a, pose_b, d);
        if simplex.iter().any(|&p| (new_point - p).norm() < 1e-6) {
            return true;
        }

        if new_point.dot(&d) <= 0.0 {
            return false;
        }
        simplex.push(new_point);

        if update_simplex(&mut simplex, &mut d) {
            return true;
        }
    }
}

/// Returns the distance between two shapes.
/// If the shapes intersect, returns 0.0.
pub fn gjk_distance(
    shape_a: &impl ShapeTrait,
    shape_b: &impl ShapeTrait,
    pose_a: &ISE3q,
    pose_b: &ISE3q,
) -> f64 {
    // Start with an initial direction.
    let mut d = pose_b.0.translation.vector - pose_a.0.translation.vector;
    if d.norm() < 1e-12 {
        d = V3::new(1.0, 0.0, 0.0);
    }

    // Initialize the simplex with one support point.
    let mut simplex = vec![support(shape_a, shape_b, pose_a, pose_b, d)];
    // New search direction is toward the origin.
    d = -simplex[0];

    const MAX_ITER: usize = 100;
    const TOL: f64 = 1e-6;

    for _ in 0..MAX_ITER {
        let new_point = support(shape_a, shape_b, pose_a, pose_b, d);
        // If the new point isn’t “past” the current closest feature,
        // then we have reached convergence.
        if new_point.dot(&d) < TOL {
            return d.norm();
        }
        simplex.push(new_point);

        // Compute the closest point on the current simplex to the origin.
        let (closest, new_simplex) = closest_point_to_origin(&simplex);
        simplex = new_simplex;
        d = -closest; // New search direction is from the closest point to the origin.

        // If the closest point is very near the origin, then the shapes intersect.
        if d.norm() < TOL {
            return 0.0;
        }
    }
    d.norm()
}

pub fn support(
    shape_a: &impl ShapeTrait,
    shape_b: &impl ShapeTrait,
    pose_a: &ISE3q,
    pose_b: &ISE3q,
    d: V3
) -> V3 {
    let support_a = shape_a.support_function(&d, pose_a);
    let support_b = shape_b.support_function(&-d, pose_b);
    let res = support_a - support_b;
    return res;
}

fn update_simplex(simplex: &mut Vec<V3>, d: &mut V3) -> bool {
    match simplex.len() {
        2 => {
            // Handle the line case.
            // Let A be the last added point and B be the other.
            let a = simplex[1];
            let b = simplex[0];
            let ab = b - a;
            let ao = -a;

            // Check if the origin is in the direction of AB.
            if ab.dot(&ao) > 0.0 {
                let new_d = ab.cross(&ao.cross(&ab));
                // let new_d = ab * (ao.dot(&ab)) - ao * (ab.dot(&ab));
                // If the new direction is nearly zero, the origin is on the line.
                if new_d.norm() < 1e-12 {
                    return true;
                } else {
                    *d = new_d;
                }
            } else {
                // The origin is in the direction of A only.
                simplex.clear();
                simplex.push(a);
                *d = ao;
                if d.norm() < 1e-12 {
                    return true;
                }
            }
        }
        3 => {
            // Handle the triangle case.
            let a = simplex[2];
            let b = simplex[1];
            let c = simplex[0];
            let ab = b - a;
            let ac = c - a;
            let ao = -a;

            let abc = ab.cross(&ac);
            let ab_perp = abc.cross(&ab);
            let ac_perp = ac.cross(&abc);

            if ab_perp.dot(&ao) > 0.0 {
                simplex.remove(0); // remove C
                *d = ab_perp;
            } else if ac_perp.dot(&ao) > 0.0 {
                simplex.remove(1); // remove B
                *d = ac_perp;
            } else {
                if abc.dot(&ao) > 0.0 {
                    *d = abc;
                } else {
                    simplex.swap(0, 1);
                    *d = -abc;
                }
            }

            if d.norm() < 1e-12 {
                return true;
            }
        }
        4 => {
            // Handle the tetrahedron case.
            let a = simplex[3];
            let b = simplex[2];
            let c = simplex[1];
            let d_point = simplex[0];
            let ao = -a;

            // Face ABC:
            let ab = b - a;
            let ac = c - a;
            let abc = ab.cross(&ac);
            if abc.dot(&ao) > 0.0 {
                simplex.remove(0); // Remove D.
                *d = abc + 0.001*V3::new_random();
                if d.norm() < 1e-12 {
                    return true;
                }
                return false;
            }

            // Face ACD:
            let ac = c - a;
            let ad = d_point - a;
            let acd = ac.cross(&ad);
            if acd.dot(&ao) > 0.0 {
                simplex.remove(2); // Remove B.
                *d = acd + 0.001*V3::new_random();
                if d.norm() < 1e-12 {
                    return true;
                }
                return false;
            }

            // Face ADB:
            let ad = d_point - a;
            let ab = b - a;
            let adb = ad.cross(&ab);
            if adb.dot(&ao) > 0.0 {
                simplex.remove(1); // Remove C.
                *d = adb + 0.001*V3::new_random();
                if d.norm() < 1e-12 {
                    return true;
                }
                return false;
            }

            // If the origin is not outside any face, it is inside the tetrahedron.
            return true;
        }
        _ => {
            unreachable!("Simplex has an unexpected number of points")
        }
    }
    false
}

fn closest_point_to_origin(simplex: &[V3]) -> (V3, Vec<V3>) {
    match simplex.len() {
        1 => (simplex[0], vec![simplex[0]]),
        2 => {
            // Simplex is a line segment.
            // Let A be the most-recent point and B the other.
            let a = simplex[1];
            let b = simplex[0];
            let ab = b - a;
            // Compute the parameter t for the projection of the origin onto AB.
            let t = (-a).dot(&ab) / ab.dot(&ab);
            if t <= 0.0 {
                // Closest to A.
                (a, vec![a])
            } else if t >= 1.0 {
                // Closest to B.
                (b, vec![b])
            } else {
                let closest = a + ab * t;
                // The active feature is the edge from A to B.
                (closest, vec![b, a])
            }
        }
        3 => {
            // Simplex is a triangle.
            // Assume the vertices are A (most-recent), B, and C.
            let a = simplex[2];
            let b = simplex[1];
            let c = simplex[0];
            let closest = closest_point_on_triangle(a, b, c);
            // In a robust implementation, you would also reduce the simplex
            // to only the vertices defining the closest feature.
            (closest, simplex.to_vec())
        }
        4 => {
            // A tetrahedron (4 points) means the origin is enclosed.
            // In a distance query this means the shapes intersect,
            // so we can return the origin (distance zero) and the simplex as-is.
            (V3::new(0.0, 0.0, 0.0), simplex.to_vec())
        }
        _ => unreachable!("Simplex should have between 1 and 4 points in the distance algorithm"),
    }
}

/// Returns the closest point on the triangle ABC to the origin.
/// This uses the standard algorithm for point-triangle distance.
fn closest_point_on_triangle(a: V3, b: V3, c: V3) -> V3 {
    // Compute vectors from A to B and A to C.
    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);
    if d1 <= 0.0 && d2 <= 0.0 {
        // Closest to vertex A.
        return a;
    }

    let bo = -b;
    let d3 = ab.dot(&bo);
    let d4 = ac.dot(&bo);
    if d3 >= 0.0 && d4 <= d3 {
        // Closest to vertex B.
        return b;
    }

    let co = -c;
    let d5 = ab.dot(&co);
    let d6 = ac.dot(&co);
    if d6 >= 0.0 && d5 <= d6 {
        // Closest to vertex C.
        return c;
    }

    // Check if the origin is in edge regions AB or AC.
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return a + ab * t;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return a + ac * t;
    }
    // Else, the origin is in the face region.
    let va = d3 * d6 - d5 * d4;
    let denom = va + vb + vc;
    let t = vb / denom;
    let s = vc / denom;
    a + ab * t + ac * s
}