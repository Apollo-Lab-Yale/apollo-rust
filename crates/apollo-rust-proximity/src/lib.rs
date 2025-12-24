use apollo_rust_mesh_utils::trimesh::TriMesh;
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::vectors::V3;
use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::ops::{Add, Div, Neg, Sub};

const _PROXIMITY_TOL: f64 = 1e-6;
const _PROXIMITY_MAX_ITERS: usize = 100;

pub trait ShapeTrait {
    fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3;
}

pub struct ConvexPolyhedron(pub TriMesh);

impl ConvexPolyhedron {
    pub fn new(input_mesh: &TriMesh) -> Self {
        Self(input_mesh.to_convex_hull())
    }
}

pub struct Sphere {
    // suppose the center is always the origin
    pub radius: f64,
}

impl Sphere {
    pub fn new(radius: f64) -> Self {
        Self { radius }
    }
}

pub struct Cuboid {
    pub half_extents: V3,
}

impl Cuboid {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            half_extents: V3::new(x, y, z),
        }
    }
}

impl ShapeTrait for ConvexPolyhedron {
    fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3 {
        let local_dir = shape_pose.0.rotation.inverse() * dir;
        let mut max_point = V3::from_column_slice(&self.0.points[0]);
        let mut max_proj = max_point.dot(&local_dir);
        for point in self.0.points.iter().skip(1) {
            let cur_point = V3::from_column_slice(point);
            let proj = cur_point.dot(&local_dir);
            if proj > max_proj {
                max_proj = proj;
                max_point = cur_point;
            }
        }
        shape_pose.0.rotation * max_point + shape_pose.0.translation.vector
    }
}

impl ShapeTrait for Sphere {
    fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3 {
        let n = if dir.norm_squared() > 1e-12 {
            dir.normalize()
        } else {
            V3::new(1.0, 0.0, 0.0)
        };
        shape_pose.0.translation.vector + self.radius * n
    }
}

impl ShapeTrait for Cuboid {
    fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3 {
        let local_dir = shape_pose.0.rotation.inverse() * dir;
        let mut max_point = V3::new(0.0, 0.0, 0.0);
        let mut max_proj = f64::NEG_INFINITY;
        for i in 0..8 {
            let mut point = self.half_extents;
            let mut _i = i;
            for j in 0..3 {
                if (_i % 2) == 1 {
                    point[j] = point[j].neg()
                }
                _i = _i.div(2);
            }
            let proj = point.dot(&local_dir);
            if proj > max_proj {
                max_proj = proj;
                max_point = point;
            }
        }
        shape_pose.0.rotation * max_point + shape_pose.0.translation.vector
    }
}

#[derive(Clone)]
struct ThreeSimplex {
    arr: [V3; 4],
    len: usize,
}

#[derive(Clone)]
struct GJKFeature {
    pub v: V3,                 // closet point to origin on the simplex
    pub simplex: ThreeSimplex, // the simplex
    pub d: f64,                // distance from the simplex to origin
}

impl GJKFeature {
    pub fn new(v: V3, arr: [V3; 4], len: usize) -> Self {
        Self {
            v,
            simplex: ThreeSimplex::new_with_data(arr, len),
            d: v.norm(),
        }
    }
    pub fn min(self, other: GJKFeature) -> GJKFeature {
        if self.d < other.d {
            self
        } else {
            other
        }
    }
}

impl ThreeSimplex {
    pub fn new() -> Self {
        Self {
            arr: [V3::zeros(); 4],
            len: 0,
        }
    }
    pub fn new_with_data(arr: [V3; 4], len: usize) -> Self {
        Self { arr, len }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn add(&mut self, point: V3) {
        self.arr[self.len] = point;
        self.len += 1;
    }

    pub fn find_and_reduce(&mut self) -> (V3, f64) {
        match self.len {
            1 => (self.arr[0], self.arr[0].norm()),
            2 => {
                let f = closest_to_origin_on_line(&self.arr[0], &self.arr[1]);
                self.arr = f.simplex.arr;
                self.len = f.simplex.len;
                (f.v, f.d)
            }
            3 => {
                let f = closest_to_origin_on_triangle(&self.arr[0], &self.arr[1], &self.arr[2]);
                self.arr = f.simplex.arr;
                self.len = f.simplex.len;
                (f.v, f.d)
            }
            _ => {
                if tetrahedron_contains_origin(
                    &self.arr[0],
                    &self.arr[1],
                    &self.arr[2],
                    &self.arr[3],
                ) {
                    (V3::zeros(), 0.0)
                } else {
                    let f = closest_to_origin_on_tetrahedron(
                        &self.arr[0],
                        &self.arr[1],
                        &self.arr[2],
                        &self.arr[3],
                    );
                    self.arr = f.simplex.arr;
                    self.len = f.simplex.len;
                    (f.v, f.d)
                }
            }
        }
    }
}

fn closest_to_origin_on_line(a: &V3, b: &V3) -> GJKFeature {
    let ab = b.sub(a);
    let t = -(a.dot(&ab)).div(&(ab.dot(&ab)));
    if t < 0.0 {
        GJKFeature::new(*a, [*a, *b, V3::zeros(), V3::zeros()], 2)
    } else if t > 1.0 {
        GJKFeature::new(*b, [*a, *b, V3::zeros(), V3::zeros()], 2)
    } else {
        let closest = ab.scale(t).add(a);
        GJKFeature::new(closest, [*a, *b, V3::zeros(), V3::zeros()], 2)
    }
}

fn closest_to_origin_on_triangle(a: &V3, b: &V3, c: &V3) -> GJKFeature {
    // compute the barycentric coordinates of the projection of the origin.
    let ab = b.sub(a);
    let ac = c.sub(a);
    let ao = a.neg();
    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);
    let d00 = ab.dot(&ab);
    let d01 = ab.dot(&ac);
    let d11 = ac.dot(&ac);
    let denom = d00 * d11 - d01 * d01;
    let u = (d11 * d1 - d01 * d2) / denom;
    let v = (d00 * d2 - d01 * d1) / denom;

    // inside
    if u > 0.0 && v > 0.0 && (u + v) < 1.0 {
        let closest = u * b + v * c + (1.0 - u - v) * a;
        return GJKFeature::new(closest, [*a, *b, *c, V3::zeros()], 3);
    }

    // on edge
    let e1 = closest_to_origin_on_line(a, b);
    let e2 = closest_to_origin_on_line(b, c);
    let e3 = closest_to_origin_on_line(a, c);
    e3.min(e2).min(e1)
}

fn closest_to_origin_on_tetrahedron(a: &V3, b: &V3, c: &V3, d: &V3) -> GJKFeature {
    let f1 = closest_to_origin_on_triangle(a, b, c);
    let f2 = closest_to_origin_on_triangle(a, b, d);
    let f3 = closest_to_origin_on_triangle(a, c, d);
    let f4 = closest_to_origin_on_triangle(b, c, d);
    f4.min(f3).min(f2).min(f1)
}

fn signed_volume(a: &V3, b: &V3, c: &V3, d: &V3) -> f64 {
    (b - a).cross(&(c - a)).dot(&(d - a))
}

pub fn sphere_sphere_contact(
    sphere1: &Sphere,
    pose1: &LieGroupISE3q,
    sphere2: &Sphere,
    pose2: &LieGroupISE3q,
) -> (V3, f64) {
    let diff = pose1.0.translation.vector - pose2.0.translation.vector;
    let dist = diff.norm();
    let dir = if dist > 1e-12 {
        diff / dist
    } else {
        V3::new(1.0, 0.0, 0.0)
    };
    (dir, dist - (sphere1.radius + sphere2.radius))
}

fn tetrahedron_contains_origin(a: &V3, b: &V3, c: &V3, d: &V3) -> bool {
    let vol_abcd = signed_volume(a, b, c, d);
    let vol_0bcd = signed_volume(&V3::zeros(), b, c, d);
    let vol_a0cd = signed_volume(a, &V3::zeros(), c, d);
    let vol_ab0d = signed_volume(a, b, &V3::zeros(), d);
    let vol_abc0 = signed_volume(a, b, c, &V3::zeros());

    if vol_abcd.abs() < _PROXIMITY_TOL {
        return false;
    }

    let sign = vol_abcd.signum();
    vol_0bcd.signum() == sign
        && vol_a0cd.signum() == sign
        && vol_ab0d.signum() == sign
        && vol_abc0.signum() == sign
}

pub fn gjk_contact<S1: ShapeTrait, S2: ShapeTrait>(
    shape1: &S1,
    pose1: &LieGroupISE3q,
    shape2: &S2,
    pose2: &LieGroupISE3q,
) -> (V3, f64) {
    let mut simplex = ThreeSimplex::new();
    let mut dir = pose1.0.translation.vector.sub(&pose2.0.translation.vector);
    if dir.norm_squared() > 1e-12 {
        dir = dir.normalize()
    } else {
        dir = V3::new(1.0, 0.0, 0.0)
    };
    let mut support = shape1
        .support(&dir, pose1)
        .sub(shape2.support(&dir.neg(), pose2));
    simplex.add(support);
    let mut dist = support.norm();
    let mut iter = 0;
    while iter < _PROXIMITY_MAX_ITERS {
        (dir, dist) = simplex.find_and_reduce();
        // intersected
        if dist < _PROXIMITY_TOL {
            if simplex.len() == 4 {
                return epa(simplex, shape1, pose1, shape2, pose2);
            } else {
                // Try to build a tetrahedron if origin is on the simplex
                let axes = [
                    V3::new(1.0, 0.0, 0.0),
                    V3::new(-1.0, 0.0, 0.0),
                    V3::new(0.0, 1.0, 0.0),
                    V3::new(0.0, -1.0, 0.0),
                    V3::new(0.0, 0.0, 1.0),
                    V3::new(0.0, 0.0, -1.0),
                ];
                let mut found = false;
                for axis in axes {
                    let support = shape1
                        .support(&axis, pose1)
                        .sub(shape2.support(&axis.neg(), pose2));
                    if (support - simplex.arr[0]).norm_squared() > _PROXIMITY_TOL {
                        let mut exists = false;
                        for i in 0..simplex.len() {
                            if (support - simplex.arr[i]).norm_squared() < _PROXIMITY_TOL {
                                exists = true;
                                break;
                            }
                        }
                        if !exists {
                            simplex.add(support);
                            found = true;
                            if simplex.len() == 4 {
                                // Check if this tetrahedron actually contains the origin
                                if tetrahedron_contains_origin(
                                    &simplex.arr[0],
                                    &simplex.arr[1],
                                    &simplex.arr[2],
                                    &simplex.arr[3],
                                ) {
                                    return epa(simplex, shape1, pose1, shape2, pose2);
                                } else {
                                    // If not, reduce and continue
                                    (dir, dist) = simplex.find_and_reduce();
                                    break;
                                }
                            }
                        }
                    }
                }
                if !found {
                    return (dir, dist);
                }
            }
        }
        if dir.norm_squared() > 1e-12 {
            dir = dir.normalize();
        } else {
            dir = V3::new(1.0, 0.0, 0.0);
        }
        support = shape1
            .support(&dir.neg(), pose1)
            .sub(shape2.support(&dir, pose2));
        let proj = support.dot(&dir);
        //the simplex closet to the origin was found
        if dist < proj + _PROXIMITY_TOL {
            return (dir, dist);
        }
        // proceed to origin
        simplex.add(support);
        iter += 1;
    }
    (dir, dist)
}

#[derive(Clone)]
struct EPAFace {
    pub indices: [usize; 3],
    pub n: V3,  // normal
    pub d: f64, // distance to origin
}

impl EPAFace {
    pub fn new(indices: [usize; 3], n: V3, d: f64) -> Self {
        Self { indices, n, d }
    }
}

// Equality (used for removal) compares only the indices.
impl PartialEq for EPAFace {
    fn eq(&self, other: &Self) -> bool {
        self.indices == other.indices
    }
}

impl Eq for EPAFace {}

// Ordering for insertion and traversal compares by d,
// and if the distances are equal, breaks ties using indices.
// Also, if the indices are equal, it forces Ordering::Equal.
impl PartialOrd for EPAFace {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for EPAFace {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.indices == other.indices {
            Ordering::Equal
        } else {
            match self.d.partial_cmp(&other.d) {
                Some(Ordering::Equal) => self.indices.cmp(&other.indices),
                Some(ord) => ord,
                None => {
                    panic!(
                        "Cannot compare EPAFace: encountered NaN in distance (d), d1={}, d2={}",
                        self.d, other.d
                    )
                }
            }
        }
    }
}

struct EPAPolytope {
    pub points: Vec<V3>,
    pub faces: BTreeSet<EPAFace>,
}

impl EPAPolytope {
    pub fn new(points: Vec<V3>) -> Self {
        Self {
            points,
            faces: BTreeSet::new(),
        }
    }

    pub fn add_face(&mut self, indices: [usize; 3]) {
        let a = self.points[indices[0]];
        let b = self.points[indices[1]];
        let c = self.points[indices[2]];
        //println!("a={},b={},c={}",indices[0], indices[1], indices[2]);
        let mut normal = (b - a).cross(&(c - a)).normalize();
        let mut dist = normal.dot(&a);
        // make sure the normal points out
        if dist < 0.0 {
            normal *= -1.0;
            dist *= -1.0;
        }
        self.faces.insert(EPAFace::new(indices, normal, dist));
    }

    pub fn add_vertex(&mut self, vertex: V3) {
        self.points.push(vertex);
    }

    pub fn closest_face_to_origin(&self) -> (V3, f64) {
        let face = self.faces.iter().next().unwrap();
        (face.n, face.d)
    }

    pub fn expand(&mut self, v: V3) {
        let mut new_edges: Vec<[usize; 2]> = Vec::new();
        let mut to_remove: Vec<EPAFace> = Vec::new();
        for face in &self.faces {
            let a = self.points[face.indices[0]];
            // the face can be seen from v
            // the tol in this judgement is extremely important to include
            // faces that have v on their planes so that all redundant edges can be removed
            if face.n.dot(&(v - a)) > -_PROXIMITY_TOL * _PROXIMITY_TOL {
                EPAPolytope::add_unique_edge(face.indices[0], face.indices[1], &mut new_edges);
                EPAPolytope::add_unique_edge(face.indices[1], face.indices[2], &mut new_edges);
                EPAPolytope::add_unique_edge(face.indices[2], face.indices[0], &mut new_edges);
                to_remove.push(face.clone());
            }
        }
        // remove faces
        for face in to_remove {
            self.faces.remove(&face);
        }
        // add the new vertex and new faces
        self.add_vertex(v);
        for edge in new_edges.iter() {
            self.add_face([edge[0], edge[1], self.points.len() - 1]);
        }
    }

    fn add_unique_edge(a: usize, b: usize, edges: &mut Vec<[usize; 2]>) {
        if let Some(index) = edges.iter().position(|&x| x == [a, b]) {
            edges.remove(index);
            return;
        }
        if let Some(index) = edges.iter().position(|&x| x == [b, a]) {
            edges.remove(index);
            return;
        }
        edges.push([a, b]);
    }
}

fn epa<S1: ShapeTrait, S2: ShapeTrait>(
    simplex: ThreeSimplex,
    shape1: &S1,
    pose1: &LieGroupISE3q,
    shape2: &S2,
    pose2: &LieGroupISE3q,
) -> (V3, f64) {
    assert_eq!(simplex.len(), 4);
    // initialize polytope
    let mut polytope = EPAPolytope::new(Vec::from(simplex.arr));
    polytope.add_face([0, 1, 2]);
    polytope.add_face([0, 3, 1]);
    polytope.add_face([0, 2, 3]);
    polytope.add_face([1, 3, 2]);
    let mut min_normal = V3::zeros();
    let mut min_dist = f64::INFINITY;
    let mut to_expand = true;
    // main loop
    while to_expand {
        (min_normal, min_dist) = polytope.closest_face_to_origin();
        /*
        let face = polytope.faces.iter().next().unwrap();
        let a = pose1.inverse().0.rotation*polytope.points[face.indices[0]]+pose1.inverse().0.translation.vector;
        let b = pose1.inverse().0.rotation*polytope.points[face.indices[1]]+pose1.inverse().0.translation.vector;
        let c =  pose1.inverse().0.rotation*polytope.points[face.indices[2]]+pose1.inverse().0.translation.vector;
        let n1 = (b-a).cross(&(c-a)).normalize();
        println!("a={:?},b={:?}, c={:?}, n={:?},n1={:?}",a,
                 b,c,
                 pose1.inverse().0.rotation*min_normal,n1);
         */
        let p1 = shape1.support(&min_normal, pose1);
        let p2 = shape2.support(&min_normal.neg(), pose2);
        let support = p1 - p2;
        if support.dot(&min_normal) > min_dist + _PROXIMITY_TOL {
            polytope.expand(support);
        } else {
            to_expand = false;
        }
    }
    (min_normal, -min_dist)
}
