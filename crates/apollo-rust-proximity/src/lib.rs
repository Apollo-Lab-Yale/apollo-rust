use std::ops::{Add, Div, Neg, Sub};
use nalgebra::{sup, MatrixSum};
use parry3d_f64::query::gjk::eps_tol;
use apollo_rust_lie::EuclideanSpaceElement;
use apollo_rust_mesh_utils::trimesh::TriMesh;
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::vectors::{ApolloVector3Trait, V3};

const _PROXIMITY_TOL: f64 =1e-6;
const _PROXIMITY_MAX_ITERS: usize = 100;

pub trait ShapeTrait {
 fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3;
}


pub struct ConvexPolyhedron(pub TriMesh);

impl ConvexPolyhedron {
  pub fn new(input_mesh: &TriMesh)->Self{
     Self(input_mesh.to_convex_hull())
 }
}

pub struct Sphere{
 // suppose the center is always the origin
 pub radius: f64,
}

impl Sphere {
 pub fn new(radius:f64)->Self{
  Self{radius}
 }
}

pub struct Cuboid{
 pub half_extents: V3
}

impl Cuboid{
 pub fn new(x:f64,y:f64,z:f64)->Self{
  Self{half_extents: V3::new(x,y,z)}
 }
}

impl ShapeTrait for ConvexPolyhedron {
 fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3 {
    let local_dir = shape_pose.0.rotation.inverse() * dir;
    let mut max_point = V3::from_column_slice(&self.0.points[0]);
    let mut max_proj = max_point.dot(&local_dir);
    for point in self.0.points.iter().skip(1) {
       let cur_point = V3::from_column_slice(point);
       let proj =cur_point.dot(&local_dir);
       if proj > max_proj {
          max_proj = proj;
          max_point = cur_point;
   }
  }
  shape_pose.0.rotation*max_point+shape_pose.0.translation.vector
 }
}

impl ShapeTrait for Sphere {
 fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3 {
     shape_pose.0.translation.vector+self.radius*dir.normalize()
 }
}

impl ShapeTrait for Cuboid {
 fn support(&self, dir: &V3, shape_pose: &LieGroupISE3q) -> V3 {
  let local_dir = shape_pose.0.rotation.inverse() * dir;
  let mut max_point = V3::new(0.0,0.0,0.0);
  let mut max_proj = f64::NEG_INFINITY;
  for i in 0..8{
       let mut point=self.half_extents.clone();
       let mut _i=i.clone();
       for j in 0..3{
          if (_i%2)==1 {point[j]=point[j].neg()}
          _i=_i.div(2);
       }
    let proj =point.dot(&local_dir);
   if proj > max_proj {
    max_proj = proj;
    max_point = point;
   }
  }
  shape_pose.0.rotation*max_point+shape_pose.0.translation.vector
 }
}


#[derive(Clone)]
struct GJKFeature{
 pub v: V3,
 pub simplex: Vec<V3>,
 pub d: f64,
}


impl GJKFeature{
  pub fn new(v: V3, simplex: Vec<V3>)->Self{
    Self{v, simplex, d:v.norm()}
  }
 pub fn min(self, other: GJKFeature) -> GJKFeature{
   if self.d < other.d { self } else { other }
 }
}

#[derive(Clone)]
struct ThreeSimplex(pub Vec<V3>);

impl ThreeSimplex {
 pub fn new() -> Self {
  Self(Vec::<V3>::with_capacity(4))
 }

 pub fn len(&self) -> usize {
  self.0.len()
 }

 pub fn add(&mut self, point: V3) {
  self.0.push(point);
 }

pub fn find_and_reduce(&mut self) -> (V3, f64) {
  match self.0.len() {
   1 => {  (self.0[0], self.0[0].norm()) },
   2 => {let f = closest_to_origin_on_line(&self.0[0], &self.0[1]); self.0=f.simplex; (f.v, f.d)},
   3 =>{let f = closest_to_origin_on_triangle(&self.0[0], &self.0[1], &self.0[2]); self.0=f.simplex; (f.v, f.d)},
   _ =>{ if tetrahedron_contains_origin(&self.0[0],&self.0[1], &self.0[2], &self.0[3]) {(V3::zeros(),0.0)} else {let f = closest_to_origin_on_tetrahedron(&self.0[0],&self.0[1], &self.0[2], &self.0[3]); self.0=f.simplex; (f.v, f.d)}},
  }
 }
}

fn closest_to_origin_on_line(a:&V3, b:&V3) -> GJKFeature {
 let ab = b.sub(a);
 let t = -(a.dot(&ab)).div(&(ab.dot(&ab)));
 if t<0.0{
  GJKFeature::new(a.clone(), vec![*a, *b])
 }
 else if t>1.0{
  GJKFeature::new(b.clone(), vec![*a, *b])
 } else{
  let closest = ab.scale(t).add(a);
  GJKFeature::new(closest, vec![*a, *b])
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
    return GJKFeature::new(u*b+v*c+(1.0-u-v)*a, vec![*a, *b, *c])
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

fn tetrahedron_contains_origin(a: &V3, b: &V3, c: &V3, d: &V3) -> bool {
 let vol = signed_volume(a, b, c, d);
 if vol.abs() < _PROXIMITY_TOL {
  return false;
 }

 // compute the signed volumes of tetrahedra that replace one vertex with the origin.
 let v1 = signed_volume(&V3::zeros(), b, c, d);
 let v2 = signed_volume(a,&V3::zeros(), c, d);
 let v3 = signed_volume(a, b, &V3::zeros(), d);
 let v4 = signed_volume(a, b, c, &V3::zeros());

 if vol > 0.0 {
  v1 > -_PROXIMITY_TOL && v2 > -_PROXIMITY_TOL && v3 > -_PROXIMITY_TOL && v4 > -_PROXIMITY_TOL
 } else {
  v1 < _PROXIMITY_TOL && v2 < _PROXIMITY_TOL && v3 < _PROXIMITY_TOL && v4 < _PROXIMITY_TOL
 }
}


pub fn gjk_distance<S1: ShapeTrait, S2: ShapeTrait>(shape1: &S1, pose1: &LieGroupISE3q, shape2: &S2, pose2:&LieGroupISE3q) -> f64 {
 let mut simplex = ThreeSimplex::new();
 let mut dir = pose1.0.translation.vector.sub(&pose2.0.translation.vector);
 if dir.norm_squared() > 1e-6 {dir=dir.normalize()} else {dir=V3::new(1.0, 0.0, 0.0)};
 let mut support = shape1.support(&dir, pose1).sub(shape2.support(&dir.neg(), pose2));
 simplex.add(support);
 let mut dist=support.norm();
 let mut iter=0;
 while iter < _PROXIMITY_MAX_ITERS {
      (dir, dist) = simplex.find_and_reduce();
      if dist < _PROXIMITY_TOL {return 0.0;}
      dir = dir.neg().normalize();
      support = shape1.support(&dir, pose1).sub(shape2.support(&dir.neg(), pose2));
      let proj = support.dot(&dir);
      //the simplex closet to the origin was found
      if proj+dist < _PROXIMITY_TOL {
         return dist;
    }
     // proceed to origin
     simplex.add(support);
     iter+=1;
  }
 dist
}