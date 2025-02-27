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
struct ThreeSimplex(pub Vec<V3>);


impl ThreeSimplex { 
 pub fn new() -> Self {
  Self(Vec::<V3>::with_capacity(4))
 }

 pub fn add_unique(&mut self, point: V3) -> bool {
  for &v in &self.0 {
   if (v - point).norm() < _PROXIMITY_TOL {
    return false;
   }
  }
  self.add(point);
  true
 }
 pub fn len(&self) ->usize{
  self.0.len()
 }

 pub fn add(&mut self, point: V3) {
  self.0.push(point);
 }
 pub fn closest_point_to_origin(&self) -> f64 {
  match self.0.len() {
   1 => self.0[0].norm(),
   2 => closest_to_origin_on_line(&self.0[1], &self.0[0]).norm(),
   3 =>closest_to_origin_on_triangle(&self.0[2], &self.0[1], &self.0[0]).norm(),
   _ => -1.0, // invalid
  }
 }
 fn process_simplex(&mut self, dir: &mut V3) -> bool{
  match self.0.len() {
   2 => self.process_line(dir),
   3 => self.process_triangle(dir),
   4 => self.process_tetrahedron(dir),
   _ => false,
  }
 }

 fn process_line(&mut self, dir: &mut V3) -> bool{
  let a = self.0[1];
  let b = self.0[0];

  let ab = b.sub(&a);
  let ao = a.neg();

  if ab.dot(&ao)>0.0 {
   // Origin is in the direction of the line
   // New direction is the triple cross product
   *dir = ab.cross(&ao).cross(&ab).normalize();
  } else {
   // Origin is in direction of just the point
    self.0 = vec![a]; // Keep only the newest point
   *dir = ao.normalize(); // New direction is towards origin
  }

  false // Cannot contain origin with just a line
 }

 fn process_triangle(&mut self, dir: &mut V3) -> bool {
  let a = self.0[2];
  let b = self.0[1];
  let c = self.0[0];

  let ab = b.sub(&a);
  let ac = c.sub(&a);
  let ao = a.neg(); // Direction from point a to origin

  let abc = ab.cross(&ac);

  // Check if origin is "outside" relative to AC
  if abc.cross(&ac).dot(&ao) > 0.0 {
   if ac.dot(&ao) > 0.0 {
    // Origin outside, in direction of AC edge
    self.0 = vec![c, a];
    *dir = ac.cross(&ao).cross(&ac).normalize();
    return false;
   }

   // If not outside AC, check AB
   if ab.dot(&ao) > 0.0 {
    // Origin outside, in direction of AB edge
    self.0 = vec![b, a];
    *dir = ab.cross(&ao).cross(&ab).normalize();
    return false;
   }

   // Otherwise, just keep A
   self.0 = vec![a];
   *dir = ao.normalize();
   return false;
  }

  // Check if origin is "outside" relative to AB
  if ab.cross(&abc).dot(&ao) > 0.0 {
   if ab.dot(&ao) > 0.0 {
    // Origin outside, in direction of AB edge
    self.0 = vec![b, a];
    *dir = ab.cross(&ao).cross(&ab).normalize();
    return false;
   }

   // Otherwise, keep A
   self.0 = vec![a];
   *dir = ao.normalize();
   return false;
  }

  // If none of the edges excludes the origin, check if it's above or below the face
  if abc.dot(&ao) > 0.0 {
   // Origin above triangle
   *dir = abc.normalize();
  } else {
   // Origin below triangle; flip winding
   self.0 = vec![a, b, c];
   *dir = abc.neg().normalize();
  }
  false // triangle doesn't conclusively contain origin yet
 }

 fn process_tetrahedron(&mut self, dir: &mut V3) -> bool {
  //println!("{:?}", self.0);
  let a = self.0[3]; // newest point
  let b = self.0[2];
  let c = self.0[1];
  let d = self.0[0];

  let ab = b.sub(&a);
  let ac = c.sub(&a);
  let ad = d.sub(&a);
  let ao = a.neg();
  let bc = c.sub(&b);
  let bd = d.sub(&b);
  let bo = b.neg();

  // compute normals for faces that include A:
  let abc = ab.cross(&ac);
  let acd = ac.cross(&ad);
  let adb = ad.cross(&ab);
  let bdc = bd.cross(&bc);

  let d_abc = abc.dot(&ao);
  let d_acd = acd.dot(&ao);
  let d_adb = adb.dot(&ao);
  let d_bdc = bdc.dot(&bo);

  if d_abc < 1e-6 && d_acd < 1e-6 && d_adb < 1e-6 && d_bdc < 1e-6 {
   return true;  // GJK: we have an intersection => distance 0
  }

  // otherwise, select the face with the maximum positive dot product.
  // this face is the one from which the origin is most separated.
  if d_abc >= d_acd && d_abc >= d_adb && d_abc >= d_bdc {
   // Origin is outside face ABC => remove D (opposite of ABC)
   self.0 = vec![c, b, a];
   *dir = abc.normalize();
  } else if d_acd >= d_abc && d_acd >= d_adb && d_acd >= d_bdc {
   // Outside face ACD => remove B
   self.0 = vec![d, c, a];
   *dir = acd.normalize();
  } else if d_adb >= d_abc && d_adb >= d_acd && d_adb >= d_bdc {
   // Outside face ADB => remove C
   self.0 = vec![b, d, a];
   *dir = adb.normalize();
  } else {
   // Outside face BCD => remove A
   self.0 = vec![d, c, b];
   *dir = bdc.normalize();
  }

  false
 }
}


fn closest_to_origin_on_line(a:&V3,b:&V3) -> V3{
 let ab = b.sub(a);
 let t = -(a.dot(&ab)).div(&(ab.dot(&ab)));
 if t<1e-6{
  a.clone()
 }
 else if t>1.0-1e-6{
  b.clone()
 } else{
  let closest = ab.scale(t).add(a);
  closest
 }
}

fn closest_to_origin_on_triangle(a: &V3, b: &V3, c: &V3) -> V3 {
 let ab = b.sub(a);
 let ac = c.sub(a);

 // Compute the barycentric coordinates of the projection of the origin.
 let ao = a.neg();
 let d1 = ab.dot(&ao);
 let d2 = ac.dot(&ao);
 let d00 = ab.dot(&ab);
 let d01 = ab.dot(&ac);
 let d11 = ac.dot(&ac);
 let denom = d00 * d11 - d01 * d01;
 let u = (d11 * d1 - d01 * d2) / denom;
 let v = (d00 * d2 - d01 * d1) / denom;

 if u >= 0.0 && v >= 0.0 && (u + v) <= 1.0 {
  return u*b+v*c+(1.0-u-v)*a;
  //let normal = ab.cross(&ac).normalize();
  //return (a.dot(&normal)).abs();
 }

 let edge_ab = closest_to_origin_on_line(a, b);
 let edge_bc = closest_to_origin_on_line(b, c);
 let edge_ca = closest_to_origin_on_line(c, a);
 let min_norm= edge_ab.norm().min(edge_bc.norm()).min(edge_ca.norm());
 if min_norm == edge_ab.norm(){return edge_ab;}
 if min_norm == edge_ca.norm() {return edge_ca;}
 edge_bc
}

pub fn gjk_distance<S1: ShapeTrait, S2: ShapeTrait>(shape1: &S1, pose1: &LieGroupISE3q, shape2: &S2, pose2:&LieGroupISE3q) -> f64 {
 let mut simplex = ThreeSimplex::new();
 let initial_dir = pose1.0.translation.vector.sub(&pose2.0.translation.vector);
 let mut dir = if initial_dir.norm_squared() > 1e-6 {initial_dir.normalize()} else {V3::new(1.0, 0.0, 0.0)};
 let mut support = shape1.support(&dir, pose1).sub(shape2.support(&dir.neg(), pose2));
 simplex.add(support);
 dir = support.normalize().neg();
 let mut dist = simplex.closest_point_to_origin();
 let mut iter=0;
 while iter < _PROXIMITY_MAX_ITERS {
      support = shape1.support(&dir, pose1).sub(shape2.support(&dir.neg(), pose2));
      let proj = support.dot(&dir);
      dist = simplex.closest_point_to_origin();
      //println!("proj={}, dist = {}, size={}",proj,  dist, simplex.len());
      //the simplex closet to the origin was found
      if proj+dist < _PROXIMITY_TOL {
         return dist;
    }
     // proceed to origin
     simplex.add(support);
     // when the simplex stabled, it's equivalent to simplex.process_tetrahedron
     let intersect = simplex.process_simplex(&mut dir);
     if intersect {return 0.0;}
     iter+=1;
  }
 dist
}