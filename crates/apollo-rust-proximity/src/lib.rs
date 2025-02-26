use std::ops::{Add, Div, Neg, Sub};
use nalgebra::sup;
use parry3d_f64::query::gjk::eps_tol;
use apollo_rust_lie::EuclideanSpaceElement;
use apollo_rust_mesh_utils::trimesh::TriMesh;
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::vectors::V3;

const _PROXIMITY_TOL: f64 =1e-3;

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

struct ThreeSimplex(pub Vec<V3>);

impl ThreeSimplex { 
 pub fn new() -> Self {
  Self(Vec::<V3>::with_capacity(4))
 }

 pub fn add(&mut self, point: V3) {
  self.0.push(point);
 }
 pub fn closest_point_to_origin(&self) -> f64 {
  match self.0.len() {
   1 => self.0[0].norm(),
   2 => ThreeSimplex::closest_point_on_line(&self.0[0], &self.0[1]),
   3 => ThreeSimplex::closest_point_on_triangle(&self.0[0], &self.0[1], &self.0[2]),
   _ => self.closest_point_on_tetrahedron(),
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
 fn closest_point_on_line(a:&V3,b:&V3) -> f64{
  let ab = b.sub(a);
  let t = -(a.dot(&ab)).div(&(ab.dot(&ab)));
  if t<=0.0{
   a.norm()
  }
  else if t>=1.0{
   b.norm()
  } else{
   let closest = ab.scale(t).add(a);
   closest.norm()
  }
 }

 fn closest_point_on_triangle(a: &V3, b: &V3, c: &V3) -> f64 {
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
   // Projection is inside the triangle.
   // Distance is the perpendicular distance from the origin to the plane.
   let normal = ab.cross(&ac).normalize();
   return (a.dot(&normal)).abs();
  }

  // Otherwise, return the minimum distance to the triangle’s edges.
  let edge_ab = ThreeSimplex::closest_point_on_line(a, b);
  let edge_bc = ThreeSimplex::closest_point_on_line(b, c);
  let edge_ca = ThreeSimplex::closest_point_on_line(c, a);
  edge_ab.min(edge_bc).min(edge_ca)
 }


 fn closest_point_on_tetrahedron(&self) -> f64{
  // Check if closest point is on a face
  let abc_dist = ThreeSimplex::closest_point_on_triangle(&self.0[0], &self.0[1], &self.0[2]);
  let acd_dist = ThreeSimplex::closest_point_on_triangle(&self.0[0], &self.0[2], &self.0[3]);
  let adb_dist = ThreeSimplex::closest_point_on_triangle(&self.0[0], &self.0[3], &self.0[1]);
  let bcd_dist = ThreeSimplex::closest_point_on_triangle(&self.0[1], &self.0[2], &self.0[3]);
  abc_dist.min(acd_dist).min(adb_dist).min(bcd_dist)
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
   self.0 = vec![c, b, a];
   *dir = abc.neg().normalize();
  }

  false // Triangle doesn't conclusively contain origin yet
 }


 fn process_tetrahedron(&mut self, dir: &mut V3) -> bool {
  let a = self.0[3]; // Newest point
  let b = self.0[2];
  let c = self.0[1];
  let d = self.0[0];

  let ab = b.sub(&a);
  let ac = c.sub(&a);
  let ad = d.sub(&a);
  let ao = a.neg();

  // Normals to faces
  let abc = ab.cross(&ac);
  let acd = ac.cross(&ad);
  let adb = ad.cross(&ab);

  // Check each face in a fixed order.
  if abc.dot(&ao) > 0.0 {
   // Origin outside face ABC: remove D.
   self.0 = vec![c, b, a];
   *dir = abc.normalize();
   return false;
  }
  if acd.dot(&ao) > 0.0 {
   // Origin outside face ACD: remove B.
   self.0 = vec![d, c, a];
   *dir = acd.normalize();
   return false;
  }
  if adb.dot(&ao) > 0.0 {
   // Origin outside face ADB: remove C.
   self.0 = vec![b, d, a];
   *dir = adb.normalize();
   return false;
  }

  // Otherwise, conclude that the origin is inside.
  true
 }


}

pub fn gjk_distance<S1: ShapeTrait, S2: ShapeTrait>(shape1: &S1, pose1: &LieGroupISE3q, shape2: &S2, pose2:&LieGroupISE3q) -> f64 {
 let mut simplex = ThreeSimplex::new();

 let initial_dir = pose1.0.translation.vector.sub(&pose2.0.translation.vector);
 let mut dir = if initial_dir.norm_squared() > 1e-6 {
  initial_dir.normalize()
 } else {
  V3::new(1.0, 0.0, 0.0)
 };
  let mut support = shape1.support(&dir, pose1).sub(shape2.support(&dir.neg(), pose2));
  simplex.add(support);
  dir = support.normalize().neg();
  let mut prev_distance = f64::INFINITY;
  loop{
    support = shape1.support(&dir, pose1).sub(shape2.support(&dir.neg(), pose2));
    let proj = support.dot(&dir);
   if proj < 0.0 {
      return 1.0;
   }
   simplex.add(support);
   let intersect = simplex.process_simplex(&mut dir);
   if intersect {return 0.0;}
  }
}
