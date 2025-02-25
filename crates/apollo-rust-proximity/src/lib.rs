use std::ops::{Add, Div, Neg, Sub};
use apollo_rust_lie::EuclideanSpaceElement;
use apollo_rust_mesh_utils::trimesh::TriMesh;
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::vectors::V3;

pub fn transform_mesh(original_mesh: &TriMesh, transformation: &LieGroupISE3q)->TriMesh{
    let mut out = original_mesh.clone();
    for i in 0..original_mesh.points.len(){
        let v = V3::new(original_mesh.points[i][0], original_mesh.points[i][1], original_mesh.points[i][2]);
        let v_new = transformation.0.rotation*v + transformation.0.translation.vector;
        out.points[i][0] = v_new.x;
        out.points[i][1] = v_new.y;
        out.points[i][2] = v_new.z;
    }
    out
}

pub fn gjk_distance(h1: &TriMesh, h2: &TriMesh) -> f64 {
  let mut simplex = Vec::<V3>::with_capacity(4);
  let mut dir = V3::new(1.0, 0.0, 0.0);
  let mut support = compute_support(h1, h2, &dir);
  simplex.push(support);
  dir = dir.neg();
  loop{
    support = compute_support(h1, h2, &dir);
   if support.dot(&dir) < 0.0 {return closest_point_to_origin(&simplex);}
   simplex.push(support);
   let intersect = process_simplex(&mut simplex, &mut dir);
   if intersect {return 0.0;}
  }
}

fn compute_support(h1: &TriMesh, h2: &TriMesh, dir: &V3)->V3{
   furthest_point(&h1.points, dir).sub(&furthest_point(&h2.points, &dir))
}

fn furthest_point(points: &Vec<[f64;3]>, dir: &V3) -> V3{
 let mut max_point = V3::from_column_slice(&points[0]);
 let mut max_proj = max_point.dot(&dir);
 for point in points.iter().skip(1) {
  let cur_point = V3::from_column_slice(point);
  let proj =cur_point.dot(&dir);
  if proj > max_proj {
   max_proj = proj;
   max_point = cur_point;
  }
 }
 max_point
}

fn closest_point_to_origin(simplex: &Vec<V3>) -> f64 {
 match simplex.len() {
  1 => simplex[0].norm(),
  2 => closest_point_on_line(&simplex[0], &simplex[1]),
  3 => closest_point_on_triangle(&simplex[0], &simplex[1], &simplex[2]),
  _ => closest_point_on_tetrahedron(simplex),
 }
}

fn process_simplex(simplex: &mut Vec<V3>, dir: &mut V3) -> bool{
 match simplex.len() {
  2 => process_line(simplex, dir),
  3 => process_triangle(simplex, dir),
  4 => process_tetrahedron(simplex, dir),
  _ => false,
 }
}

fn closest_point_on_line(a: &V3, b: &V3) -> f64{
 let ab = b.sub(a);
 let t = -(a.dot(&ab)).div(&(ab.dot(&ab)));
   if t<=0.0{
      a.norm()
   }
   else if t>=1.0{
     b.norm()
   } else{
    let closest = b.sub(a).scale(t).add(a);
    closest.norm()
   }
}

fn closest_point_on_triangle(a: &V3, b: &V3, c: &V3) -> f64{
 // if closest point is on an edge
 let ab_dist = closest_point_on_line(a, b);
 let bc_dist = closest_point_on_line(b, c);
 let ca_dist = closest_point_on_line(c, a);
 let edge_dist = ab_dist.min(bc_dist).min(ca_dist);

 // if closest point is on the face of the triangle
 let ab = b.sub(a);
 let ac = c.sub(a);
 let normal = ab.cross(&ac).normalize();

 let distance_to_plane = a.dot(&normal);
 let face_dist = distance_to_plane.abs();

 // Return the minimum of edge and face distances
 edge_dist.min(face_dist)
}

fn closest_point_on_tetrahedron(simplex: &Vec<V3>) -> f64{
 // Check if closest point is on a face
 let abc_dist = closest_point_on_triangle(&simplex[0], &simplex[1], &simplex[2]);
 let acd_dist = closest_point_on_triangle(&simplex[0], &simplex[2], &simplex[3]);
 let adb_dist = closest_point_on_triangle(&simplex[0], &simplex[3], &simplex[1]);
 let bcd_dist = closest_point_on_triangle(&simplex[1], &simplex[2], &simplex[3]);
  abc_dist.min(acd_dist).min(adb_dist).min(bcd_dist)
}

fn process_line(simplex: &mut Vec<V3>, dir: &mut V3) -> bool{
 let a = simplex[1];
 let b = simplex[0];

 let ab = b.sub(&a);
 let ao = a.neg();

 if ab.dot(&ao)>0.0 {
  // Origin is in the direction of the line
  // New direction is the triple cross product
  *dir = ab.cross(&ao).cross(&ab).normalize();
 } else {
  // Origin is in direction of just the point
  *simplex = vec![a]; // Keep only the newest point
  *dir = ao.normalize(); // New direction is towards origin
 }

 false // Cannot contain origin with just a line
}
fn process_triangle(simplex: &mut Vec<V3>, dir: &mut V3) -> bool{
 let a = simplex[2];
 let b = simplex[1];
 let c = simplex[0];

 let ab = b.sub(&a);
 let ac = c.sub(&a);
 let ao = a.neg(); // Direction from point a to origin

 let abc = ab.cross(&ac);

 // Check if origin is above or below the triangle
 if abc.cross(&ac).dot(&ao) > 0.0 {
    if ac.dot(&ao) >0.0 {
       // Origin outside, in direction of ac edge
      *simplex = vec![c, a];
      *dir = ac.cross(&ao).cross(&ac).normalize();
  } else {
         if ab.dot(&ao) > 0.0 {
         // Origin outside, in direction of ab edge
          *simplex = vec![b, a];
          *dir =ab.cross(&ao).cross(&ab).normalize();
   }     else {
           *simplex = vec![a]; // Keep only a
           *dir = ao.normalize();
   }
  }
 }
 else {
  if ab.cross(&abc).dot(&ao) > 0.0 {
   if ab.dot(&ao) > 0.0 {
    // Origin outside, in direction of ab edge
    *simplex = vec![b, a];
    *dir = ab.cross(&ao).cross(&ab).normalize();
   } else {
    // Origin above, in direction of a
    *simplex = vec![a]; // Keep only a
    *dir = ao.normalize();
   }
  } else {
   // Check if origin is above or below triangle
   if abc.dot(&ao) > 0.0 {
    // Origin above triangle
    *dir = abc.normalize();
   } else {
    // Origin below triangle, reverse winding
    *simplex = vec![c, b, a]; // Rearrange to change winding
    *dir = abc.neg().normalize();
   }
  }
 }

 false // Triangle cannot contain origin
}

fn process_tetrahedron(simplex: &mut Vec<V3>, dir: &mut V3)->bool{
   let a = simplex[3]; // Newest point
   let b = simplex[2];
   let c = simplex[1];
   let d = simplex[0];

   let ab = b.sub(&a);
   let ac = c.sub(&a);
   let ad = d.sub(&a);
   let ao = a.neg();

   // Normal to faces
   let abc = ab.cross(&ac);
   let acd = ac.cross(&ad);
   let adb = ad.cross(&ab);

   // Check each face
   if abc.dot(&ao) > 0.0 {
    // Origin outside face abc
    // Remove d and keep only a, b, c
    *simplex = vec![c, b, a];
    *dir = abc.normalize();
    return false;
   }

   if acd.dot(&ao) > 0.0 {
    *simplex = vec![d, c, a];
    *dir = acd.normalize();
    return false;
   }

   if adb.dot(&ao) > 0.0 {
    // Origin outside face adb
    // Remove c and keep only a, d, b
    *simplex = vec![b, d, a];
    *dir = adb.normalize();
    return false;
   }

   // Origin is inside tetrahedron
   true
}