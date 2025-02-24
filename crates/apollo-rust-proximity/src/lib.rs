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
 0.0
}
