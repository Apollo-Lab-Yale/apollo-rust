use apollo_rust_mesh_utils::stl::load_stl_file;
use apollo_rust_mesh_utils::trimesh::{ToTriMesh, TriMesh};
use std::env;

#[test]
fn test_trimesh_construction_and_hull() {
    // Create a simple tetrahedron
    let points = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    let indices = vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];

    let mut mesh = TriMesh::new_empty();
    mesh.extend_from_points_and_indices(&points, &indices);

    assert_eq!(mesh.points.len(), 4);
    assert_eq!(mesh.indices.len(), 4);

    // Convex hull of a tetrahedron (which is already convex) should be similar
    let hull = mesh.to_convex_hull();
    assert!(!hull.points.is_empty());
    assert!(!hull.indices.is_empty());
}

#[test]
fn test_stl_roundtrip() {
    let mut file_path = env::temp_dir();
    file_path.push("apollo_mesh_utils_regression_test.stl");

    // Create a simple triangle mesh
    let points = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let indices = vec![[0, 1, 2]];

    let mut mesh = TriMesh::new_empty();
    mesh.extend_from_points_and_indices(&points, &indices);

    // Save to STL
    mesh.save_to_stl(&file_path);

    // Load from STL
    let loaded_mesh_res = load_stl_file(&file_path);
    assert!(
        loaded_mesh_res.is_ok(),
        "Failed to load STL file from {:?}",
        file_path
    );

    let loaded_mesh = loaded_mesh_res.unwrap();
    let trimesh_from_stl = loaded_mesh.to_trimesh();

    // Verification
    assert!(!trimesh_from_stl.points.is_empty());
    assert!(!trimesh_from_stl.indices.is_empty());

    // Clean up
    let _ = std::fs::remove_file(file_path);
}
