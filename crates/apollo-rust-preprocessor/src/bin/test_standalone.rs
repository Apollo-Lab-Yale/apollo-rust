use apollo_rust_preprocessor::standalone_preprocessor::StandalonePreprocessor;
use std::path::PathBuf;

fn main() {
    let candidates = vec![
        "../../crates/apollo-rust-app/public/ur5/ur5.urdf",
        "crates/apollo-rust-app/public/ur5/ur5.urdf",
    ];
    let urdf_path = candidates
        .iter()
        .map(|p| PathBuf::from(p))
        .find(|p| p.exists())
        .expect("Could not find ur5.urdf in likely locations");

    let mesh_dir = urdf_path
        .parent()
        .expect("urdf path has no parent")
        .to_path_buf();
    let output_dir = PathBuf::from("standalone_output");

    // Clean output directory
    if output_dir.exists() {
        std::fs::remove_dir_all(&output_dir).expect("Could not clean output directory");
    }
    std::fs::create_dir(&output_dir).expect("Could not create output directory");

    println!("Testing Standalone Preprocessor...");
    match StandalonePreprocessor::process_robot(&urdf_path, &mesh_dir, &output_dir) {
        Ok(_) => {
            println!("Successfully processed robot!");

            // Verification: Check if modules exist
            let modules_to_check = vec![
                "ur5/urdf_module/module.json",
                "ur5/dof_module/module.json",
                "ur5/chain_module/module.json",
                "ur5/connections_module/module.json",
                "ur5/mesh_modules/original_meshes_module/module.json",
                "ur5/mesh_modules/plain_meshes_module/module.json",
                "ur5/mesh_modules/convex_hull_meshes_module/module.json",
                "ur5/mesh_modules/convex_decomposition_meshes_module/module.json",
                "ur5/bounds_module/module.json",
                "ur5/link_shapes_modules/link_shapes_max_distance_from_origin_module/module.json",
                "ur5/link_shapes_modules/link_shapes_distance_statistics_module/module.json",
                "ur5/link_shapes_modules/link_shapes_simple_skips_module/module.json",
                "ur5/link_shapes_modules/link_shapes_approximations_module/module.json",
            ];

            for module_rel_path in modules_to_check {
                let path = output_dir.join(module_rel_path);
                if !path.exists() {
                    eprintln!("Verification Failure: Module not found at {:?}", path);
                    std::process::exit(1);
                } else {
                    println!("Verified module existence: {:?}", path);
                }
            }

            // Verification: Check if meshes were resolved in URDF (using new path)
            let urdf_module_path = output_dir.join("ur5/urdf_module/module.json");
            let file = std::fs::File::open(urdf_module_path)
                .expect("Could not open generated URDF module");
            let module: apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule =
                serde_json::from_reader(file).expect("Could not parse URDF module");

            let mut found_mesh = false;
            for link in module.links {
                for visual in link.visual {
                    if let apollo_rust_modules::robot_modules::urdf_module::ApolloURDFGeometry::Mesh { filename, .. } = visual.geometry {
                        println!("Found mesh filename: {}", filename);
                        if !filename.starts_with("package://") {
                            found_mesh = true;
                        }
                    }
                }
            }

            if found_mesh {
                println!("Verification Success: found rewritten mesh paths.");
            } else {
                eprintln!("Verification Failure: All mesh paths still start with package:// or no meshes found.");
                std::process::exit(1);
            }
        }
        Err(e) => eprintln!("Error processing robot: {}", e),
    }
}
