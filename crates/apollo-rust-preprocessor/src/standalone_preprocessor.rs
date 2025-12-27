use crate::robot_modules_preprocessor::modules::chain_module::ChainModuleBuilders;
use crate::robot_modules_preprocessor::modules::connections_module::ConnectionsModuleBuilders;
use crate::robot_modules_preprocessor::modules::dof_module::DOFModuleBuilders;
use crate::PreprocessorModule;
use crate::robot_modules_preprocessor::modules::mesh_modules::convex_decomposition_meshes_module::ConvexDecompositionMeshesModuleBuilders;
use crate::robot_modules_preprocessor::modules::mesh_modules::convex_hull_meshes_module::ConvexHullMeshesModuleBuilders;
use crate::robot_modules_preprocessor::modules::mesh_modules::original_meshes_module::OriginalMeshesModuleBuilders;
use crate::robot_modules_preprocessor::modules::mesh_modules::plain_meshes_module::PlainMeshesModuleBuilders;
use crate::utils::progress_bar::ProgressBarWrapper;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::ApolloLinkShapesDistanceStatisticsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::{
    ApolloURDFGeometry, ApolloURDFJoint, ApolloURDFLink, ApolloURDFMaterial, ApolloURDFModule,
};
use apollo_rust_modules::{ResourcesSubDirectory, ResourcesType};
use std::collections::HashMap;
use urdf_rs::read_from_string;

pub struct StandalonePreprocessor;

impl StandalonePreprocessor {
    pub fn process_robot<P: ApolloPathBufTrait + Clone>(
        urdf_path: &P,
        mesh_dir: &P,
        output_dir: &P,
    ) -> Result<(), String> {
        println!("Processing robot in standalone mode (Generic P)...");

        // 1. Read URDF or Xacro
        let urdf_string = if let Some(ext) = urdf_path.path_extension() {
            if ext.to_lowercase() == "xacro" {
                println!("Detected Xacro file, parsing...");
                let xacro_string = urdf_path.read_file_contents_to_string_result()?;
                let base_path = urdf_path.parent().unwrap_or(urdf_path.clone());
                apollo_rust_xacro::parser::XacroParser::parse(
                    &xacro_string,
                    &base_path,
                    &[mesh_dir.clone()],
                    None,
                )
                .map_err(|e| format!("Xacro parsing error: {:?}", e))?
            } else {
                urdf_path.read_file_contents_to_string_result()?
            }
        } else {
            urdf_path.read_file_contents_to_string_result()?
        };

        let urdf_res = read_from_string(&urdf_string);
        let urdf_robot = match urdf_res {
            Ok(r) => r,
            Err(e) => return Err(format!("Error parsing URDF: {:?}", e)),
        };

        // 2. Create Output Directory if it doesn't exist
        if !output_dir.path_exists() {
            output_dir.create_directory();
        }

        // 3. Generate ApolloURDFModule
        let mut urdf_module = ApolloURDFModule {
            name: urdf_robot.name.clone(),
            links: urdf_robot
                .links
                .iter()
                .map(|x| ApolloURDFLink::from_link(x))
                .collect(),
            joints: urdf_robot
                .joints
                .iter()
                .map(|x| ApolloURDFJoint::from_joint(x))
                .collect(),
            materials: urdf_robot
                .materials
                .iter()
                .map(|x| ApolloURDFMaterial::from_material(x))
                .collect(),
        };

        // 4. Index and Resolve Meshes
        println!("Indexing mesh directory: {:?}", mesh_dir);
        let mesh_map = index_mesh_directory(mesh_dir);
        resolve_meshes(&mut urdf_module, &mesh_map, mesh_dir);

        // Virtual ResourcesSubDirectory
        // We construct a fake one to satisfy the builder traits.
        let s = ResourcesSubDirectory {
            name: urdf_robot.name.clone(),
            root_directory: output_dir.clone(), // This is a bit hacky, root and dir are same here?
            directory: output_dir.clone().append(&urdf_robot.name),
            resources_type: ResourcesType::Robot,
        };
        s.directory.create_directory();

        // Save URDF Module
        urdf_module.save(&s);

        // Save raw processed URDF string
        let urdf_file_path = s.directory.clone().append("robot.urdf");
        urdf_file_path.write_string_to_file(&urdf_string);

        // The builders typically look for modules relative to `s.directory`.
        // If `output_dir` IS the robot directory (e.g. `standalone_output`), then this is correct.

        // 5. Generate other key modules

        // DOF Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "DOF Module");
        let dof_module = ApolloDOFModule::build_from_urdf_module(&urdf_module, &mut pb)?;
        dof_module.save(&s);

        // Chain Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Chain Module");
        let chain_module = ApolloChainModule::build_from_urdf_module(&urdf_module, &mut pb)?;
        chain_module.save(&s);

        // Connections Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Connections Module");
        let connections_module = ApolloConnectionsModule::build_from_urdf_and_chain_modules(
            &urdf_module,
            &chain_module,
            &mut pb,
        )?;
        connections_module.save(&s);

        // --- NEW MODULES ---

        // Original Meshes Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Original Meshes");
        let original_meshes_module =
            ApolloOriginalMeshesModule::build_from_urdf_module(&s, &mut pb)?;
        original_meshes_module.save(&s);

        // Plain Meshes Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Plain Meshes");
        let plain_meshes_module =
            ApolloPlainMeshesModule::build_from_original_meshes_module(&s, &mut pb)?;
        plain_meshes_module.save(&s);

        // Convex Hull Meshes Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Convex Hull Meshes");
        let convex_hull_meshes_module =
            ApolloConvexHullMeshesModule::build_from_plain_meshes_module(&s, &mut pb)?;
        convex_hull_meshes_module.save(&s);

        // Convex Decomposition Meshes Module (Expensive!)
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Convex Decomposition (VHACD)");
        let convex_decomposition_meshes_module =
            ApolloConvexDecompositionMeshesModule::build_from_plain_meshes_module(&s, &mut pb)?;
        convex_decomposition_meshes_module.save(&s);

        // Bounds Module
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Bounds Module");
        let bounds_module = ApolloBoundsModule::build_raw(&s, &mut pb)?;
        bounds_module.save(&s);

        // Link Shapes Modules
        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Link Shapes (Max Dist)");
        let max_dist_module = ApolloLinkShapesMaxDistanceFromOriginModule::build_raw(&s, &mut pb)?;
        max_dist_module.save(&s);

        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Link Shapes (Distance Stats)");
        let distance_stats_module =
            ApolloLinkShapesDistanceStatisticsModule::build_raw(&s, &mut pb)?;
        distance_stats_module.save(&s);

        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Link Shapes (Simple Skips)");
        let simple_skips_module = ApolloLinkShapesSimpleSkipsModule::build_raw(&s, &mut pb)?;
        simple_skips_module.save(&s);

        let mut pb = ProgressBarWrapper::new(&urdf_robot.name, "Link Shapes (Approximations)");
        let approximations_module = ApolloLinkShapesApproximationsModule::build_raw(&s, &mut pb)?;
        approximations_module.save(&s);

        println!("Standalone processing complete.");
        Ok(())
    }
}

// Helper: Trait needed for saving?
// PreprocessorModule trait has save method.

fn index_mesh_directory<P: ApolloPathBufTrait + Clone>(mesh_dir: &P) -> HashMap<String, P> {
    let mut map = HashMap::new();
    index_mesh_directory_recursive(mesh_dir, &mut map);
    map
}

fn index_mesh_directory_recursive<P: ApolloPathBufTrait + Clone>(
    dir: &P,
    map: &mut HashMap<String, P>,
) {
    // 1. Get all files
    let files = dir.get_all_items_in_directory(false, false, true, false);
    for file in files {
        if let Some(ext) = file.path_extension() {
            let ext = ext.to_lowercase();
            if vec![
                "dae", "stl", "obj", "glb", "gltf", "DAE", "STL", "OBJ", "GLB", "GLTF",
            ]
            .contains(&ext.as_str())
            {
                let segments = file.split_into_strings();
                if let Some(filename) = segments.last() {
                    map.insert(filename.clone(), file);
                }
            }
        }
    }

    // 2. Get all directories
    let dirs = dir.get_all_items_in_directory(true, false, false, false);
    for sub_dir in dirs {
        index_mesh_directory_recursive(&sub_dir, map);
    }
}

fn resolve_meshes<P: ApolloPathBufTrait + Clone>(
    module: &mut ApolloURDFModule,
    mesh_map: &HashMap<String, P>,
    root_mesh_dir: &P,
) {
    for link in &mut module.links {
        // Visuals
        for visual in &mut link.visual {
            resolve_geometry(&mut visual.geometry, mesh_map, root_mesh_dir);
        }
        // Collisions
        for collision in &mut link.collision {
            resolve_geometry(&mut collision.geometry, mesh_map, root_mesh_dir);
        }
    }
}

fn resolve_geometry<P: ApolloPathBufTrait + Clone>(
    geometry: &mut ApolloURDFGeometry,
    mesh_map: &HashMap<String, P>,
    _root_mesh_dir: &P,
) {
    if let ApolloURDFGeometry::Mesh { filename, .. } = geometry {
        // Extract just the filename from the path
        // e.g. "package://ur5_description/meshes/ur5/visual/base.dae" -> "base.dae"
        let path_parts: Vec<&str> = filename.split('/').collect();
        if let Some(name) = path_parts.last() {
            if let Some(resolved_path) = mesh_map.get(*name) {
                // We found a match!
                // We need to convert this absolute path P to something relative or absolute that the engine uses.
                // For standalone, usually we want the relative path from the output directory or just the absolute path if it's virtual.
                // The `filename` field is a String.
                // If we are in WASM, P is WebPathBuf, which is essentially a string key.
                // So calling .to_path_buf() might not be what we want if we want the string representation.
                // We can use `.split_into_strings()` and join them, or some to_string() equivalent?
                // ApolloPathBufTrait doesn't force ToString, but usually P is PathBuf or WebPathBuf.
                // Let's assume we can get a string representation.
                // WebPathBuf usually just holds the path string.

                // Let's try to get a string rep.
                // For PathBuf, to_str(). For WebPathBuf, it probably implements Display or similar.
                // Actually, the trait requires Debug.
                // But we can use `split_into_strings().join("/")` or similar to reconstruct for now if needed,
                // OR we can rely on `get_a_to_b_path` if we want relative.
                // Let's just use the absolute path string if possible.
                // Wait, `filename` in URDF is a string.
                // We can use `resolved_path.split_into_strings().join("/")` to be safe and cross-platformish?
                // Or checking trait methods... `to_path_buf()` returns `PathBuf`. `PathBuf` has `to_str()`.
                // This assumes `to_path_buf` works for `WebPathBuf`. It might return a dummy `PathBuf`.
                // Let's look at `WebPathBuf` later.
                // For now, let's use a heuristic:
                // Construct a string key.

                // Hack: use the Debug formatted string? No.
                // Hack: use split_into_strings.
                let parts = resolved_path.split_into_strings();
                let new_path = parts.join("/");
                *filename = new_path;
            } else {
                println!("Warning: Could not resolve mesh: {}", filename);
            }
        }
    }
}
