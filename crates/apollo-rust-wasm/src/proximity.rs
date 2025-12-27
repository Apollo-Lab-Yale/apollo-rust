use crate::robot_modules::WASMRobotModules;
use apollo_rust_linalg::V;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use apollo_rust_robotics_core::robot_functions::robot_proximity_functions::RobotProximityFunctions;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct WASMProximity {}

#[wasm_bindgen]
impl WASMProximity {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {}
    }

    /// Checks for self-collision given joint angles.
    /// Returns true if any collision is found.
    /// Uses ConvexHull representation for speed by default.
    pub fn check_self_collision(
        &self,
        modules: &WASMRobotModules,
        joint_angles: &[f64],
    ) -> Result<bool, JsValue> {
        if !modules.is_ready_for_proximity() {
            return Err(JsValue::from_str(
                "Modules not ready for proximity. Load LinkShapes module too.",
            ));
        }

        let urdf_module = modules.urdf_nalgebra_module.as_ref().unwrap();
        let chain_module = modules.chain_module.as_ref().unwrap();
        let dof_module = modules.dof_module.as_ref().unwrap();
        let link_shapes_module = modules.link_shapes_module.as_ref().unwrap();

        if joint_angles.len() != dof_module.num_dofs {
            return Err(JsValue::from_str(&format!(
                "Incorrect number of joint angles. Expected {}, got {}",
                dof_module.num_dofs,
                joint_angles.len()
            )));
        }

        let state = V::from_column_slice(joint_angles);
        let link_poses =
            RobotKinematicsFunctions::fk(&state, urdf_module, chain_module, dof_module);

        // Perform self-intersection check
        let res = RobotProximityFunctions::self_intersect(
            link_shapes_module,
            &link_poses,
            LinkShapeMode::Decomposition, // Using decomposition for more accurate collision
            LinkShapeRep::ConvexHull,     // Convex Hull is standard
            None,                         // No skips for now (or we could expose loading skips)
            true,                         // early stop
        );

        // If outputs is not empty, we found a collision.
        // The DoubleGroupProximityQueryOutput stores true for each collision found.
        if !res.outputs.is_empty() {
            Ok(true)
        } else {
            Ok(false)
        }
    }
}
