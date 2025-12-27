use crate::robot_modules::WASMRobotModules;
use apollo_rust_linalg::V;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct WASMKinematics {
    // Stateless helper
}

#[wasm_bindgen]
impl WASMKinematics {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {}
    }

    /// Computes FK and returns a flattened vector of poses.
    /// Each pose is 7 elements: [tx, ty, tz, qx, qy, qz, qw]
    pub fn compute_fk(
        &self,
        modules: &WASMRobotModules,
        joint_angles: &[f64],
    ) -> Result<Vec<f64>, JsValue> {
        if !modules.is_ready() {
            return Err(JsValue::from_str(
                "Modules not ready. Load URDF, Chain, and DOF modules first.",
            ));
        }

        let urdf_module = modules.urdf_nalgebra_module.as_ref().unwrap();
        let chain_module = modules.chain_module.as_ref().unwrap();
        let dof_module = modules.dof_module.as_ref().unwrap();

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

        let mut out = Vec::with_capacity(link_poses.len() * 7);
        for pose in link_poses {
            let t = pose.0.translation.vector;
            let q = pose.0.rotation.quaternion();

            out.push(t[0]);
            out.push(t[1]);
            out.push(t[2]);
            out.push(q.i);
            out.push(q.j);
            out.push(q.k);
            out.push(q.w);
        }

        Ok(out)
    }
}
