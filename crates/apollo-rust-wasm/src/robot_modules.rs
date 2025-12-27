use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::ApolloLinkShapesModule;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use serde_json;
use std::path::PathBuf;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct WASMRobotModules {
    #[wasm_bindgen(skip)]
    pub urdf_module: Option<ApolloURDFModule>,
    #[wasm_bindgen(skip)]
    pub chain_module: Option<ApolloChainModule>,
    #[wasm_bindgen(skip)]
    pub dof_module: Option<ApolloDOFModule>,
    #[wasm_bindgen(skip)]
    pub urdf_nalgebra_module: Option<ApolloURDFNalgebraModule>,
    #[wasm_bindgen(skip)]
    pub link_shapes_module: Option<ApolloLinkShapesModule>,
    #[wasm_bindgen(skip)]
    pub convex_hull_meshes_module: Option<ApolloConvexHullMeshesModule<PathBuf>>,
    #[wasm_bindgen(skip)]
    pub convex_decomposition_meshes_module: Option<ApolloConvexDecompositionMeshesModule<PathBuf>>,
}

#[wasm_bindgen]
impl WASMRobotModules {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            urdf_module: None,
            chain_module: None,
            dof_module: None,
            urdf_nalgebra_module: None,
            link_shapes_module: None,
            convex_hull_meshes_module: None,
            convex_decomposition_meshes_module: None,
        }
    }

    pub fn load_urdf_module(&mut self, json_str: &str) -> Result<(), JsValue> {
        let module: ApolloURDFModule = serde_json::from_str(json_str)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse URDF Module JSON: {}", e)))?;
        self.urdf_nalgebra_module = Some(ApolloURDFNalgebraModule::from_urdf_module(&module));
        self.urdf_module = Some(module);
        Ok(())
    }

    pub fn load_chain_module(&mut self, json_str: &str) -> Result<(), JsValue> {
        let module: ApolloChainModule = serde_json::from_str(json_str)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse Chain Module JSON: {}", e)))?;
        self.chain_module = Some(module);
        Ok(())
    }

    pub fn load_dof_module(&mut self, json_str: &str) -> Result<(), JsValue> {
        let module: ApolloDOFModule = serde_json::from_str(json_str)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse DOF Module JSON: {}", e)))?;
        self.dof_module = Some(module);
        Ok(())
    }

    pub fn load_convex_hull_meshes_module(&mut self, json_str: &str) -> Result<(), JsValue> {
        let module: ApolloConvexHullMeshesModule<PathBuf> = serde_json::from_str(json_str)
            .map_err(|e| {
                JsValue::from_str(&format!(
                    "Failed to parse Convex Hull Meshes Module JSON: {}",
                    e
                ))
            })?;
        self.convex_hull_meshes_module = Some(module);
        Ok(())
    }

    pub fn load_convex_decomposition_meshes_module(
        &mut self,
        json_str: &str,
    ) -> Result<(), JsValue> {
        let module: ApolloConvexDecompositionMeshesModule<PathBuf> = serde_json::from_str(json_str)
            .map_err(|e| {
                JsValue::from_str(&format!(
                    "Failed to parse Convex Decomposition Meshes Module JSON: {}",
                    e
                ))
            })?;
        self.convex_decomposition_meshes_module = Some(module);
        Ok(())
    }

    pub fn is_ready(&self) -> bool {
        self.urdf_module.is_some() && self.chain_module.is_some() && self.dof_module.is_some()
    }

    pub fn is_ready_for_proximity(&self) -> bool {
        self.is_ready() && self.link_shapes_module.is_some()
    }
}
