/*
use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};
use apollo_rust_linalg::dmatrix_from_2dvec;
use apollo_rust_proximity::proxima::proxima2::LieAlgMode;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_lie_alg_error_models_module::ApolloLinkShapesLieAlgErrorModelsModule;
use crate::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesLieAlgErrorModelsNalgebraModule {
    pub full_convex_hulls_standard_models: DMatrix<([f64; 3], [f64; 3])>,
    pub full_obbs_standard_models: DMatrix<([f64; 3], [f64; 3])>,
    pub full_bounding_spheres_standard_models: DMatrix<([f64; 3], [f64; 3])>,
    pub decomposition_convex_hulls_standard_models: DMatrix<([f64; 3], [f64; 3])>,
    pub decomposition_obbs_standard_models: DMatrix<([f64; 3], [f64; 3])>,
    pub decomposition_bounding_spheres_standard_models: DMatrix<([f64; 3], [f64; 3])>,

    pub full_convex_hulls_pseudo_models: DMatrix<([f64; 3], [f64; 3])>,
    pub full_obbs_pseudo_models: DMatrix<([f64; 3], [f64; 3])>,
    pub full_bounding_spheres_pseudo_models: DMatrix<([f64; 3], [f64; 3])>,
    pub decomposition_convex_hulls_pseudo_models: DMatrix<([f64; 3], [f64; 3])>,
    pub decomposition_obbs_pseudo_models: DMatrix<([f64; 3], [f64; 3])>,
    pub decomposition_bounding_spheres_pseudo_models: DMatrix<([f64; 3], [f64; 3])>
}
impl ApolloLinkShapesLieAlgErrorModelsNalgebraModule {
    pub fn from_link_shapes_lie_alg_error_models_module(apollo_link_shapes_lie_alg_error_models_module: &ApolloLinkShapesLieAlgErrorModelsModule) -> Self {
        Self {
            full_convex_hulls_standard_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.full_convex_hulls_standard_models),
            full_obbs_standard_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.full_obbs_standard_models),
            full_bounding_spheres_standard_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.full_bounding_spheres_standard_models),
            decomposition_convex_hulls_standard_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.decomposition_convex_hulls_standard_models),
            decomposition_obbs_standard_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.decomposition_obbs_standard_models),
            decomposition_bounding_spheres_standard_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.decomposition_bounding_spheres_standard_models),

            full_convex_hulls_pseudo_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.full_convex_hulls_pseudo_models),
            full_obbs_pseudo_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.full_obbs_pseudo_models),
            full_bounding_spheres_pseudo_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.full_bounding_spheres_pseudo_models),
            decomposition_convex_hulls_pseudo_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.decomposition_convex_hulls_pseudo_models),
            decomposition_obbs_pseudo_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.decomposition_obbs_pseudo_models),
            decomposition_bounding_spheres_pseudo_models: dmatrix_from_2dvec(&apollo_link_shapes_lie_alg_error_models_module.decomposition_bounding_spheres_pseudo_models),
        }
    }

    #[inline(always)]
    pub fn get_model(&self, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, lie_alg_mode: LieAlgMode) -> &DMatrix<([f64; 3], [f64; 3])> {
        match (link_shape_mode, link_shape_rep, lie_alg_mode) {
            (LinkShapeMode::Full, LinkShapeRep::ConvexHull, LieAlgMode::Standard) => &self.full_convex_hulls_standard_models,
            (LinkShapeMode::Full, LinkShapeRep::OBB, LieAlgMode::Standard) => &self.full_obbs_standard_models,
            (LinkShapeMode::Full, LinkShapeRep::BoundingSphere, LieAlgMode::Standard) => &self.full_bounding_spheres_standard_models,
            (LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, LieAlgMode::Standard) => &self.decomposition_convex_hulls_standard_models,
            (LinkShapeMode::Decomposition, LinkShapeRep::OBB, LieAlgMode::Standard) => &self.decomposition_obbs_standard_models,
            (LinkShapeMode::Decomposition, LinkShapeRep::BoundingSphere, LieAlgMode::Standard) => &self.decomposition_bounding_spheres_standard_models,

            (LinkShapeMode::Full, LinkShapeRep::ConvexHull, LieAlgMode::Pseudo) => &self.full_convex_hulls_pseudo_models,
            (LinkShapeMode::Full, LinkShapeRep::OBB, LieAlgMode::Pseudo) => &self.full_obbs_pseudo_models,
            (LinkShapeMode::Full, LinkShapeRep::BoundingSphere, LieAlgMode::Pseudo) => &self.full_bounding_spheres_pseudo_models,
            (LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, LieAlgMode::Pseudo) => &self.decomposition_convex_hulls_pseudo_models,
            (LinkShapeMode::Decomposition, LinkShapeRep::OBB, LieAlgMode::Pseudo) => &self.decomposition_obbs_pseudo_models,
            (LinkShapeMode::Decomposition, LinkShapeRep::BoundingSphere, LieAlgMode::Pseudo) => &self.decomposition_bounding_spheres_pseudo_models,
        }
    }
}
*/