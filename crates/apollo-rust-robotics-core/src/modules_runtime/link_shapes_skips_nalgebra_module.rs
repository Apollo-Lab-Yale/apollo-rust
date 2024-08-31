use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};
use apollo_rust_linalg::dmatrix_from_2dvec;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use crate::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesSkipsNalgebraModule {
    pub full_convex_hulls_skips: DMatrix<bool>,
    pub full_obbs_skips: DMatrix<bool>,
    pub full_bounding_spheres_skips: DMatrix<bool>,
    pub decomposition_convex_hulls_skips: DMatrix<bool>,
    pub decomposition_obbs_skips: DMatrix<bool>,
    pub decomposition_bounding_spheres_skips: DMatrix<bool>
}
impl ApolloLinkShapesSkipsNalgebraModule {
    pub fn from_link_shapes_skips_module(apollo_link_shapes_skips_module: &ApolloLinkShapesSkipsModule) -> Self {
        Self {
            full_convex_hulls_skips: dmatrix_from_2dvec(&apollo_link_shapes_skips_module.full_convex_hulls_skips),
            full_obbs_skips: dmatrix_from_2dvec(&apollo_link_shapes_skips_module.full_obbs_skips),
            full_bounding_spheres_skips: dmatrix_from_2dvec(&apollo_link_shapes_skips_module.full_bounding_spheres_skips),
            decomposition_convex_hulls_skips: dmatrix_from_2dvec(&apollo_link_shapes_skips_module.decomposition_convex_hulls_skips),
            decomposition_obbs_skips: dmatrix_from_2dvec(&apollo_link_shapes_skips_module.decomposition_obbs_skips),
            decomposition_bounding_spheres_skips: dmatrix_from_2dvec(&apollo_link_shapes_skips_module.decomposition_bounding_spheres_skips),
        }
    }

    #[inline(always)]
    pub fn get_skips(&self, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep) -> &DMatrix<bool> {
        match &link_shape_mode {
            LinkShapeMode::Full => {
                match &link_shape_rep {
                    LinkShapeRep::ConvexHull => { &self.full_convex_hulls_skips }
                    LinkShapeRep::OBB => { &self.full_obbs_skips }
                    LinkShapeRep::BoundingSphere => { &self.full_bounding_spheres_skips }
                }
            }
            LinkShapeMode::Decomposition => {
                match &link_shape_rep {
                    LinkShapeRep::ConvexHull => { &self.decomposition_convex_hulls_skips }
                    LinkShapeRep::OBB => { &self.decomposition_obbs_skips }
                    LinkShapeRep::BoundingSphere => { &self.decomposition_bounding_spheres_skips }
                }
            }
        }
    }
}