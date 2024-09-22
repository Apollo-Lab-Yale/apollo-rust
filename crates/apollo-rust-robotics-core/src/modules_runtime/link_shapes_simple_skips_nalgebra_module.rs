use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};
use apollo_rust_linalg::dmatrix_from_2dvec;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use crate::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

/// The `ApolloLinkShapesSimpleSkipsNalgebraModule` struct holds simple skip matrices for different link shapes and representations.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesSimpleSkipsNalgebraModule {
    pub full_convex_hulls_simple_skips: DMatrix<bool>,
    pub full_obbs_simple_skips: DMatrix<bool>,
    pub full_bounding_spheres_simple_skips: DMatrix<bool>,
    pub decomposition_convex_hulls_simple_skips: DMatrix<bool>,
    pub decomposition_obbs_simple_skips: DMatrix<bool>,
    pub decomposition_bounding_spheres_simple_skips: DMatrix<bool>
}
impl ApolloLinkShapesSimpleSkipsNalgebraModule {
    /// Creates a new `ApolloLinkShapesSimpleSkipsNalgebraModule` from an `ApolloLinkShapesSimpleSkipsModule`.
    ///
    /// # Arguments
    /// - `apollo_link_shapes_simple_skips_module`: A reference to the `ApolloLinkShapesSimpleSkipsModule` containing the simple skips.
    ///
    /// # Returns
    /// An instance of `ApolloLinkShapesSimpleSkipsNalgebraModule`.
    pub fn from_link_shapes_simple_skips_module(apollo_link_shapes_simple_skips_module: &ApolloLinkShapesSimpleSkipsModule) -> Self {
        Self {
            full_convex_hulls_simple_skips: dmatrix_from_2dvec(&apollo_link_shapes_simple_skips_module.full_convex_hulls_simple_skips),
            full_obbs_simple_skips: dmatrix_from_2dvec(&apollo_link_shapes_simple_skips_module.full_obbs_simple_skips),
            full_bounding_spheres_simple_skips: dmatrix_from_2dvec(&apollo_link_shapes_simple_skips_module.full_bounding_spheres_simple_skips),
            decomposition_convex_hulls_simple_skips: dmatrix_from_2dvec(&apollo_link_shapes_simple_skips_module.decomposition_convex_hulls_simple_skips),
            decomposition_obbs_simple_skips: dmatrix_from_2dvec(&apollo_link_shapes_simple_skips_module.decomposition_obbs_simple_skips),
            decomposition_bounding_spheres_simple_skips: dmatrix_from_2dvec(&apollo_link_shapes_simple_skips_module.decomposition_bounding_spheres_simple_skips),
        }
    }

    /// Retrieves the simple skips matrix based on the provided `LinkShapeMode` and `LinkShapeRep`.
    ///
    /// # Arguments
    /// - `link_shape_mode`: The mode of the link shapes (full or decomposition).
    /// - `link_shape_rep`: The representation of the link shapes (convex hull, OBB, or bounding sphere).
    ///
    /// # Returns
    /// A reference to the corresponding `DMatrix<bool>` representing the simple skips.
    #[inline(always)]
    pub fn get_skips(&self, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep) -> &DMatrix<bool> {
        match &link_shape_mode {
            LinkShapeMode::Full => {
                match &link_shape_rep {
                    LinkShapeRep::ConvexHull => { &self.full_convex_hulls_simple_skips }
                    LinkShapeRep::OBB => { &self.full_obbs_simple_skips }
                    LinkShapeRep::BoundingSphere => { &self.full_bounding_spheres_simple_skips }
                }
            }
            LinkShapeMode::Decomposition => {
                match &link_shape_rep {
                    LinkShapeRep::ConvexHull => { &self.decomposition_convex_hulls_simple_skips }
                    LinkShapeRep::OBB => { &self.decomposition_obbs_simple_skips }
                    LinkShapeRep::BoundingSphere => { &self.decomposition_bounding_spheres_simple_skips }
                }
            }
        }
    }
}