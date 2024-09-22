use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use crate::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

/// The `LinkShapesMaxDistanceFromOriginTrait` trait provides a method to retrieve the maximum distances
/// from the origin for different link shapes and representations in shape indices.
pub trait LinkShapesMaxDistanceFromOriginTrait {
    /// Retrieves the maximum distances from the origin for the specified `LinkShapeMode` and `LinkShapeRep`.
    /// The result is in shape index, not link index.
    ///
    /// # Arguments
    /// - `link_shape_mode`: The mode of the link shapes (full or decomposition).
    /// - `link_shape_rep`: The representation of the link shapes (convex hull, OBB, or bounding sphere).
    ///
    /// # Returns
    /// A `Vec<f64>` containing the maximum distances from the origin for the shapes.
    fn get_shapes_max_distances_from_origin(&self, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep) -> Vec<f64>;
}
impl LinkShapesMaxDistanceFromOriginTrait for ApolloLinkShapesMaxDistanceFromOriginModule {
    fn get_shapes_max_distances_from_origin(&self, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep) -> Vec<f64> {
        match link_shape_mode {
            LinkShapeMode::Full => {
                match link_shape_rep {
                    LinkShapeRep::ConvexHull => {
                        self.full_convex_hulls_maximum_distances.iter().filter_map(|x| {
                            match x {
                                None => { None }
                                Some(x) => { Some(*x) }
                            }
                        }).collect()
                    }
                    LinkShapeRep::OBB => {
                        self.full_obbs_maximum_distances.iter().filter_map(|x| {
                            match x {
                                None => { None }
                                Some(x) => { Some(*x) }
                            }
                        }).collect()
                    }
                    LinkShapeRep::BoundingSphere => {
                        self.full_bounding_spheres_maximum_distances.iter().filter_map(|x| {
                            match x {
                                None => { None }
                                Some(x) => { Some(*x) }
                            }
                        }).collect()
                    }
                }
            }
            LinkShapeMode::Decomposition => {
                match link_shape_rep {
                    LinkShapeRep::ConvexHull => {
                        self.decomposition_convex_hulls_maximum_distances.clone().into_iter().flatten().collect()
                    }
                    LinkShapeRep::OBB => {
                        self.decomposition_obbs_maximum_distances.clone().into_iter().flatten().collect()
                    }
                    LinkShapeRep::BoundingSphere => {
                        self.decomposition_bounding_spheres_maximum_distances.clone().into_iter().flatten().collect()
                    }
                }
            }
        }
    }
}