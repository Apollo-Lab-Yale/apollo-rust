use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};
use apollo_rust_linalg::dmatrix_from_2dvec;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::{ApolloLinkShapesDistanceStatisticsModule, LinkShapesDistanceStatistics};
use crate::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

/// The `ApolloLinkShapesDistanceStatisticsNalgebraModule` struct holds distance statistics for different link shapes.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesDistanceStatisticsNalgebraModule {
    pub full_convex_hulls: LinkShapesDistanceStatisticsNalgebra,
    pub full_obbs: LinkShapesDistanceStatisticsNalgebra,
    pub full_bounding_spheres: LinkShapesDistanceStatisticsNalgebra,
    pub decomposition_convex_hulls: LinkShapesDistanceStatisticsNalgebra,
    pub decomposition_obbs: LinkShapesDistanceStatisticsNalgebra,
    pub decomposition_bounding_spheres: LinkShapesDistanceStatisticsNalgebra,
}
impl ApolloLinkShapesDistanceStatisticsNalgebraModule {
    /// Converts an `ApolloLinkShapesDistanceStatisticsModule` into a `ApolloLinkShapesDistanceStatisticsNalgebraModule`.
    ///
    /// # Arguments
    /// - `link_shapes_distance_statistics_module`: A reference to the `ApolloLinkShapesDistanceStatisticsModule` containing distance statistics.
    ///
    /// # Returns
    /// An instance of `ApolloLinkShapesDistanceStatisticsNalgebraModule`.
    pub fn from_link_shapes_distance_statistics_module(link_shapes_distance_statistics_module: &ApolloLinkShapesDistanceStatisticsModule) -> Self {
        Self {
            full_convex_hulls: LinkShapesDistanceStatisticsNalgebra::from_link_shapes_distance_statistics(&link_shapes_distance_statistics_module.full_convex_hulls),
            full_obbs: LinkShapesDistanceStatisticsNalgebra::from_link_shapes_distance_statistics(&link_shapes_distance_statistics_module.full_obbs),
            full_bounding_spheres: LinkShapesDistanceStatisticsNalgebra::from_link_shapes_distance_statistics(&link_shapes_distance_statistics_module.full_bounding_spheres),
            decomposition_convex_hulls: LinkShapesDistanceStatisticsNalgebra::from_link_shapes_distance_statistics(&link_shapes_distance_statistics_module.decomposition_convex_hulls),
            decomposition_obbs: LinkShapesDistanceStatisticsNalgebra::from_link_shapes_distance_statistics(&link_shapes_distance_statistics_module.decomposition_obbs),
            decomposition_bounding_spheres: LinkShapesDistanceStatisticsNalgebra::from_link_shapes_distance_statistics(&link_shapes_distance_statistics_module.decomposition_bounding_spheres),
        }
    }

    /// Retrieves the distance statistics based on the provided `LinkShapeRep` and `LinkShapeMode`.
    ///
    /// # Arguments
    /// - `link_shape_rep`: A reference to the representation mode of the link shape.
    /// - `link_shape_mode`: A reference to the mode of the link shape.
    ///
    /// # Returns
    /// A reference to the corresponding `LinkShapesDistanceStatisticsNalgebra`.
    pub fn get_stats(&self, link_shape_rep: &LinkShapeRep, link_shape_mode: &LinkShapeMode) -> &LinkShapesDistanceStatisticsNalgebra {
        return match link_shape_mode {
            LinkShapeMode::Full => {
                match link_shape_rep {
                    LinkShapeRep::ConvexHull => { &self.full_convex_hulls }
                    LinkShapeRep::OBB => { &self.full_obbs }
                    LinkShapeRep::BoundingSphere => { &self.full_bounding_spheres }
                }
            }
            LinkShapeMode::Decomposition => {
                match link_shape_rep {
                    LinkShapeRep::ConvexHull => { &self.decomposition_convex_hulls }
                    LinkShapeRep::OBB => { &self.decomposition_obbs }
                    LinkShapeRep::BoundingSphere => { &self.decomposition_bounding_spheres }
                }
            }
        }
    }
}

/// The `LinkShapesDistanceStatisticsNalgebra` struct holds statistical data for link shapes, including averages, maximums, and minimums.
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct LinkShapesDistanceStatisticsNalgebra {
    pub averages: DMatrix<f64>,
    pub maximums: DMatrix<f64>,
    pub minimums: DMatrix<f64>
}
impl LinkShapesDistanceStatisticsNalgebra {
    /// Converts a `LinkShapesDistanceStatistics` into a `LinkShapesDistanceStatisticsNalgebra`.
    ///
    /// # Arguments
    /// - `link_shapes_distance_statistics`: A reference to the `LinkShapesDistanceStatistics` containing the statistical data.
    ///
    /// # Returns
    /// An instance of `LinkShapesDistanceStatisticsNalgebra`.
    pub fn from_link_shapes_distance_statistics(link_shapes_distance_statistics: &LinkShapesDistanceStatistics) -> Self {
        Self {
            averages: dmatrix_from_2dvec(&link_shapes_distance_statistics.averages),
            maximums: dmatrix_from_2dvec(&link_shapes_distance_statistics.maximums),
            minimums: dmatrix_from_2dvec(&link_shapes_distance_statistics.minimums),
        }
    }
}