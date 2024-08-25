use apollo_rust_proximity::proxima::proxima2::{get_lladis_taylor_series_error_dataset, LieAlgMode, PolynomialFit, quantile_optimization};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let c = r.get_subdirectory("b1").to_chain_nalgebra();

    let mut s = c.link_shapes_module.get_shapes(LinkShapeMode::Full, LinkShapeRep::ConvexHull).clone();
    s.extend(c.link_shapes_module.get_shapes(LinkShapeMode::Decomposition, LinkShapeRep::OBB).clone());
    s.extend(c.link_shapes_module.get_shapes(LinkShapeMode::Decomposition, LinkShapeRep::BoundingSphere).clone());
    s.extend(c.link_shapes_module.get_shapes(LinkShapeMode::Full, LinkShapeRep::OBB).clone());

    // let ds = get_lladis_taylor_series_error_dataset(&s, LieAlgMode::Pseudo, 100000, 10, 10, 2.0, 2.0);
    // let res = quantile_optimization(0.999, &ds, &PolynomialFit::LinearNoIntercept);
    // println!("{:?}", res);
}