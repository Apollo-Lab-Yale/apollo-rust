/// The apollo-rust-robot-modules crate has all modules as structs without any initialization code.
/// This allows for these structs to be loaded into other crates that can use them while avoiding
/// circular dependencies.

pub mod urdf_module;
pub mod chain_module;
pub mod mesh_modules;
pub mod dof_module;
pub mod connections_module;
pub mod link_shapes_max_distance_from_origin_module;
pub mod link_shapes_distance_statistics_module;
pub mod bounds_module;
