use apollo_rust_modules::ResourcesRootDirectory;


fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    // let c = r.get_subdirectory("b1_floating_base").to_chain_nalgebra();

    // ApolloLinkShapesSkipsModule::load_or_build(&r.get_subdirectory("b1"), true).expect("error");
}