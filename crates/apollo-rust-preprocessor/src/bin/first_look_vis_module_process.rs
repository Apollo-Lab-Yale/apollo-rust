use apollo_rust_preprocessor::process_functions::run_first_look_vis_module_process;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    assert_eq!(args.len(), 2);
    let chain_name = &args[1];

    run_first_look_vis_module_process(chain_name);
}
