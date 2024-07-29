use apollo_rust_robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_robot_modules::ResourcesSingleRobotDirectory;
use apollo_rust_robot_modules::urdf_module::ApolloURDFModule;
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;


impl PreprocessorModule for ApolloConnectionsModule {
    type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "connections_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false)?;
        let chain_module = ApolloChainModule::load_or_build(s, false)?;

        let num_links = urdf_module.links.len();

        let mut link_paths = vec![vec![None; num_links]; num_links];

        for i in 0..num_links {
            for j in 0..num_links {
                if i == j { link_paths[i][j] = Some(vec![j]) }

                let link_in_chain_i = &chain_module.links_in_chain[i];

                let mut stack: Vec<Vec<usize>> = link_in_chain_i.children_link_idxs().iter().map(|x| vec![*x]).collect();
                'l: while !stack.is_empty() {
                    let curr = stack.pop().unwrap();
                    let last = curr.last().unwrap();
                    let link_in_chain = &chain_module.links_in_chain[*last];
                    if link_in_chain.link_idx() == j { link_paths[i][j] = Some(curr); break 'l; }

                    link_in_chain.children_link_idxs().iter().for_each(|x| {
                        let mut curr_clone = curr.clone();
                        curr_clone.push(*x);
                        stack.push(curr_clone);
                    });
                }

            }
        }

        progress_bar.done_preset();
        Ok(Self { link_connection_paths: link_paths } )
    }
}

