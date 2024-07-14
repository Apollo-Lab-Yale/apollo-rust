use std::collections::HashMap;
use apollo_rust_robot_modules::chain_module::{ApolloChainModule, ApolloJointInChain, ApolloLinkInChain};
use apollo_rust_robot_modules::urdf_module::ApolloURDFModule;
use crate::{RobotPreprocessorModule, RobotPreprocessorSingleRobotDirectory};
use crate::utils::progress_bar::ProgressBarWrapper;

pub trait ChainModuleBuilders: Sized {
    fn build_from_urdf_module(urdf_module: &ApolloURDFModule, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
}
impl ChainModuleBuilders for ApolloChainModule {
    fn build_from_urdf_module(urdf_module: &ApolloURDFModule, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let mut links = vec![];
        let mut curr_idx = 0;
        let mut link_name_to_link_idx_map = HashMap::new();
        urdf_module.links.iter().for_each(|x| {
            links.push(ApolloLinkInChain {
                name: x.name.clone(),
                link_idx: curr_idx,
                parent_joint_idx: None,
                parent_link_idx: None,
                children_joint_idxs: vec![],
                children_link_idxs: vec![],
            });
            link_name_to_link_idx_map.insert(x.name.clone(), curr_idx);
            curr_idx += 1;
        });

        let mut joints = vec![];
        let mut curr_idx = 0;
        urdf_module.joints.iter().for_each(|x| {
            let parent_link_idx = *link_name_to_link_idx_map.get(&x.parent.link).expect("error");
            let child_link_idx = *link_name_to_link_idx_map.get(&x.child.link).expect("error");

            joints.push(ApolloJointInChain {
                joint_name: x.name.clone(),
                joint_idx: curr_idx,
                parent_link_name: x.parent.link.clone(),
                parent_link_idx: *link_name_to_link_idx_map.get(&x.parent.link).expect("error"),
                child_link_name: x.child.link.clone(),
                child_link_idx: *link_name_to_link_idx_map.get(&x.child.link).expect("error"),
            });

            links[child_link_idx].parent_link_idx = Some(parent_link_idx);
            links[parent_link_idx].children_link_idxs.push(child_link_idx);
            links[child_link_idx].parent_joint_idx = Some(curr_idx);
            links[parent_link_idx].children_joint_idxs.push(curr_idx);

            curr_idx += 1;
        });

        let mut kinematic_hierarchy = vec![];
        let mut root_idx = 0;
        let mut set = vec![false; links.len()];
        'l: for link in &links {
            if link.parent_link_idx.is_none() { kinematic_hierarchy.push(vec![link.link_idx]); set[link.link_idx] = true; root_idx = link.link_idx; break 'l; }
        }

        while !set.iter().all(|x| *x == true) {
            let mut v = vec![];

            let last = kinematic_hierarchy.last().unwrap();
            for link in &links {
                if link.parent_link_idx.is_some() && last.contains(&link.parent_link_idx.unwrap()) {
                    v.push(link.link_idx);
                    set[link.link_idx] = true;
                }
            }

            kinematic_hierarchy.push(v);
        }

        progress_bar.done_preset();
        Ok(Self {
            links_in_chain: links,
            joints_in_chain: joints,
            kinematic_hierarchy,
            root_idx,
        })
    }
}

impl RobotPreprocessorModule for ApolloChainModule {
    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String {
        "chain_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &RobotPreprocessorSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false).expect(&format!("could not build ChainModule module in {:?} because urdf module could not be loaded or built.", s.directory));
        return Self::build_from_urdf_module(&urdf_module, progress_bar);
    }
}