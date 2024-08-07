use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_linalg::V;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::urdf_module::ApolloURDFJointType;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::{ApolloLieAlgPackIse3qTrait, ISE3q};
use apollo_rust_spatial::vectors::{V3, V6};
use crate::modules_runtime::urdf_nalgebra_module::{ApolloURDFAxisNalgebra, ApolloURDFNalgebraModule};

pub struct RobotKinematicsFunctions;
impl RobotKinematicsFunctions {
    #[inline]
    pub fn fk(state: &V, urdf_module: &ApolloURDFNalgebraModule, chain_module: &ApolloChainModule, dof_module: &ApolloDOFModule) -> Vec<ISE3q> {
        assert_eq!(state.len(), dof_module.num_dofs);

        let links = &urdf_module.links;
        let joints = &urdf_module.joints;
        let kinematic_hierarchy = &chain_module.kinematic_hierarchy;
        let joint_idx_to_dofs_mapping = &dof_module.joint_idx_to_dof_idxs_mapping;

        let num_links = links.len();
        let mut out = vec![ISE3q::identity_element(); num_links];

        'l: for (i, layer) in kinematic_hierarchy.iter().enumerate() {
            if i == 0 { continue 'l; }

            layer.iter().for_each(|link_idx| {
                let link_in_chain = &chain_module.links_in_chain[*link_idx];
                let parent_link_idx = link_in_chain.parent_link_idx().expect("error");
                let parent_joint_idx = link_in_chain.parent_joint_idx().expect("error");
                let parent_joint = &joints[parent_joint_idx];

                let constant_transform = &parent_joint.origin.ise3q;
                let dof_idxs = &joint_idx_to_dofs_mapping[parent_joint_idx];
                let joint_dofs: Vec<f64> = dof_idxs.iter().map(|i| state[*i]).collect();
                let joint_axis = &parent_joint.axis;
                let joint_type = &parent_joint.joint_type;
                let variable_transform = Self::get_joint_variable_transform_urdf_axis(joint_type, joint_axis, &joint_dofs);

                out[*link_idx] = out[parent_link_idx].group_operator(constant_transform).group_operator(&variable_transform);
            });
        }

        out
    }

    #[inline(always)]
    fn get_joint_variable_transform_urdf_axis(joint_type: &ApolloURDFJointType, joint_axis: &ApolloURDFAxisNalgebra, joint_dofs: &[f64]) -> ISE3q {
        Self::get_joint_variable_transform(joint_type, &joint_axis.axis, joint_dofs)
    }

    #[inline(always)]
    pub fn get_joint_variable_transform(joint_type: &ApolloURDFJointType, joint_axis: &V3, joint_dofs: &[f64]) -> ISE3q {
        return match joint_type {
            ApolloURDFJointType::Revolute => {
                assert_eq!(joint_dofs.len(), 1);
                let sa = joint_dofs[0] * joint_axis;
                ISE3q::new(I3::from_slices_scaled_axis(&[0., 0., 0.], sa.as_slice()))
            }
            ApolloURDFJointType::Continuous => {
                assert_eq!(joint_dofs.len(), 1);
                let sa = joint_dofs[0] * joint_axis;
                ISE3q::new(I3::from_slices_scaled_axis(&[0., 0., 0.], sa.as_slice()))
            }
            ApolloURDFJointType::Prismatic => {
                assert_eq!(joint_dofs.len(), 1);
                let sa = joint_dofs[0] * joint_axis;
                ISE3q::new(I3::from_slices_scaled_axis(sa.as_slice(), &[0., 0., 0.]))
            }
            ApolloURDFJointType::Fixed => {
                assert_eq!(joint_dofs.len(), 0);
                ISE3q::identity_element()
            }
            ApolloURDFJointType::Floating => {
                assert_eq!(joint_dofs.len(), 6);
                let v6 = V6::from_column_slice(joint_dofs);
                v6.to_lie_alg_ise3q().exp()
            }
            ApolloURDFJointType::Planar => {
                assert_eq!(joint_dofs.len(), 2);
                let t = V3::new(joint_dofs[0], joint_dofs[1], 0.0);
                ISE3q::new(I3::from_slices_scaled_axis(t.as_slice(), &[0., 0., 0.]))
            }
            ApolloURDFJointType::Spherical => {
                assert_eq!(joint_dofs.len(), 3);
                ISE3q::new(I3::from_slices_scaled_axis(&[0.0, 0.0, 0.0], joint_dofs))
            }
        }
    }
}