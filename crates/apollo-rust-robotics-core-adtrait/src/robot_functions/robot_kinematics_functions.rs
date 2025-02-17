use ad_trait::AD;
use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use apollo_rust_linalg_adtrait::{ApolloDVectorTrait, V};
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFJointType;
use apollo_rust_spatial_adtrait::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::{ApolloLieAlgPackIse3qTrait, ISE3q};
use apollo_rust_spatial_adtrait::vectors::{V3, V6};
use crate::modules_runtime::urdf_nalgebra_module::{ApolloURDFAxisNalgebra, ApolloURDFNalgebraModule};

pub struct RobotKinematicsFunctions;
impl RobotKinematicsFunctions {
    #[inline]
    pub fn fk<A: AD>(state: &V<A>, urdf_module: &ApolloURDFNalgebraModule<A>, chain_module: &ApolloChainModule, dof_module: &ApolloDOFModule) -> Vec<ISE3q<A>> {
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

                let joint_dofs = match &parent_joint.mimic {
                    None => {
                        let dof_idxs = &joint_idx_to_dofs_mapping[parent_joint_idx];
                        let joint_dofs: Vec<A> = dof_idxs.iter().map(|i| state[*i]).collect();
                        joint_dofs
                    }
                    Some(mimic) => {
                        let mimic_joint_name = mimic.joint.clone();
                        let mimic_joint_in_chain = chain_module.joints_in_chain.iter().find(|x| x.joint_name == mimic_joint_name).expect(&format!("{} could not be found", mimic_joint_name));
                        let dof_idxs = &joint_idx_to_dofs_mapping[mimic_joint_in_chain.joint_idx];
                        let joint_dofs: Vec<A> = dof_idxs.iter().map(|i| mimic.offset.unwrap_or(0.0).to_other_ad_type::<A>() + mimic.multiplier.unwrap_or(1.0).to_other_ad_type::<A>() * state[*i]).collect();
                        joint_dofs
                    }
                };

                let joint_axis = &parent_joint.axis;
                let joint_type = &parent_joint.joint_type;
                let variable_transform = Self::get_joint_variable_transform_urdf_axis(joint_type, joint_axis, &joint_dofs);

                out[*link_idx] = out[parent_link_idx].group_operator(constant_transform).group_operator(&variable_transform);
            });
        }

        out
    }

    /// Computes the reverse of forward kinematics, given the link frames.
    ///
    /// # Arguments
    /// - `link_frames`: A reference to a vector of `ISE3q` representing the link frames.
    /// - `urdf_module`: A reference to the URDF module that holds the robot's URDF structure.
    /// - `chain_module`: A reference to the chain module containing the kinematic chain.
    /// - `dof_module`: A reference to the DOF (Degrees of Freedom) module.
    ///
    /// # Returns
    /// A `V` representing the state that would result in the given link poses.
    pub fn reverse_of_fk<A: AD>(link_frames: &Vec<ISE3q<A>>, urdf_module: &ApolloURDFNalgebraModule<A>, chain_module: &ApolloChainModule, dof_module: &ApolloDOFModule) -> V<A> {
        let mut out = V::new(&vec![0.0; dof_module.num_dofs]).to_other_ad_type::<A>();

        chain_module.joints_in_chain.iter().for_each(|joint_in_chain| {
            let joint_idx = joint_in_chain.joint_idx;
            let parent_link_idx = joint_in_chain.parent_link_idx;
            let child_link_idx = joint_in_chain.child_link_idx;
            let joint = &urdf_module.joints[joint_idx];
            let constant_transform = &joint.origin.ise3q;
            let joint_type = &joint.joint_type;
            let dof_idxs = &dof_module.joint_idx_to_dof_idxs_mapping[joint_idx];
            let axis = &joint.axis.axis;

            let t_variable = constant_transform.inverse().group_operator(&link_frames[parent_link_idx].inverse()).group_operator(&link_frames[child_link_idx]);
            let t_variable_vee = t_variable.ln().vee();

            match joint_type {
                ApolloURDFJointType::Revolute => {
                    let mut value = t_variable_vee.norm() * A::constant(2.0);
                    let tmp = V3::new(t_variable_vee[0], t_variable_vee[1], t_variable_vee[2]);
                    let d = tmp.dot(axis);
                    if d < A::zero() { value *= A::constant(-1.0) }
                    out[dof_idxs[0]] = value;
                }
                ApolloURDFJointType::Continuous => {
                    let mut value = t_variable_vee.norm() * A::constant(2.0);
                    let tmp = V3::new(t_variable_vee[0], t_variable_vee[1], t_variable_vee[2]);
                    let d = tmp.dot(axis);
                    if d < A::zero() { value *= A::constant(-1.0); }
                    out[dof_idxs[0]] = value;
                }
                ApolloURDFJointType::Prismatic => {
                    let mut value = t_variable_vee.norm();
                    let tmp = V3::new(t_variable_vee[3], t_variable_vee[4], t_variable_vee[5]);
                    let d = tmp.dot(axis);
                    if d < A::constant(0.0) { value *= A::constant(-1.0) }
                    out[dof_idxs[0]] = value;
                }
                ApolloURDFJointType::Fixed => { }
                ApolloURDFJointType::Floating => {
                    dof_idxs.iter().enumerate().for_each(|(i, x)| out[*x] = t_variable_vee[i]);
                }
                ApolloURDFJointType::Planar => {
                    out[dof_idxs[0]] = t_variable_vee[4];
                    out[dof_idxs[1]] = t_variable_vee[5];
                }
                ApolloURDFJointType::Spherical => {
                    out[dof_idxs[0]] = t_variable_vee[0];
                    out[dof_idxs[1]] = t_variable_vee[1];
                    out[dof_idxs[2]] = t_variable_vee[2];
                }
            }
        });

        out
    }

    /// Retrieves the joint's variable transform from its URDF axis.
    ///
    /// # Arguments
    /// - `joint_type`: A reference to the joint type.
    /// - `joint_axis`: A reference to the URDF axis.
    /// - `joint_dofs`: A slice of joint DOFs (Degrees of Freedom).
    ///
    /// # Returns
    /// An `ISE3q` representing the joint's variable transform.
    #[inline(always)]
    fn get_joint_variable_transform_urdf_axis<A: AD>(joint_type: &ApolloURDFJointType, joint_axis: &ApolloURDFAxisNalgebra<A>, joint_dofs: &[A]) -> ISE3q<A> {
        Self::get_joint_variable_transform(joint_type, &joint_axis.axis, joint_dofs)
    }

    /// Retrieves the joint's variable transform.
    ///
    /// # Arguments
    /// - `joint_type`: A reference to the joint type.
    /// - `joint_axis`: A reference to the joint axis.
    /// - `joint_dofs`: A slice of joint DOFs (Degrees of Freedom).
    ///
    /// # Returns
    /// An `ISE3q` representing the joint's variable transform.
    #[inline(always)]
    pub fn get_joint_variable_transform<A: AD>(joint_type: &ApolloURDFJointType, joint_axis: &V3<A>, joint_dofs: &[A]) -> ISE3q<A> {
        return match joint_type {
            ApolloURDFJointType::Revolute => {
                assert_eq!(joint_dofs.len(), 1);
                let sa = joint_axis * joint_dofs[0];
                ISE3q::new(I3::from_slices_scaled_axis(&[A::zero(), A::zero(), A::zero()], sa.as_slice()))
            }
            ApolloURDFJointType::Continuous => {
                assert_eq!(joint_dofs.len(), 1);
                let sa = joint_axis * joint_dofs[0];
                ISE3q::new(I3::from_slices_scaled_axis(&[A::zero(), A::zero(), A::zero()], sa.as_slice()))
            }
            ApolloURDFJointType::Prismatic => {
                assert_eq!(joint_dofs.len(), 1);
                let sa = joint_axis * joint_dofs[0];
                ISE3q::new(I3::from_slices_scaled_axis(sa.as_slice(), &[A::zero(), A::zero(), A::zero()]))
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
                let t = V3::new(joint_dofs[0], joint_dofs[1], A::zero());
                ISE3q::new(I3::from_slices_scaled_axis(t.as_slice(), &[A::zero(), A::zero(), A::zero()]).to_other_ad_type::<A>())
            }
            ApolloURDFJointType::Spherical => {
                assert_eq!(joint_dofs.len(), 3);
                ISE3q::new(I3::from_slices_scaled_axis(&[A::zero(), A::zero(), A::zero()], joint_dofs))
            }
        }
    }
}