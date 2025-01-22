use bevy::prelude::{Assets, Color, Commands, Component, Cuboid, default, Mesh, PbrBundle, Query, Res, ResMut, Resource, StandardMaterial, Transform, Vec3};
use transform_gizmo_bevy::GizmoTarget;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::transform::TransformUtils;

pub struct TransformGizmoSystems;
impl TransformGizmoSystems {
    pub fn system_transform_gizmos_set_transforms_for_get(mut tge: ResMut<TransformGizmoEngine>, query: Query<(&Transform, &TransformGizmoMarker)>) {
        for (t, tgm) in query.iter() {
            let t: &Transform = t;
            let tgm: &TransformGizmoMarker = tgm;

            tge.gizmo_transforms[tgm.0] = TransformUtils::util_convert_y_up_bevy_transform_to_pose(t);
        }
    }

    pub fn system_transform_gizmos_set_transforms(mut tge: ResMut<TransformGizmoEngine>, mut query: Query<(&mut Transform, &TransformGizmoMarker)>) {
        tge.new_set_transform_idxs.iter().for_each(|x| {
            for (mut t, tgm) in query.iter_mut() {
                let t: &mut Transform = &mut t;
                let tgm: &TransformGizmoMarker = tgm;

                if tgm.0 == *x {
                    *t = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&tge.gizmo_transforms[*x]);
                }
            }
        });

        tge.new_set_transform_idxs.clear();
    }

    pub fn system_transform_gizmos_insert(mut tge: ResMut<TransformGizmoEngine>, mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
        for insert_idx in tge.new_inserts.iter() {
            commands.spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(Cuboid { half_size: Vec3::new(0.02, 0.02, 0.02) })),
                material: materials.add(Color::srgb(1.0, 1.0, 0.0)),
                transform: TransformUtils::util_convert_pose_to_y_up_bevy_transform(&tge.gizmo_transforms[*insert_idx]),
                ..default()
            })
                .insert(GizmoTarget::default())
                .insert(TransformGizmoMarker(*insert_idx));
        }

        tge.new_inserts.clear();
    }
}

#[derive(Resource, Clone)]
pub struct TransformGizmoEngine {
    pub gizmo_idxs_in_use: Vec<bool>,
    pub gizmo_transforms: Vec<ISE3q>,
    pub new_inserts: Vec<usize>,
    pub new_deletes: Vec<usize>,
    pub new_set_transform_idxs: Vec<usize>
}
impl TransformGizmoEngine {
    pub fn new() -> Self {
        Self {
            gizmo_idxs_in_use: vec![],
            gizmo_transforms: vec![],
            new_inserts: vec![],
            new_deletes: vec![],
            new_set_transform_idxs: vec![],
        }
    }

    pub fn get_transform(&self, idx: usize) -> ISE3q {
        if self.gizmo_idxs_in_use[idx] == false { panic!("not valid") }
        return self.gizmo_transforms[idx].clone()
    }

    pub fn set_transform(&mut self, idx: usize, transform: ISE3q) {
        self.new_set_transform_idxs.push(idx);
        self.gizmo_transforms[idx] = transform;
    }

    pub fn remove_transform_gizmo(&mut self, idx: usize) {
        self.gizmo_idxs_in_use[idx] = false;
        self.new_deletes.push(idx);
    }

    pub fn insert_new_transform_gizmo(&mut self, init_transform: Option<ISE3q>) -> usize {
        let mut new_idx = self.gizmo_idxs_in_use.len();
        let mut already_existing_idx = false;
        'l: for (idx, in_use) in self.gizmo_idxs_in_use.iter().enumerate() {
            if *in_use == false {
                new_idx = idx;
                self.gizmo_idxs_in_use[idx] = true;
                already_existing_idx = true;
                break 'l;
            }
        }
        if !already_existing_idx {
            self.gizmo_idxs_in_use.push(true);
        }

        self.new_inserts.push(new_idx);
        let t = match init_transform {
            None => { ISE3q::new(I3::new_random_with_range(-1.0, 1.0)) }
            Some(t) => { t }
        };
        if already_existing_idx {
            self.gizmo_transforms[new_idx] = t;
        } else {
            self.gizmo_transforms.push(t);
        }

        return new_idx;
    }
}

#[derive(Component, Clone, Debug)]
pub struct TransformGizmoMarker(pub usize);