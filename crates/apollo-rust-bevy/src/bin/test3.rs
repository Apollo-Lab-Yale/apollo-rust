use bevy::app::{App};
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let mut app = App::new()
        .apollo_bevy_robotics_base(false)
        .apollo_bevy_spawn_transform_gizmos(vec![None, None, Some(ISE3q::identity()), None]);

    /*
    app.add_systems(Update, |mut a: ResMut<TransformGizmoEngine>, input: Res<ButtonInput<KeyCode>>| {
        println!("{:?}", a.gizmo_transforms);
        if input.just_pressed(KeyCode::Space) {
            a.insert_new_transform_gizmo(None);
        }

        if input.just_pressed(KeyCode::Enter) {
            a.remove_transform_gizmo(0);
        }

        if input.just_pressed(KeyCode::KeyZ) {
            a.set_transform(0, ISE3q::identity());
        }
    });
    */

    /*
    app.add_systems(Startup, | mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
        commands.spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(Cuboid { half_size: Vec3::new(0.02, 0.02, 0.02) })),
            material: materials.add(Color::srgb(1.0, 1.0, 0.0)),
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            ..default()
        }).insert(GizmoTarget::default());

        commands.spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(Cuboid { half_size: Vec3::new(0.02, 0.02, 0.02) })),
            material: materials.add(Color::srgb(1.0, 1.0, 0.0)),
            transform: Transform::from_xyz(1.0, 0.0, 0.0),
            ..default()
        }).insert(GizmoTarget::default());
     });
     */

    /*
    app.add_systems(Update, |mut commands: Commands, mut query: Query<(Entity)>, input: Res<ButtonInput<KeyCode>>| {
        if input.just_pressed(KeyCode::Space) {
            for (e) in query.iter() {

            }
        }
    });
    */

    app.run();
}