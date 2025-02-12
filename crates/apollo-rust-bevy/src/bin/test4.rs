use apollo_rust_bevy::apollo_bevy_utils::transform::TransformUtils;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let t = ISE3q::new_random();
    println!("{:?}", t);
    let tt = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&t);
    println!("{:?}", tt);
    println!("{:?}", TransformUtils::util_convert_y_up_bevy_transform_to_pose(&tt));
}