
use nalgebra::DMatrix;
use parry3d_f64::query::{Contact, contact, distance, intersection_test};
use parry3d_f64::shape::Shape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::offset_shape::OffsetShape;

macro_rules! create_double_group_query {
    ($func_name: ident, $output_type: ty, $query_func_code: expr, $push_code: expr, $early_stop_code: expr, $extra_args: ty) => {

        pub fn $func_name(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, skip_symmetrical_entries: bool, skips: Option<&DMatrix<bool>>, early_stop: bool, extra_args: $extra_args) -> Vec<((usize, usize), $output_type)> {
            assert_eq!(group_a.len(), poses_a.len());
            assert_eq!(group_b.len(), poses_b.len());

            let mut out = vec![];

            for (i, (sa, pa)) in group_a.iter().zip(poses_a).enumerate() {
                'l: for (j, (sb, pb)) in group_b.iter().zip(poses_b).enumerate() {
                    if skip_symmetrical_entries { if i >= j { continue 'l; } }
                    if skips.is_some() { if skips.as_ref().unwrap()[(i,j)] { continue 'l; } }

                    let ppa = sa.get_transform(pa);
                    let ppb = sa.get_transform(pb);

                    let res = $query_func_code(&*ppa, &**sa.shape(), &*ppb, &**sb.shape(), &extra_args);
                    if $push_code(&res) {
                        out.push( ((i,j), res) );
                    }

                    if early_stop {
                        if $early_stop_code(&res) { return out; }
                    }

                }
            }

            out
        }
    };
}

create_double_group_query!(
    pairwise_group_query_intersection,
    bool,
    |pose_a: &ISE3q, shape_a: &dyn Shape, pose_b: &ISE3q, shape_b: &dyn Shape, _extra_args: &()|
        {
            intersection_test(&pose_a.0, shape_a, &pose_b.0, shape_b).expect("error")
        },
    |res: &bool|
        {
            return if *res { true } else { false }
        },
    |res: &bool |
        {
            return if *res { true } else { false }
        },
    ()
);

create_double_group_query!(
    pairwise_group_query_distance,
    f64,
    |pose_a: &ISE3q, shape_a: &dyn Shape, pose_b: &ISE3q, shape_b: &dyn Shape, _extra_args: &()|
        {
            distance(&pose_a.0, shape_a, &pose_b.0, shape_b).expect("error")
        },
    |_res: &f64|
        {
            return true;
        },
    |res: &f64 |
        {
            return if *res <= 0.0 { true } else { false }
        },
    ()
);

create_double_group_query!(
    pairwise_group_query_contact,
    Option<Contact>,
    |pose_a: &ISE3q, shape_a: &dyn Shape, pose_b: &ISE3q, shape_b: &dyn Shape, extra_args: &f64|
        {
            contact(&pose_a.0, shape_a, &pose_b.0, shape_b, *extra_args).expect("error")
        },
    |_res: &Option<Contact> |
        {
            return true;
        },
    |res: &Option<Contact> |
        {
            if let Some(c) = res {
                if c.dist <= 0.0 { return true; }
            }
            return false;
        },
    f64
);