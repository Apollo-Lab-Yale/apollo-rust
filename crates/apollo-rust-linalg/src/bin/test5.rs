use std::sync::Arc;
use std::time::Instant;

#[derive(Clone)]
pub struct SingleParentOp {
    pub parent: Arc<rad_rc>
}

#[derive(Clone, Debug)]
pub struct SingleParentOp2 {
    pub parent: *const rad_rc2
}

#[derive(Clone)]
pub struct DoubleParentOp {
    pub parent_a: Arc<rad_rc>,
    pub parent_b: Arc<rad_rc>
}

#[derive(Clone, Debug)]
pub struct DoubleParentOp2 {
    pub parent_a: *const rad_rc2,
    pub parent_b: *const rad_rc2
}

#[derive(Clone)]
pub enum Op {
    None, Add(DoubleParentOp), Sub(DoubleParentOp), Sin(SingleParentOp)
}

#[derive(Clone, Debug)]
pub enum Op2 {
    None, Add(DoubleParentOp2), Sub(DoubleParentOp2), Sin(SingleParentOp2)
}

#[allow(non_camel_case_types)]
pub struct rad_rc {
    pub op: Op
}
impl Default for rad_rc {
    fn default() -> Self {
        rad_rc {
            op: Op::None,
        }
    }
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct rad_rc2 {
    pub op: Op2
}
impl Default for rad_rc2 {
    fn default() -> Self {
        rad_rc2 {
            op: Op2::None,
        }
    }
}


fn main() {
    let a = rad_rc2::default();
    let b = rad_rc2::default();
    let c = rad_rc2 {
        op: Op2::Add(DoubleParentOp2 {
            parent_a: &a as *const rad_rc2,
            parent_b: &b as *const rad_rc2,
        }),
    };

    // let x = 42;
    // let ptr: *const i32 = &x as *const i32;

    let start = Instant::now();
    for _ in 0..1000 {
        let res = c.clone();
        std::hint::black_box(res);
    }
    println!("{:?}", start.elapsed());

    let aa = c.clone();
    println!("{:?}", aa);
}