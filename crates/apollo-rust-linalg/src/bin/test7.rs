use std::mem::ManuallyDrop;
use std::time::Instant;

#[derive(Clone)]
pub enum Op {
    None, Add
}

#[derive(Clone)]
#[allow(non_camel_case_types)]
pub struct adrr {
    pub value: f64,
    pub op: Op,
    pub parent_a: Option<*const ManuallyDrop<adrr>>,
    pub parent_b: Option<*const ManuallyDrop<adrr>>
}

fn test() -> *const i32 {
    let a = Box::new(5);
    return Box::into_raw(a)
}

fn main() {
    let a = ManuallyDrop::new(adrr {
        value: 1.0,
        op: Op::None,
        parent_a: None,
        parent_b: None,
    });
    let b = ManuallyDrop::new(adrr {
        value: 2.0,
        op: Op::None,
        parent_a: None,
        parent_b: None,
    });

    let start = Instant::now();
    for _ in 0..10000 {
        let res = ManuallyDrop::new(
            adrr {
                value: a.value / b.value,
                op: Op::Add,
                parent_a: Some(&a as *const ManuallyDrop<adrr>),
                parent_b: Some(&b as *const ManuallyDrop<adrr>),
            }
        );
        std::hint::black_box(res);
    }
    let d1 = start.elapsed();
    println!("{:?}", d1);

    let a = 1.0;
    let b = 2.0;
    let start = Instant::now();
    for _ in 0..10000 {
        let res = a / b;
        std::hint::black_box(res);
    }
    let d2 = start.elapsed();
    println!("{:?}", d2);
    println!("{:?}", d1.as_secs_f64()/d2.as_secs_f64());

    let res = test();
    let a = unsafe { *res };
    println!("{:?}", a);
}


