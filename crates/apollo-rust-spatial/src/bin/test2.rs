use apollo_rust_spatial::quaternions::UQ;

fn main() {
    let q = UQ::from_euler_angles(1.9,2.,-1.5);
    println!("{:?}", q.w);
    println!("{:?}", q.i);
    println!("{:?}", q.j);
    println!("{:?}", q.k);
}