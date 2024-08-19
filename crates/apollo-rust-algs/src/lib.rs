

pub fn power_set<T: Clone>(vec: Vec<T>) -> Vec<Vec<T>> {
    let n = vec.len();
    let mut result = Vec::new();

    for i in 0..(1 << n) {
        let mut subset = Vec::new();
        for j in 0..n {
            if i & (1 << j) != 0 {
                subset.push(vec[j].clone());
            }
        }
        result.push(subset);
    }

    result
}



pub trait VecOfOptionsToVecOfVecsTrait<T> where T: Clone {
    fn to_vec_of_vecs(&self) -> Vec<Vec<T>>;
}
impl<T> VecOfOptionsToVecOfVecsTrait<T> for Vec<Option<T>> where T: Clone {
    fn to_vec_of_vecs(&self) -> Vec<Vec<T>> {
        self.iter().map(|x| {
            match x {
                None => { vec![] }
                Some(x) => { vec![x.clone()] }
            }
        }).collect()
    }
}