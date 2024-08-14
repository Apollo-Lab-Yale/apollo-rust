

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