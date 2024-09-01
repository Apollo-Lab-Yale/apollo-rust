

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

pub fn combinations_of_n<T: Clone>(vec: Vec<T>, n: usize) -> Vec<Vec<T>> {
    let mut result = Vec::new();
    let len = vec.len();

    if n > len {
        return result;
    }

    let mut indices = (0..n).collect::<Vec<_>>();

    loop {
        result.push(indices.iter().map(|&i| vec[i].clone()).collect());

        let mut i = n;
        while i > 0 {
            i -= 1;
            if indices[i] != i + len - n {
                break;
            }
        }

        if i == 0 && indices[i] == len - n {
            break;
        }

        indices[i] += 1;
        for j in i+1..n {
            indices[j] = indices[j-1] + 1;
        }
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