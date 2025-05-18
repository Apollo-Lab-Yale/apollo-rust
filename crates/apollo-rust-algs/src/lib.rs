/// Generate the power set of a given vector. The power set is the set of all subsets of the vector.
///
/// # Arguments
/// * `vec` - A vector of elements for which the power set is to be generated.
///
/// # Returns
/// A vector containing all subsets of the input vector.
pub fn power_set<T: Clone>(vec: Vec<T>) -> Vec<Vec<T>> {
    let n = vec.len();
    let mut result = Vec::new();

    // Iterate over all possible combinations of elements (2^n combinations).
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

/// Generate all combinations of `n` elements from a given vector.
///
/// # Arguments
/// * `vec` - A vector of elements to choose combinations from.
/// * `n` - The number of elements in each combination.
///
/// # Returns
/// A vector containing all combinations of `n` elements from the input vector.
///
/// # Panics
/// Will return an empty vector if `n` is greater than the length of the input vector.
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

/// Trait to convert a vector of `Option<T>` into a vector of vectors of `T`.
///
/// # Example
/// Given a vector of `Option<T>`, the conversion transforms `Some(T)` into a vector containing `T`
/// and `None` into an empty vector.
///
/// # Returns
/// A vector of vectors, where each `Option<T>` is either an empty vector (`None`) or a vector with one element (`Some(T)`).
pub trait VecOfOptionsToVecOfVecsTrait<T> where T: Clone {
    /// Converts a vector of `Option<T>` into a vector of vectors of `T`.
    fn to_vec_of_vecs(&self) -> Vec<Vec<T>>;
}

/// Implementation of `VecOfOptionsToVecOfVecsTrait` for vectors of `Option<T>`.
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
