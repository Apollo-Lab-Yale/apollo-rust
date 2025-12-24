use apollo_rust_algs::{combinations_of_n, power_set, VecOfOptionsToVecOfVecsTrait};

#[test]
fn test_power_set() {
    let input = vec![1, 2, 3];
    let result = power_set(input);

    // Power set of {1, 2, 3} has 2^3 = 8 elements.
    assert_eq!(result.len(), 8);

    // Check specific subsets exist (implementation preserves input order)
    assert!(result.contains(&vec![]));
    assert!(result.contains(&vec![1]));
    assert!(result.contains(&vec![2]));
    assert!(result.contains(&vec![3]));
    assert!(result.contains(&vec![1, 2]));
    assert!(result.contains(&vec![1, 3]));
    assert!(result.contains(&vec![2, 3]));
    assert!(result.contains(&vec![1, 2, 3]));
}

#[test]
fn test_combinations_of_n() {
    let input = vec![1, 2, 3, 4];
    let n = 2;
    let result = combinations_of_n(input, n);

    // C(4, 2) = 6
    assert_eq!(result.len(), 6);

    assert!(result.contains(&vec![1, 2]));
    assert!(result.contains(&vec![1, 3]));
    assert!(result.contains(&vec![1, 4]));
    assert!(result.contains(&vec![2, 3]));
    assert!(result.contains(&vec![2, 4]));
    assert!(result.contains(&vec![3, 4]));
}

#[test]
fn test_vec_of_options_to_vec_of_vecs() {
    let input: Vec<Option<i32>> = vec![Some(1), None, Some(3), None];
    let result = input.to_vec_of_vecs();

    assert_eq!(result.len(), 4);
    assert_eq!(result[0], vec![1]);
    assert_eq!(result[1], Vec::<i32>::new());
    assert_eq!(result[2], vec![3]);
    assert_eq!(result[3], Vec::<i32>::new());
}
