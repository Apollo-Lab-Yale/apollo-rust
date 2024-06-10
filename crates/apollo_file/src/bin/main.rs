use std::path::PathBuf;
use apollo_file::ApolloPathBufTrait;

fn main() {
    let test = "hello".to_string();

    let f = PathBuf::new_from_documents_dir().append("hello.yaml");

    f.save_object_to_yaml_file(&test);
    let tt = f.load_object_from_yaml_file::<String>();
    println!("{:?}", tt);
}
