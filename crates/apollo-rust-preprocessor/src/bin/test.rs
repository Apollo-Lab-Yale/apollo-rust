use std::path::PathBuf;
use std::process::{Command, ExitCode, Stdio, Termination};
use serde::{Deserialize, Serialize};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_file::traits::FromJsonString;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoolWrapper(pub bool);
impl Termination for BoolWrapper {
    fn report(self) -> ExitCode {
        ExitCode::SUCCESS
    }
}

fn main() {
    let fp = PathBuf::new_from_documents_dir().append("apollo-rust/crates/apollo-rust-preprocessor");
    let output = Command::new("cargo")
        .current_dir(fp.to_str().expect("error"))
        .arg("run")
        .arg("--bin")
        .arg("process")
        .arg("--")
        .arg("arg1")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("Failed to execute cargo command");

    if output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout).trim().to_string();
        // println!("{:?}", stdout);

        let s = stdout.split("\n");
        let ss: Vec<String> = s.map(|x| x.to_string()).collect();
        let last = ss.last().expect("error").trim().to_string();

        println!("{:?}", last);
        let res = BoolWrapper::from_json_string(&last);
        println!("{:?}", res);
    }
}