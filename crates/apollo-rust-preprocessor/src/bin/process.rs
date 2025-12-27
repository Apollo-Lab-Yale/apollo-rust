use apollo_rust_file::traits::ToJsonString;
use serde::{Deserialize, Serialize};
use std::env;
use std::process::{ExitCode, Termination};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoolWrapper(pub bool);
impl Termination for BoolWrapper {
    fn report(self) -> ExitCode {
        ExitCode::SUCCESS
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    println!("{:?}", args);

    // let command = &args[1];
    // let command_args = &args[2..];

    let result = BoolWrapper(false);
    println!("woah");
    println!("{}", result.to_json_string());
}
