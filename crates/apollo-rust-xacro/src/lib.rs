use apollo_rust_file::ApolloPathBufTrait;
use std::collections::HashMap;

pub mod context;
pub mod expressions;
pub mod parser;

#[derive(Debug)]
pub enum XacroError {
    IoError(std::io::Error),
    XmlError(roxmltree::Error),
    ExpressionError(evalexpr::EvalexprError),
    Custom(String),
}

impl From<std::io::Error> for XacroError {
    fn from(e: std::io::Error) -> Self {
        XacroError::IoError(e)
    }
}

impl From<roxmltree::Error> for XacroError {
    fn from(e: roxmltree::Error) -> Self {
        XacroError::XmlError(e)
    }
}

impl From<evalexpr::EvalexprError> for XacroError {
    fn from(e: evalexpr::EvalexprError) -> Self {
        XacroError::ExpressionError(e)
    }
}

use crate::parser::XacroParser;

use colored::*;

pub fn process_xacro<P: ApolloPathBufTrait + Clone>(
    path: &P,
    search_dirs: &[P],
    arguments: Option<&HashMap<String, String>>,
) -> Result<String, XacroError> {
    if !path.path_exists() {
        return Err(XacroError::Custom(format!("File not found: {:?}", path)));
    }

    println!(
        "{} {} {} ",
        "[".cyan().bold(),
        "Xacro Parser".yellow().bold(),
        format!("] Processing {:?}...", path.to_path_buf())
            .cyan()
            .bold()
    );

    let content = path
        .read_file_contents_to_string_result()
        .map_err(|e| XacroError::IoError(std::io::Error::new(std::io::ErrorKind::NotFound, e)))?;

    let base_path = P::new_from_path(&path.to_path_buf().parent().unwrap().to_path_buf());

    let res = XacroParser::parse(&content, &base_path, search_dirs, arguments);

    if res.is_ok() {
        println!(
            "{} {} {} {}",
            "[".green().bold(),
            "Xacro Parser".yellow().bold(),
            "]".green().bold(),
            "Processing Complete. 🚀".green().bold()
        );
    } else {
        println!(
            "{} {} {} {}",
            "[".red().bold(),
            "Xacro Parser".yellow().bold(),
            "]".red().bold(),
            "Processing Failed. ❌".red().bold()
        );
    }

    res
}

pub fn process_xacro_to_file<P: ApolloPathBufTrait + Clone>(
    path: &P,
    search_dirs: &[P],
    output_path: &P,
    arguments: Option<&HashMap<String, String>>,
) -> Result<(), XacroError> {
    let urdf_string = process_xacro(path, search_dirs, arguments)?;
    output_path.write_string_to_file(&urdf_string);
    println!(
        "{} {} {} {}",
        "[".green().bold(),
        "Xacro Parser".yellow().bold(),
        "]".green().bold(),
        format!("File Saved to {:?}. 💾", output_path.to_path_buf())
            .green()
            .bold()
    );
    Ok(())
}
