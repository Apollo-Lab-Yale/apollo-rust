#[cfg(not(target_arch = "wasm32"))]
use colored::*;
#[cfg(not(target_arch = "wasm32"))]
use pbr::{ProgressBar, Units};
use std::io::Stdout;

pub struct ProgressBarWrapper {
    #[cfg(not(target_arch = "wasm32"))]
    pbr: ProgressBar<Stdout>,
    name: String,
    module_name: String,
    maximum: Option<usize>,
    curr_count: usize,
}
impl ProgressBarWrapper {
    pub fn new(name: &str, module_name: &str) -> Self {
        Self {
            #[cfg(not(target_arch = "wasm32"))]
            pbr: get_default_progress_bar(100),
            name: name.to_string(),
            module_name: module_name.to_string(),
            maximum: None,
            curr_count: 0,
        }
    }
    pub fn set_max_increment(&mut self, max: usize) {
        self.maximum = Some(max);
    }
    pub fn increment(&mut self) {
        assert!(self.maximum.is_some(), "max increment must be set");
        self.curr_count += 1;

        let max = self.maximum.clone().unwrap() as f64;
        let curr = self.curr_count.clone() as f64;

        let ratio = curr / max;
        let percent = ratio * 100.0;

        self.update_with_percentage_preset(percent);
    }
    pub fn update_with_percentage(&mut self, message: &str, percent_done: f64) {
        assert!(percent_done >= 0.0 && percent_done <= 100.0);
        let count = percent_done.round() as u64;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.pbr.message(message);
            self.pbr.set(count);
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _ = message;
            let _ = count;
        }
    }
    pub fn done(&mut self, message: &str) {
        self.update_with_percentage_preset(100.0);
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.pbr.finish_println(message);
            println!();
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _ = message;
        }
    }
    /// percent done is out of 100.0
    /// percent done is out of 100.0
    pub fn update_with_percentage_preset(&mut self, percent_done: f64) {
        #[cfg(not(target_arch = "wasm32"))]
        let message = format!(
            "{} {} {} ",
            "[".cyan().bold(),
            self.name.yellow().bold(),
            format!("] {}...", self.module_name).cyan().bold()
        );
        #[cfg(target_arch = "wasm32")]
        let message = format!("[{}] {}... ", self.name, self.module_name);

        self.update_with_percentage(&message, percent_done);
    }
    pub fn done_preset(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        let message = format!(
            "{} {} {} {}",
            "[".green().bold(),
            self.name.yellow().bold(),
            "]".green().bold(),
            format!("{} Complete. 🚀", self.module_name).green().bold()
        );
        #[cfg(target_arch = "wasm32")]
        let message = format!("[{}] {} Complete.", self.name, self.module_name);

        self.done(&message);
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn get_default_progress_bar(max_total_of_bar: usize) -> ProgressBar<Stdout> {
    let mut out_self = ProgressBar::new(max_total_of_bar as u64);
    out_self.show_counter = false;
    out_self.show_speed = false;
    out_self.show_percent = true;
    out_self.show_time_left = false;
    out_self.set_width(Some(40));
    out_self.set_units(Units::Bytes);
    out_self.format(&get_progress_bar_format_string());
    out_self
}
#[cfg(not(target_arch = "wasm32"))]
fn get_progress_bar_format_string() -> String {
    // return "".to_string();
    return "╢▌▌░╟".to_string();
    // return "|#--|".to_string();
}
