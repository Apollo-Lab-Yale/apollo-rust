use std::io::Stdout;
use pbr::{ProgressBar, Units};

pub struct ProgressBarWrapper {
    pbr: ProgressBar<Stdout>,
    robot_name: String,
    module_name: String,
}
impl ProgressBarWrapper {
    pub fn new(robot_name: &str, module_name: &str) -> Self {
        Self {
            pbr: get_default_progress_bar(100),
            robot_name: robot_name.to_string(),
            module_name: module_name.to_string(),
        }
    }
    pub fn update(&mut self, message: &str, percent_done: f64) {
        assert!(percent_done >= 0.0 && percent_done <= 100.0);
        let count = percent_done.round() as u64;
        self.pbr.message(message);
        self.pbr.set(count);
    }
    pub fn done(&mut self, message: &str) {
        self.update_preset(100.0);
        self.pbr.finish_println(message);
        println!();
    }
    pub fn update_preset(&mut self, percent_done: f64) {
        let message = format!("Building module.  Robot name: {:?}, Module name: {:?}: ", self.robot_name, self.module_name);
        self.update(&message, percent_done);
    }
    pub fn done_preset(&mut self) {
        let message = format!("Module complete!  Robot name: {:?}, Module name: {:?}", self.robot_name, self.module_name);
        self.done(&message);
    }
}

pub fn get_default_progress_bar(max_total_of_bar: usize) -> ProgressBar<Stdout> {
    let mut out_self = ProgressBar::new(max_total_of_bar as u64);
    out_self.show_counter = false;
    out_self.set_width(Some(200));
    out_self.set_units(Units::Bytes);
    out_self.format(&get_progress_bar_format_string());
    out_self
}
fn get_progress_bar_format_string() -> String {
    // return "".to_string();
    return "╢▌▌░╟".to_string();
    // return "|#--|".to_string();
}