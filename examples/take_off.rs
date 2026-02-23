use crate::offboard::OffboardController;
use std::{thread, time::Duration};

const TAKEOFF_ALT: f64 = 5.0;

pub struct TakeOff;

impl TakeOff {
    pub fn takeoff(controller: &OffboardController) {

        println!("TAKEOFF START");

        // command climb
        controller.set_target(0.0, 0.0, -TAKEOFF_ALT, 0.0);

        // wait for climb (simple + reliable)
        thread::sleep(Duration::from_secs(6));

        println!("TAKEOFF COMPLETE");
    }
}
