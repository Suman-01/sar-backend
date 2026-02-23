use crate::offboard::OffboardController;
use rclrs::*;
use std::{thread, time::Duration};
use std::error::Error;

const ALTITUDE: f64 = 5.0;

pub fn run_mission(
    _node: &Node,
    controller: &OffboardController,
) -> Result<(), Box<dyn Error>> {

    println!("MISSION START");

    // current takeoff position assumed (0,0,-5)
    // move 10m forward (N axis)

    controller.set_target(10.0, 0.0, -ALTITUDE, 0.0);

    // hold for some time so drone reaches
    thread::sleep(Duration::from_secs(8));

    println!("MISSION COMPLETE");
    Ok(())
}
