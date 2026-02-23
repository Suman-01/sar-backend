mod offboard;
mod arm;
mod take_off;
mod mission;

use rclrs::*;
use std::sync::Arc;
use std::thread;

use offboard::OffboardController;
use arm::ArmDrone;
use take_off::TakeOff;
use mission::run_mission;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    // =====================
    // ROS SETUP
    // =====================
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let controller = Arc::new(OffboardController::new(&executor)?);
    let node = executor.create_node("mission_controller")?;

    let arm = ArmDrone::new(&node)?;

    // =====================
    // OFFBOARD THREAD
    // =====================
    let ctrl_clone = controller.clone();
    thread::spawn(move || {
        ctrl_clone.spin_loop();
    });

    // =====================
    // MISSION THREAD
    // =====================
    let ctrl_m = controller.clone();
    let node_m = node.clone();

    thread::spawn(move || {

        thread::sleep(std::time::Duration::from_secs(1));

        arm.set_offboard();
        arm.arm();

        // TAKEOFF
        TakeOff::takeoff(&ctrl_m);

        // MISSION
        run_mission(&node_m, &ctrl_m).ok();
    });

    // =====================
    // EXECUTOR (MAIN THREAD)
    // =====================
    executor
        .spin(SpinOptions::default())
        .first_error()?;

    Ok(())
}
