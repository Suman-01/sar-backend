// orchestrator.rs

mod offboard;
mod arm;
mod take_off;
mod mission;

use rclrs::*;
use std::sync::Arc;
use std::thread;

use offboard::OffboardController;
use arm::ArmDrone;
use mission::run_mission;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    // create offboard controllers for both namespaces
    let controller1 = Arc::new(OffboardController::new(&executor, "px4_1")?);
    let controller2 = Arc::new(OffboardController::new(&executor, "px4_2")?);

    // single node for mission/arming topics
    let node = executor.create_node("mission_controller")?;

    let arm1 = Arc::new(ArmDrone::new(&node, "px4_1", 2)?);
    let arm2 = Arc::new(ArmDrone::new(&node, "px4_2", 3)?);


    {
        let c = controller1.clone();
        thread::spawn(move || {
            c.spin_loop();
        });
    }
    {
        let c = controller2.clone();
        thread::spawn(move || {
            c.spin_loop();
        });
    }


    let ctrl1 = controller1.clone();
    let ctrl2 = controller2.clone();
    let node_m = node.clone();
    let arm1_m = arm1.clone();
    let arm2_m = arm2.clone();

    thread::spawn(move || {
        // small delay so everything is up
        thread::sleep(std::time::Duration::from_secs(1));

        // now run the coordinated mission in mission.rs
        if let Err(e) = run_mission(&node_m, ctrl1, ctrl2, arm1_m, arm2_m) {
            eprintln!("Mission error: {:?}", e);
        }
    });

    // =====================
    // EXECUTOR (MAIN THREAD)
    // =====================
    executor
        .spin(SpinOptions::default())
        .first_error()?;

    Ok(())
}
