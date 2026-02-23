// mod offboard;
// mod arm;
// mod take_off;

// use rclrs::*;
// use offboard::OffboardController;
// use arm::ArmDrone;
// use take_off::TakeOff;

// fn main() -> Result<(), Box<dyn std::error::Error>> {
//     let context = Context::default_from_env()?;
//     let mut executor = context.create_basic_executor();

//     let controller = OffboardController::new(&executor)?;
//     let node = executor.create_node("mission_controller")?;

//     let arm = ArmDrone::new(&node)?;
//     let takeoff = TakeOff::new(&node)?;

//     std::thread::spawn(move || {
//         controller.spin_loop();
//     });

//     std::thread::sleep(std::time::Duration::from_secs(1));

//     arm.set_offboard();
//     arm.arm();

//     std::thread::sleep(std::time::Duration::from_secs(1));

//     takeoff.takeoff();

//     executor.spin(SpinOptions::default()).first_error()?;
//     Ok(())
// }

//state machine based approach next

mod arm;
mod take_off;
mod offboard;
mod mission;
mod tsdf_listner;

use rclrs::*;
use std::error::Error;
use std::task::Context;
use std::thread;
use  std::time::Duration;

use arm::ArmDrone;
use take_off::TakeOff;
use offboard::OffboardController;

fn main() -> Result<(), Box<dyn Error>> {
    let context = Context::default_from_env()?;
    let mut  executor = context.create_basic_executor();

    let node = executor.create_node("mission_orchestrator")?;
    
    let offboard = OffboardController::new(&executor)?;
    let tsdf = tsdf_listner::TsdfListner::new(&node)?;

    let off_clone = offboard.clone();
    thread::spawn(move || {
        off_clone.spin_loop();
    });

    //arming
    let arm_dr = ArmDrone::new(&node);
    thread::sleep(Duration::from_millis(millis(500)));

    println!("OFFBOARD MODE ENABLED");
    arm_dr.set_offboard();

    println!("ARMING...");
    arm_dr.arm();

    thread::sleep(Duration::from_secs(2));

    //take_off
    let take_off_dr = TakeOff::new(&node);
    println!("TAKE OFF INITIATED...");

    take_off_dr.takeoff();

    thread::sleep(Duration::from_secs(5));

    //mission
    println!("STARTING MISSION...");
    mission::run_mission(&node)?;
    println!("Mission Finished");

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())

}


