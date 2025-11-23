use std::process::{Command, Child};
use std::thread::sleep;
use std::time::Duration;
use std::io::{self, Stdout, Write};


//Start the uXRCE agent
fn start_dds_agent(cmd: &str) -> Result<Child, String> {

    Command::new("bash")
            .arg("-lc")
            .arg(cmd)
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .spawn()
            .map_err(|e| format!("failed to spawn uXRCE agent '{}': {}", cmd, e))
}

fn ros2_call(cmd: &str) -> Result<(String, String), String> {

    let out = Command::new("bash")
                                    .arg("-lc")
                                    .arg(cmd)
                                    .output()
                                    .map_err(|e| format!("failed to execute the ros2 service '{}': {}", cmd, e))?;

    let out_str = String::from_utf8_lossy(&out.stdout).to_string();
    let err_str = String::from_utf8_lossy(&out.stderr).to_string();

    if out.status.success() {
        Ok((out_str, err_str))
    } else {
        Err(format!(
            "Failed to call ros2 service (Status: {}), stdout: \n{}\nstderr:\n{}",
            out.status, out_str, err_str
        ))
    }
} 

fn main() -> Result<(), String> {
    let agent_binary = "home/admin1/prj/uXRCE/install/microxrcedds_agent/bin/MicroXRCEAgent";
    let agent_cmd = format!("{} udp4 -p 8888", agent_binary);

    let vehicles: Vec<u32> = vec![1, 2];

    let takeoff_alt_met: f32 = 3.0;

    let ros2_src = "source /opt/ros/humble/setup.bash";

    let px4_service = "/fmu/vehicle_command";

    let agent_start_wait: u32 = 3; //sec
    let vehicle_call_wait = 2; //sec

    println!("Starting MicroXRCEAgent with: {}", agent_cmd);

    let mut agent = start_dds_agent(&agent_cmd);

    println!("Agent pid = {}. Waiting {}s fo rinit...", agent.id(), agent_start_wait);

    sleep(Duration::from_secs(agent_start_wait));

    for &sys in &vehicles {

        println!("=== Vehicle = {} ===", sys);

        let arm = format!(
            "{} && ros2 service call {} px4_msgs/srv/VehicleCommand \"{{command: 400, param1: 1.0, target_system: {}, target_component: 1}}\" -r",
            ros2_src, px4_service, sys
        );

        print!("Sending ARM command to drone {} ...", sys);
        io::stdout().flush().ok();

        match ros2_call(&arm_call) {
            Ok((stdout, stderr)) => {
                println!("OK");

                if !stdout.trim().is_empty() {
                    println!("-> service stdout:\n{}", stdout);
                }

                if !stderr.trim().is_empty() {
                    println!("-> service stderr:\n{}", stderr);
                }
            }

            Err(e) => {
                println!("\nARM service call error: {}\nProceeding to next vehicle.", e);
            }
        }

        sleep(Duration::from_secs(vehicle_call_wait));

        let takeoff = format!(
            "{} && ros2 service call {} px4_msgs/srv/VehicleCommand \"{{command: 22, param7: {:.2}.0, target_system: {}, target_component: 1}}\" -r",
            ros2_src, px4_service, takeoff_alt_met, sys
        );

        print!("Sending TAKEOFF command to drone {} ...", sys);

        io::stdout().flush().ok();

        match ros2_call(&takeoff) {
            
            Ok((stdout, stderr)) => {
                println!("OK");

                if !stdout.trim().is_empty() {
                    println!("-> service stdout:\n{}", stdout);
                }

                if !stderr.trim().is_empty() {
                    println!("-> service stderr:\n{}", stderr);
                }
            }

            Err(e) => {
                println!("\nTAKEOFF service call error: {}\nProceeding to next vehicle.", e);
            }
        }

        println!("Wait 6s for drone {} to TAKEOFF ... ", sys);

        sleep(Duration::from_secs(6));

    }

    println!("All done. Agent still running (pid {}).", agent.id());
    println!("If you want to stop agent, kill pid {} or CTRL + C.", agent.id());

    Ok(())

}