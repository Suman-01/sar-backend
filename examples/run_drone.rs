use std::process::{Command, Child, Stdio};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::io::{self, Write};

fn ros2_publisher(topic: &str, msg_type: &str, yaml_body: &str, rate: u32, duration: u64, src: &str) -> Result<(), String> {
    let cmd = format!(
        "{} && ros2 topic pub -r {} {} {} \"{}\"",
        src, rate, topic, msg_type, yaml_body.replace('"', "\\\"")
    );

    let mut child = Command::new("bash")
                                   .arg("-lc")
                                   .arg(cmd)
                                   .stdout(Stdio::null())
                                   .stderr(Stdio::piped())
                                   .spawn()
                                   .map_err(|e| format!("Failed to staet ros2 topic publisher: {}", e))?;
    
    let start = Instant::now();

    while start.elapsed() < Duration::from_secs(duration) {
        sleep(Duration::from_millis(200));

        match child.try_wait() {
            Ok(Some(status)) => {
                return Err(format!("ros2 topic exited with status: {}", status));
            }
            Ok(None) => {}
            Err(e) => return Err(format!("Error checking ros2 pub process: {}", e)),
        }
    }

    let _ = child.kill();
    let _ = child.wait();

    Ok(())
            
}

fn trajectory_yaml(x: f64, y: f64, z: f64, yaw: f64) -> String {
    format! (
        "x: {x:.3}\n
         y: {y:.3}\n
         z: {z:.3}\n
         yaw: {yaw:.3}",
         x = x, 
         y = y,
         z = z,
         yaw = yaw
    )
}

fn rectangle (cx: f64, cy: f64, w: f64, h: f64, z: f64) -> Vec<(f64, f64, f64)> {
    let hw = w / 2.0;
    let hh = h / 2.0;

    vec![
        (cx - hw, cy - hh, z),
        (cx + hw, cy - hh, z),
        (cx + hw, cy + hh, z),
        (cx - hw, cy + hh, z),
    ]
}

fn main() -> Result<(), String> {
    let ros2_source = "source /opt/ros/humble/setup.bash";
    let topic = "/fmu/trajectory_setpoint";
    let msg_type = "px4_msgs/msg/TrajectorySetpoint";

    let publish_rate_hz: u32 = 10;
    let hold_secs_per_waypoint: u64 = 4;
    let loops: u32 = 1;

    let rect1  = (0.0, 0.0, 4.0, 2.0, 3.0);
    let rect2 = (3.0, 0.0, 4.0, 2.0, 3.0);

    let drones: Vec<u32> = vec![1, 2];

    println!("Starting rectangle waypoint publisher.");
    println!("Publishing to topic {} type {}", topic, msg_type);
    println!("Each waypoint will be published at {} Hz for {} s", publish_rate_hz, hold_secs_per_waypoint);

    let corners1 = rectangle(rect1.0, rect1.1, rect1.2, rect1.3, rect1.4);
    let corners2 = rectangle(rect2.0, rect2.1, rect2.2, rect2.3, rect2.4);

    // Operate one drone at a time (sequential)

    for (id, &sys) in drones.iter().enumerate() {
        println!("\n=== Commanding drine {} ===", sys);
        let corners = if id == 0 { &corners1 } else { &corners2 };

        for loop_id in 0..loops {
            println!("Loop {}/{} for drone {}", loop_id+1, loops, sys);
            for (i, &(x, y, z)) in corners.iter().enumerate() {
                println!(" -> Waypoint {}: x={:.2}, y={:.2}, z={:.2}", i+1, x, y, z);

                let yaml = trajectory_yaml(x, y, z, 0.0);

                let dur: u64 = hold_secs_per_waypoint;

                print!("   publishing...");

                io::stdout().flush().ok();

                match ros2_publisher(topic, msg_type, &yaml, publish_rate_hz, dur, ros2_source) {
                    Ok(_) => println!(" done"),
                    Err(e) => println!(" error: {}", e),
                }

                sleep(Duration::from_millis(300));
            }
        }

        println!("Drone {} finished rectangle traversal.", sys);
    
    }

    println!("All drones finished their rectangle paths.");
    Ok(())

}



