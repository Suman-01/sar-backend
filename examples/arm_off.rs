use rclrs::*;
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint, VehicleCommand};
use std::error::Error;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

struct Px4Publishers {
    offboard: Arc<Publisher<OffboardControlMode>>,
    traj: Arc<Publisher<TrajectorySetpoint>>,
    cmd: Arc<Publisher<VehicleCommand>>,
    sys_id: u8,
}

struct OffboardControl {
    _node: Node,
    px4_1: Px4Publishers,
    px4_2: Px4Publishers,
}

impl OffboardControl {
    /// Create node + publishers. Remap the topic names if your PX4 expects different paths.
    pub fn new(exe: &Executor) -> Result<Self, RclrsError> {
        // create a node via executor (depends on rclrs version; this matches your pattern)
        let node = exe.create_node("offboard_control")?;

        let px4_1 = Px4Publishers {
            offboard: Arc::new(node.create_publisher::<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode")?),
            traj: Arc::new(node.create_publisher::<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint")?),
            cmd: Arc::new(node.create_publisher::<VehicleCommand>("/px4_1/fmu/in/vehicle_command")?),
            // MUST match the system id of the PX4 instance for drone1
            sys_id: 2,
        };

        let px4_2 = Px4Publishers {
            offboard: Arc::new(node.create_publisher::<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode")?),
            traj: Arc::new(node.create_publisher::<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint")?),
            cmd: Arc::new(node.create_publisher::<VehicleCommand>("/px4_2/fmu/in/vehicle_command")?),
            // MUST match the system id of the PX4 instance for drone2
            sys_id: 3,
        };

        Ok(Self { _node: node, px4_1, px4_2 })
    }

    fn ts() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64
    }

    /// Publish OffboardControlMode (position enabled). Keep publishing frequently.
    fn pub_offboard(p: &Px4Publishers) {
        let mut msg = OffboardControlMode::default();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = Self::ts();
        if let Err(e) = p.offboard.publish(&msg) {
            eprintln!("failed publish offboard (sys {}): {:?}", p.sys_id, e);
        }
    }

    /// Publish a position trajectory setpoint at a fixed location (example).
    /// Replace with your real setpoint generation / mission logic.
    fn pub_traj(p: &Px4Publishers, north: f32, east: f32, down: f32, yaw: f32) {
        let mut msg = TrajectorySetpoint::default();
        // PX4 expects NED: x = north, y = east, z = down (down positive)
        msg.position = [north, east, down];
        msg.yaw = yaw;
        msg.timestamp = Self::ts();
        if let Err(e) = p.traj.publish(&msg) {
            eprintln!("failed publish traj (sys {}): {:?}", p.sys_id, e);
        }
    }

    /// Send a generic VehicleCommand. Uses target_system from Px4Publishers struct.
    fn pub_cmd(p: &Px4Publishers, cmd: u32, p1: f32, p2: f32, p3: f32, p4: f32) {
        let mut msg = VehicleCommand::default();
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.param3 = p3;
        msg.param4 = p4;
        msg.target_system = p.sys_id as u8 as u8;
        msg.target_component = 1;
        // source_system/source_component: small values are ok; some bridges expect 1
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = Self::ts();
        if let Err(e) = p.cmd.publish(&msg) {
            eprintln!("failed publish vehicle_command (sys {}): {:?}", p.sys_id, e);
        }
    }

    /// Ask PX4 to switch to OFFBOARD (VEHICLE_CMD_DO_SET_MODE)
    /// PX4 expects param1=custom_sub_mode(?), param2=custom_main_mode (6 = OFFBOARD), but historically param1=1.0, param2=6.0 works.
    fn set_offboard(p: &Px4Publishers) {
        // param1: base_mode? many setups use 1.0; param2: custom_main_mode=6 (OFFBOARD)
        Self::pub_cmd(p, VehicleCommand::VEHICLE_CMD_DO_SET_MODE as u32, 1.0, 6.0, 0.0, 0.0);
    }

    /// Request arm (VEHICLE_CMD_COMPONENT_ARM_DISARM)
    fn arm(p: &Px4Publishers) {
        // param1 = 1.0 to arm, 0.0 to disarm
        Self::pub_cmd(p, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM as u32, 1.0, 0.0, 0.0, 0.0);
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    // Initialize rclrs context & executor
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    // Build control object
    let ctrl = Arc::new(OffboardControl::new(&executor)?);

    // Clone for thread
    let ctl = ctrl.clone();

    // Background thread: continuously publish offboard+trajectory and attempt arm/mode
    // Purpose: keep setpoints flowing (>= 2 Hz) while sending ARM/MODE commands.
    thread::spawn(move || {
        // We'll publish at ~10 Hz
        let publish_period = Duration::from_millis(100);
        // After this many publishes, send set-mode + arm sequence (and then repeat if necessary)
        let mut publish_count: usize = 0;

        // How many times to retry set-mode/arm (per drone). Use large value or loop forever if you prefer.
        let max_mode_attempts = 50usize;

        let mut mode_attempts_1 = 0usize;
        let mut mode_attempts_2 = 0usize;

        loop {
            // 1) Publish OffboardControlMode for both drones (required continuously)
            OffboardControl::pub_offboard(&ctl.px4_1);
            OffboardControl::pub_offboard(&ctl.px4_2);

            // 2) Publish a TrajectorySetpoint for each drone.
            // Example: hold at N=0, E=0, down=-5m (i.e., altitude +5m)
            // Replace with real mission setpoint generation.
            OffboardControl::pub_traj(&ctl.px4_1, 0.0, 0.0, -5.0, 0.0);
            OffboardControl::pub_traj(&ctl.px4_2, 0.0, 0.0, -5.0, 0.0);

            // 3) After a short warmup period, send set-mode + arm (and retry until attempts exhausted).
            // We wait until publish_count reaches 10 (~1 second at 100ms) to ensure PX4 receives some setpoints first.
            if publish_count >= 10 {
                // Drone 1: set mode then arm (retry loop limited by max_mode_attempts)
                if mode_attempts_1 < max_mode_attempts {
                    OffboardControl::set_offboard(&ctl.px4_1);
                    // small delay between mode and arm
                    thread::sleep(Duration::from_millis(50));
                    OffboardControl::arm(&ctl.px4_1);
                    mode_attempts_1 = mode_attempts_1.saturating_add(1);
                }

                // Drone 2
                if mode_attempts_2 < max_mode_attempts {
                    OffboardControl::set_offboard(&ctl.px4_2);
                    thread::sleep(Duration::from_millis(50));
                    OffboardControl::arm(&ctl.px4_2);
                    mode_attempts_2 = mode_attempts_2.saturating_add(1);
                }
            }

            publish_count = publish_count.saturating_add(1);
            thread::sleep(publish_period);
        }
    });

    // Spin the executor in the main thread — this keeps subscriptions and publishers alive.
    // Use SpinOptions::default() as you had before.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
