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
    pub fn new(exe: &Executor) -> Result<Self, RclrsError> {
        let node = exe.create_node("offboard_control")?;

        let px4_1 = Px4Publishers {
            offboard: Arc::new(node.create_publisher("/px4_1/fmu/in/offboard_control_mode")?),
            traj: Arc::new(node.create_publisher("/px4_1/fmu/in/trajectory_setpoint")?),
            cmd: Arc::new(node.create_publisher("/px4_1/fmu/in/vehicle_command")?),
            sys_id: 2,
        };

        let px4_2 = Px4Publishers {
            offboard: Arc::new(node.create_publisher("/px4_2/fmu/in/offboard_control_mode")?),
            traj: Arc::new(node.create_publisher("/px4_2/fmu/in/trajectory_setpoint")?),
            cmd: Arc::new(node.create_publisher("/px4_2/fmu/in/vehicle_command")?),
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

    fn pub_offboard(p: &Px4Publishers) {
        let mut msg = OffboardControlMode::default();
        msg.position = true;
        msg.timestamp = Self::ts();
        let _ = p.offboard.publish(&msg);
    }

    fn pub_traj(p: &Px4Publishers) {
        let mut msg = TrajectorySetpoint::default();
        msg.position = [0.0, 0.0, -5.0];
        msg.yaw = -3.14;
        msg.timestamp = Self::ts();
        let _ = p.traj.publish(&msg);
    }

    fn pub_cmd(p: &Px4Publishers, cmd: u32, p1: f32, p2: f32) {
        let mut msg = VehicleCommand::default();
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = p.sys_id;
        msg.target_component = 1;
        msg.source_system = 255;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = Self::ts();
        let _ = p.cmd.publish(&msg);
    }

    fn set_offboard(p: &Px4Publishers) {
        Self::pub_cmd(
            p,
            VehicleCommand::VEHICLE_CMD_DO_SET_MODE as u32,
            1.0,
            6.0,
        );
    }

    fn arm(p: &Px4Publishers) {
        Self::pub_cmd(
            p,
            VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM as u32,
            1.0,
            0.0,
        );
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let ctrl = Arc::new(OffboardControl::new(&executor)?);

    let c = ctrl.clone();
    thread::spawn(move || {
        let mut i = 0usize;
        loop {
            OffboardControl::pub_offboard(&c.px4_1);
            OffboardControl::pub_offboard(&c.px4_2);

            OffboardControl::pub_traj(&c.px4_1);
            OffboardControl::pub_traj(&c.px4_2);

            if i == 10 {
                OffboardControl::set_offboard(&c.px4_1);
                OffboardControl::set_offboard(&c.px4_2);
                thread::sleep(Duration::from_millis(50));
                OffboardControl::arm(&c.px4_1);
                OffboardControl::arm(&c.px4_2);
            }

            i = i.saturating_add(1);
            thread::sleep(Duration::from_millis(100));
        }
    });

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
