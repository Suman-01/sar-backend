use rclrs::*;
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Clone, Copy)]
pub struct Target {
    pub n: f64,
    pub e: f64,
    pub d: f64,
    pub yaw: f64,
}

pub struct OffboardController {
    _node: Node,
    off_pub: Arc<Publisher<OffboardControlMode>>,
    traj_pub: Arc<Publisher<TrajectorySetpoint>>,
    target: Arc<Mutex<Target>>,
}

impl OffboardController {
    fn ts() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64
    }

    pub fn new(exe: &Executor) -> Result<Self, RclrsError> {
        let node = exe.create_node("offboard_controller")?;

        let off_pub = Arc::new(
            node.create_publisher("/px4_1/fmu/in/offboard_control_mode")?
        );

        let traj_pub = Arc::new(
            node.create_publisher("/px4_1/fmu/in/trajectory_setpoint")?
        );

        let target = Arc::new(Mutex::new(Target {
            n: 0.0,
            e: 0.0,
            d: 0.0,
            yaw: 0.0,
        }));

        Ok(Self { _node: node, off_pub, traj_pub, target })
    }

    pub fn set_target(&self, n: f64, e: f64, d: f64, yaw: f64) {
        *self.target.lock().unwrap() = Target { n, e, d, yaw };
    }

    pub fn spin_loop(&self) {
        loop {
            let mut off = OffboardControlMode::default();
            off.position = true;
            off.timestamp = Self::ts();
            self.off_pub.publish(&off).ok();

            let t = *self.target.lock().unwrap();

            let mut sp = TrajectorySetpoint::default();
            sp.timestamp = Self::ts();
            sp.position = [t.n as f32, t.e as f32, t.d as f32];
            sp.yaw = t.yaw as f32;

            self.traj_pub.publish(&sp).ok();

            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}
