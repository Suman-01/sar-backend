use rclrs::*;
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

pub struct OffboardController {
    _node: Node,
    off_pub: Arc<Publisher<OffboardControlMode>>,
    traj_pub: Arc<Publisher<TrajectorySetpoint>>,
    sp_pub: Arc<Publisher<TrajectorySetpoint>>,
}

impl OffboardController {
    fn ts() -> u64 {
        SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_micros() as u64s
    }

    pub fn new(exe: &Executor) -> Result<Self, RclrsError> {
        let node = exe.create_node("offboard_controller")?;

        let off_pub = Arc::new(
            node.create_publisher("/px4_1/fmu/in/offboard_control_mode")?
        );

        let traj_pub = Arc::new(
            node.create_publisher("/px4_1/fmu/in/trajectory_setpoint")?
        );

        let sp_pub = Arc::new(Mutex::new(TrajectorySetpoint::default()));
        let sp_clone = sp_pub.clone();

        node.create_subscription::<TrajectorySetpoint>(
            "/desired_setpoint",
            move |msg: TrajectorySetpoint| {
                *sp_clone.lock().unwrap() = msg;
            },
        )?;

        Ok(Self { 
            _node: node, 
            off_pub, 
            traj_pub, 
            sp_pub,
        })
    }

    pub fn spin_loop(&self) {
        loop {
            let mut offboard = OffboardControlMode::default();
            offboard.position = true;
            offboard.timestamp = Self::ts();
            self.off_pub.publish(&offboard).ok();

            let mut setpoint = self.sp_pub.lock().unwrap().clone();
            setpoint.timestamp = Self::ts();
            self.traj_pub.publish(&setpoint).ok();

            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}