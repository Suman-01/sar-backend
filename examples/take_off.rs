use rclrs::*;
use px4_msgs::msg::TrajectorySetpoint;
use std::sync::Arc;

pub struct TakeOff {
    pub_toff: Arc<Publisher<TrajectorySetpoint>>,
}

impl TakeOff {
    pub fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self { 
            pub_toff: Arc::new(
                node.create_publisher("/desired_setpoint")?
            ),
        })
    }

    pub fn takeoff(&self) {
        let mut setpoint = TrajectorySetpoint::default();
        setpoint.position = [0.0, 0.0, -5.0];
        setpoint.yaw = 0.0;
        self.pub_toff.publish(&setpoint).ok();
    }
}