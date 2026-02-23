use rclrs::*;
use px4_msgs::msg::VehicleCommand;
use std::sync::Arc;

pub struct ArmDrone {
    pub_arm: Arc<Publisher<VehicleCommand>>,
}

impl ArmDrone {
    pub fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            pub_arm: Arc::new(
                node.create_publisher("/px4_1/fmu/in/vehicle_command")?
            ),
        })
    }

    fn send(&self, mut msg: VehicleCommand) {
        msg.target_system = 2;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        self.pub_arm.publish(&msg).ok();
    }

    pub fn arm(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM as u32;
        msg.param1 = 1.0;
        self.send(msg);
    }

    pub fn set_offboard(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE as u32;
        msg.param1 = 1.0;
        msg.param2 = 6.0;
        self.send(msg);
    }
}
