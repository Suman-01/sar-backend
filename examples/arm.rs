//This thing only arms and runs the drone 1


use rclrs::*;
use px4_msgs::msg::VehicleCommand;
use std::sync::Arc;

pub struct ArmDrone {
    pub_arm: Arc<Publisher<VehicleCommand>>,
}

impl ArmDrone {
    pub fn new(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self{
            pub_arm: Arc::new(
                node.create_publisher("/px4_1/fmu/in/vehicle_command")?
            ),
        })
    }

    pub fn arm(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM as u32;   //u16 -> u32
        msg.param1 = 1.0;
        msg.target_system = 2;   //only drone1
        msg.from_external = true;
        self.pub_arm.publish(&msg).ok();
    }

    pub fn set_offboard(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE as u32;   //u16 -> u32
        msg.param1 = 1.0;
        msg.param2 = 6.0;
        msg.target_system = 2;   //only drone1
        msg.from_external = true;
        self.pub_arm.publish(&msg).ok();

    }
}