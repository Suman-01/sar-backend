// arm.rs
use rclrs::*;
use px4_msgs::msg::VehicleCommand;
use std::sync::Arc;

#[derive(Clone)]
pub struct ArmDrone {
    pub_arm: Arc<Publisher<VehicleCommand>>,
    system_id: u8,
}

impl ArmDrone {
    /// new(node, namespace, system_id)
    /// namespace like "px4_1" or "px4_2"
    pub fn new(node: &Node, ns: &str, system_id: u8) -> Result<Self, RclrsError> {
        let topic = format!("/{}/fmu/in/vehicle_command", ns);
        Ok(Self {
            pub_arm: Arc::new(node.create_publisher(topic.as_str())?),
            system_id,
        })
    }

    fn send(&self, mut msg: VehicleCommand) {
        // set correct target system from constructor
        msg.target_system = self.system_id as u8;
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

    pub fn disarm(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM as u32;
        msg.param1 = 0.0;
        self.send(msg);
    }

    pub fn set_offboard(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE as u32;
        msg.param1 = 1.0;
        msg.param2 = 6.0;
        self.send(msg);
    }

    pub fn land(&self) {
        let mut msg = VehicleCommand::default();
        msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND as u32;
        self.send(msg);
    }
}
