use rclrs::*;
use sensor_msgs::msg::Image;
use std::sync::{Arc, Mutex};

pub struct TsdfListner {
    pub latest_image: Arc<Mutex<Option<Image>>>,
}

impl TsdfListner {
    pub fn new(node: &Node) -> Result<Self, RclrsError> {
        let latest_image = Arc::new(Mutex(None));

        let img_clone = latest_image.clone();

        node.create_subscription::<Image, _>(
            "nvblox_boundary_image",
            move |msg| {
                println!(
                    "Recieved TSDF image: {} x {}",
                    msg.width,
                    msg.height
                );

                *img.lock().unwrap() = Some(msg);
            },
        )?;

        Ok(Self{latest_image})
    }
}
