// Mapper to keep a 2D grid in memory

use rclrs::*;
use geometry_msgs::msg::Pose;
use nav_msgs::msg::{MapMetaData, OccupancyGrid};
use sensor_msgs::msg::Image;
use std::sync::{Arc, Mutex};

use crate::state_estimation::{PoseEstimate, StateEstimator};

