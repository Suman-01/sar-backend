// simple state esimator for pose and orientation

use rclrs::*;
use geometry_msgs::msg::PoseStamped;
use px4_msgs::msg::VehicleOdometry;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Clone, Debug, Default)]

pub struct PoseEstimate {
    // Translational Comp
    pub x: f64,
    pub y: f64,
    pub z: f64,
    // Rotational Comp (eul ang)
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    //quaternions
    pub qw: f64,    
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

pub struct StateEstimator {
    _node: Node,
    _sub_odometry: Subscription<VehicleOdometry>,
    pos_pub: Arc<Publisher<PoseStamped>>,
    curr_pos: Arc<Mutex<PoseEstimate>>,
    frame_id: String,
}

impl StateEstimator {

    // Timestamp corresponding to the unix epoch const time, date
    fn timestamp() -> u64 {
        SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_micros() as u64
    }
    // Quaternions to Roll, Pitch, Yaw
    fn quat_to_rpy(qw: f64, qx: f64, qy: f64, qz: f64) -> (f64, f64, f64) {
        // Roll
        let s_r = 2.0 * (qw*qx + qy*qz);
        let c_r = 1.0 - 2.0*(qx*qx + qy*qy);
        let roll = s_r.atan2(c_r);

        // Pitch
        s_p = 2.0 * (qw*qy - qz*qx);

        let pitch = if s_p.abs() >= 1.0 {
            s_p.signum() * std::f64::consts::FRAC_PI_2
        } else {
            s_p.asin2()
        };

        // Yaw
        s_y = 2.0 * (qw*qz - qx*qy);
        c_y = 1.0 -  2.0*(qy*qy + qz*qz);
        let yaw = s_y.atan2(c_y);

        (roll, pitch, yaw)
    }

    fn fill_pose_msg(frame_id: &str, est: &PoseEstimate) -> PoseStamped {
        let mut msg = PoseStamped::default();
        msg.header.frame_id = frame_id.to_string();
        msg.header.stamp.sec = (Self::timestamp() / 1_000_000) as i32;
        msg.header.stamp.nanosec = ((Self::timestamp() % 1_000_000) * 1000) as u32;

        msg.pose.position.x = est.x;
        msg.pose.position.y = est.y;
        msg.pose.position.z = est.z;

        msg.pose.orientation.w = est.qw;
        msg.pose.orientation.x = est.qx;
        msg.pose.orientation.y = est.qy;
        msg.pose.orientation.z = est.qz;

        msg
    }

    pub fn new(exe: &Executor, ns: &str, od_topic: &str) -> Result<Self, RclrsError> {
        // node init
        let node_name = format!("state_estimator_{}", ns.replace('/', "_"));
        let node = exe.create_node(node_name.as_str())?;

        // pub init
        pose_topic = format!("/{}/estimated_pose", ns);
        pose_pub = Arc::new(node.create_publisher::<PoseStamped>(pose_topic.as_str())?);

        let curr_pos = Arc::new(Mutex::new(PoseEstimate::default()));
        let frame_id = "map".to_string();

        let curr_pos_cl = curr_pos.clone();
        let pose_pub_cl = pose_pub.clone();
        let frame_id_cl = frame_id.clone();

        let sub_odometry = node.create_subscription::<VehicleOdometry, _>(
            od_topic,
            move |msg: VehicleOdometry| {
                let p = msg.position;   // x, y, z
                let q = msg.q;  // quaternions

                let qw = q[0] as f64;
                let qx = q[1] as f64;
                let qy = q[2] as f64;
                let qz = q[3] as f64;

                let (roll, pitch, yaw) = Self::quat_to_rpy(qw, qx, qy, qz);

                let est_loc = PoseEstimate {
                    x: p[0] as f64,
                    y: p[1] as f64,
                    z: p[2] as f64,
                    roll,
                    pitch,
                    yaw,
                    qw,
                    qx,
                    qy,
                    qz,
                };

                *curr_pos_cl.lock().unwrap() = est_loc.clone();

                let pose_msg = Self::fill_pose_msg(&frame_id, &est_loc);
                let _ = pose_pub_cl.publish(&pose_msg); // real-time propagation (Time driven)
            },
        )?;

        Ok(Self {
            _node: node,
            _sub_odometry: sub_odometry,
            pos_pub,
            curr_pos,
            frame_id,
        })
    }

    pub fn curr_pos(&self) -> PoseEstimate {
        self.curr_pos.lock().unwrap().clone()
    }

    pub fn publish_curr_pos(&self) {
        let pos = self.curr_pos();
        let msg = Self::fill_pose_msg(&self.frame_id, &pos);
        let _ = self.pos_pub.publish(&msg); // Manual (Event-Driven)
    }
}




