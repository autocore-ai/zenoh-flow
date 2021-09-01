use cxx::{CxxString, UniquePtr};
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;

// geometry_msgs/msg/Transform.idl
#[derive(Serialize, Deserialize)]
pub struct Transform {
    pub translation: geometry_msgs::msg::Vector3,
    pub rotation: geometry_msgs::msg::Quaternion,
}

// geometry_msgs/msg/Vector3.idl
#[derive(Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

// geometry_msgs/msg/Quaternion.idl
#[derive(Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

// geometry_msgs/msg/TransformStamped.idl
#[derive(Serialize, Deserialize)]
pub struct TransformStamped {
    pub header: std_msgs::msg::Header,
    pub child_frame_id: String,
    pub transform: geometry_msgs::msg::Transform,
}
