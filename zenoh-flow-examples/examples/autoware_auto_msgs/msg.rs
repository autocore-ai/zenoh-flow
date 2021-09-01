use cxx::{CxxVector, UniquePtr};
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;

//注意：rust的struct没有默认类型，即没有idl中的@default （value）

// autoware_auto_msgs/msg/VehicleKinematicState.idl
#[derive(Serialize, Deserialize)]
pub struct VehicleKinematicState {
    pub header: std_msgs::msg::Header,
    pub state: autoware_auto_msgs::msg::TrajectoryPoint,
    pub delta: geometry_msgs::msg::Transform
}

// autoware_auto_msgs/msg/TrajectoryPoint.idl
#[derive(Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub time_from_start: builtin_interfaces::msg::Duration,
    pub x: f32,
    pub y: f32,
    pub heading: autoware_auto_msgs::msg::Complex32,
    pub longitudinal_velocity_mps: f32,
    pub lateral_velocity_mps: f32,
    pub acceleration_mps2: f32,
    pub heading_rate_rps: f32,
    pub front_wheel_angle_rad: f32,
    pub rear_wheel_angle_rad: f32,   
}

// autoware_auto_msgs/msg/Complex32.idl
#[derive(Serialize, Deserialize)]
pub struct Complex32 {
    pub real: f32,
    pub imag: f32,
}

// autoware_auto_msgs/msg/Trajectory.idl
#[derive(Serialize, Deserialize)]
pub struct Trajectory {
    pub header: std_msgs::msg::Header, 
    pub points: Vec<autoware_auto_msgs::msg::TrajectoryPoint>,
}
