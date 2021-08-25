use ffi_convert::*;
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;

// geometry_msgs/msg/Transform.idl
#[derive(Serialize, Deserialize)]
pub struct Transform {
    pub translation: geometry_msgs::msg::Vector3,
    pub rotation: geometry_msgs::msg::Quaternion,
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(Transform)]
pub struct CTransform {
    pub translation: geometry_msgs::msg::CVector3,
    pub rotation: geometry_msgs::msg::CQuaternion,
}


// geometry_msgs/msg/Vector3.idl
#[derive(Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(Vector3)]
pub struct CVector3 {
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

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(Quaternion)]
pub struct CQuaternion {
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

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(TransformStamped)]
pub struct CTransformStamped {
    pub header: std_msgs::msg::CHeader,
    pub child_frame_id: *const libc::c_char,
    pub transform: geometry_msgs::msg::CTransform,
}
