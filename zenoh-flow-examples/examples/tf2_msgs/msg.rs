use ffi_convert::*;
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;

// tf2_msgs/msg/TFMessage.idl
#[derive(Serialize, Deserialize)]
pub struct TFMessage {
    pub transforms: Vec<geometry_msgs::msg::TransformStamped>,
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(TFMessage)]
pub struct CTFMessage {
    pub transforms: CArray<geometry_msgs::msg::CTransformStamped>,
}