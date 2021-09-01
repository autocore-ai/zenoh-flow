use cxx::{CxxVector, UniquePtr};
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;

// tf2_msgs/msg/TFMessage.idl
#[derive(Serialize, Deserialize)]
pub struct TFMessage {
    pub transforms: Vec<geometry_msgs::msg::TransformStamped>,
}
