use cxx::{UniquePtr, CxxString};
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;
// std_msgs/msg/Header.idl
#[derive(Serialize, Deserialize)]
pub struct Header {
    pub stamp: builtin_interfaces::msg::Time,
    pub frame_id: String,
}
