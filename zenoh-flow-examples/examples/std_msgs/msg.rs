use ffi_convert::*;
use zenoh_flow::serde::{Deserialize, Serialize};

use crate::*;
// std_msgs/msg/Header.idl
#[derive(Serialize, Deserialize)]
pub struct Header {
    pub stamp: builtin_interfaces::msg::Time,
    pub frame_id: String
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(Header)]
pub struct CHeader {
    pub stamp: builtin_interfaces::msg::CTime,
    pub frame_id: *const libc::c_char
}