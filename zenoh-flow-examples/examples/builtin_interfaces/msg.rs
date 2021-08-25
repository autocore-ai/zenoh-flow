use ffi_convert::*;
use zenoh_flow::serde::{Deserialize, Serialize};

// builtin_interfaces/msg/Time.idl
#[derive(Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(Time)]
pub struct CTime {
    pub sec: i32,
    pub nanosec: u32
}


#[derive(Serialize, Deserialize)]
// builtin_interfaces/msg/Duration.idl
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(Duration)]
pub struct CDuration {
    pub sec: i32,
    pub nanosec: u32
}
