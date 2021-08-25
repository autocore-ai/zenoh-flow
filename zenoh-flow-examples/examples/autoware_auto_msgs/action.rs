use ffi_convert::*;
use zenoh_flow::serde::{Deserialize, Serialize};

// autoware_auto_msgs/action/ReplayTrajectory.idl
#[derive(Serialize, Deserialize)]
pub struct ReplayTrajectoryGoal {
    pub replay_path: String,
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(ReplayTrajectoryGoal)]
pub struct CReplayTrajectoryGoal {
    pub replay_path: *const libc::c_char,
}

#[derive(Serialize, Deserialize)]
pub struct ReplayTrajectoryFeedback {
    pub record_replay_state: i32,
    pub remaining_length: i32,
}

#[repr(C)]
#[derive(CReprOf, AsRust, CDrop)]
#[target_type(ReplayTrajectoryFeedback)]
pub struct CReplayTrajectoryFeedback {
    pub record_replay_state: i32,
    pub remaining_length: i32,
}
