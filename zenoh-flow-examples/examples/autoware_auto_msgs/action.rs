use zenoh_flow::serde::{Deserialize, Serialize};

// autoware_auto_msgs/action/ReplayTrajectory.idl
#[derive(Serialize, Deserialize)]
pub struct ReplayTrajectoryGoal {
    pub replay_path: String,
}

#[derive(Serialize, Deserialize)]
pub struct ReplayTrajectoryFeedback {
    pub record_replay_state: i32,
    pub remaining_length: i32,
}
