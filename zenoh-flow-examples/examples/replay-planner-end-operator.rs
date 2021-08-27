//
// Copyright (c) 2017, 2021 ADLINK Technology Inc.
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ADLINK zenoh team, <zenoh@adlink-labs.tech>
//
mod std_msgs;
mod builtin_interfaces;
mod autoware_auto_msgs;
mod geometry_msgs;
mod tf2_msgs;

use async_std::sync::Arc;
use std::collections::HashMap;
use zenoh_flow::{
    export_operator, get_input,
    serde::{Deserialize, Serialize},
    types::{
        DataTrait, FnInputRule, FnOutputRule, FnRun, InputRuleOutput, OperatorTrait,
        OutputRuleOutput, RunOutput, StateTrait, ZFInput, ZFResult, ZFError
    },
    zenoh_flow_derive::ZFState,
    zf_data, zf_empty_state, Token, ZFComponentOutput, ZFContext,
};

use zenoh_flow_examples::{ZFString, ZFUsize, ZFBytes};
use autoware_auto_msgs::action::{ReplayTrajectoryFeedback};
use autoware_auto_msgs::msg::{VehicleKinematicState, CVehicleKinematicState, Trajectory, CTrajectory};
use uhlc::{Timestamp, NTP64, ID};

static LINK_ID_INPUT_PLANNED_TRAJECTORY: &str = "planned_trajectory";  //规划好的轨迹
static LINK_ID_INPUT_REPLAY_TRAJECTORY_FEEDBACK: &str = "replay_trajectory_feedback";  //规划器反馈
static LINK_ID_OUTPUT: &str = "output";  //输出

#[derive(Serialize, Deserialize, Debug, ZFState)]
struct ReplayPlannerEndOperator {}

impl ReplayPlannerEndOperator {
    fn input_rule(_ctx: ZFContext, inputs: &mut HashMap<String, Token>) -> InputRuleOutput {
        for token in inputs.values() {
            match token {
                Token::Ready(_) => continue,
                Token::NotReady => return Ok(false),
            }
        }

        Ok(true)
    }

    fn run(_ctx: ZFContext, mut inputs: ZFInput) -> RunOutput {
        let mut result = HashMap::<String, Arc<dyn DataTrait>>::with_capacity(1);
        //接收反馈消息
        let (timestamp, replay_trajectory_feedback)= match get_input!(ZFBytes, String::from(LINK_ID_INPUT_REPLAY_TRAJECTORY_FEEDBACK), inputs) {
            Ok((timestamp, replay_trajectory_feedback)) => (timestamp, replay_trajectory_feedback),
            Err(e) => {
                if e == ZFError::MissingInput(LINK_ID_INPUT_REPLAY_TRAJECTORY_FEEDBACK.to_string()) {
                    println!("Missing Input {}", LINK_ID_INPUT_REPLAY_TRAJECTORY_FEEDBACK.to_string());
                    let buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                    (Timestamp::new(NTP64(0), ID::new(0, buf)), ZFBytes{bytes: Vec::new()})
                } else {
                    return Err(e);
                }
            }
        };
        if !replay_trajectory_feedback.bytes.is_empty() {
            let replay_trajectory_feedback: ReplayTrajectoryFeedback = bincode::deserialize(&(replay_trajectory_feedback.bytes)).unwrap();
            println!("Received replay_trajectory_feedback, trajectory_length is {}, planner_status is {}", replay_trajectory_feedback.remaining_length, replay_trajectory_feedback.record_replay_state);
        }

        //接收规划好的路径
        let (timestamp, trajectory_data)= match get_input!(ZFBytes, String::from(LINK_ID_INPUT_PLANNED_TRAJECTORY), inputs) {
            Ok((timestamp, trajectory_data)) => (timestamp, trajectory_data),
            Err(e) => {
                if e == ZFError::MissingInput(LINK_ID_INPUT_PLANNED_TRAJECTORY.to_string()) {
                    println!("Missing Input {}", LINK_ID_INPUT_PLANNED_TRAJECTORY.to_string());
                    let buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                    (Timestamp::new(NTP64(0), ID::new(0, buf)), ZFBytes{bytes: Vec::new()})
                } else {
                    return Err(e);
                }
            }
        };
        if !trajectory_data.bytes.is_empty() {
            let trajectory_data: Trajectory = bincode::deserialize(&(trajectory_data.bytes)).unwrap();
            println!("Received planned_trajectory_data, planned_trajectory_length is {}", trajectory_data.points.len());
        }

        result.insert(String::from(LINK_ID_OUTPUT), zf_data!(ZFString("end".to_string())));
        Ok(result)
    }
}

impl OperatorTrait for ReplayPlannerEndOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(ReplayPlannerEndOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(ReplayPlannerEndOperator::run)
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(zenoh_flow::default_output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        zenoh_flow::zf_empty_state!()
    }
}

export_operator!(register);

extern "C" fn register(
    _configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::OperatorTrait + Send>> {
    Ok(Box::new(ReplayPlannerEndOperator {}) as Box<dyn zenoh_flow::OperatorTrait + Send>)
}