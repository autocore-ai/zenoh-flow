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
    types::{
        DataTrait, FnInputRule, FnOutputRule, FnRun, InputRuleOutput, OperatorTrait,
        OutputRuleOutput, RunOutput, StateTrait, ZFInput, ZFResult, ZFError
    },
    zf_data, zf_empty_state, Token, ZFComponentOutput, ZFContext,
    serde::{Deserialize, Serialize},
    zenoh_flow_derive::ZFState,
};
use zenoh_flow_examples::{ZFString, ZFUsize, ZFBytes};

use uhlc::{Timestamp, NTP64, ID};
use std::ffi::{CString, c_void};
use libc::c_char;

use autoware_auto_msgs::msg::{VehicleKinematicState, CVehicleKinematicState, Trajectory, CTrajectory};
use autoware_auto_msgs::action::{ReplayTrajectoryGoal, ReplayTrajectoryFeedback};

use ffi_convert::*;

//input and output ID
static LINK_ID_INPUT_VEHICLE_STATE: &str = "vehicle_state"; //车辆状态
static LINK_ID_INPUT_REPLAY_TRAJECTORY_GOAL: &str = "replay_trajectory_goal";   //路径规划目标

static LINK_ID_OUTPUT_PLANNED_TRAJECTORY: &str = "planned_trajectory";  //规划好的轨迹
static LINK_ID_OUTPUT_REPLAY_TRAJECTORY_FEEDBACK: &str = "replay_trajectory_feedback";  //规划好的轨迹

//NULL in C
const NULL: *mut c_void = 0 as *mut c_void;

//全局变量
pub struct ReplayPlannerOperatorGlobalVar {
    m_planner_ptr: *mut c_void,
    m_goal_distance_threshold_m: f64, 
    m_goal_angle_threshold_rad: f64, 
    m_odom_frame_id: String,
}
impl ReplayPlannerOperatorGlobalVar {
    pub fn new(ptr: *mut c_void) -> Self {
        ReplayPlannerOperatorGlobalVar{
            m_planner_ptr: ptr, 
            m_goal_distance_threshold_m: 0.0,
            m_goal_angle_threshold_rad: 0.0,
            m_odom_frame_id: String::new(),
        }
    }

    // 可能用不上，如果不能直接修改field的话就把注释去掉
    // pub fn get_m_planner(&self) -> *mut c_void {
    //     self.m_planner
    // }

    // pub fn set_m_planner(&mut self, m_planner: *mut c_void ) {
    //     self.m_planner = m_planner;
    // }

    // pub fn get_m_goal_distance_threshold_m(&self) -> f32 {
    //     self.m_goal_distance_threshold_m
    // }

    // pub fn set_m_goal_distance_threshold_m(&mut self, m_goal_distance_threshold_m: f32) {
    //     self.m_goal_distance_threshold_m = m_goal_distance_threshold_m;
    // }

    // pub fn get_m_goal_angle_threshold_rad(&self) -> f32 {
    //     self.m_goal_angle_threshold_rad
    // }
    
    // pub fn set_m_goal_angle_threshold_rad(&mut self, m_goal_angle_threshold_rad: f32) {
    //     self.m_goal_angle_threshold_rad = m_goal_angle_threshold_rad;
    // }
}

#[macro_use]
extern crate lazy_static;
extern crate mut_static;
use mut_static::MutStatic;

lazy_static! {
    pub static ref GLOBAL_VAR: MutStatic<ReplayPlannerOperatorGlobalVar> = {
        let ptr = m_planner_new();
        MutStatic::from(ReplayPlannerOperatorGlobalVar::new(ptr))
    };
}

unsafe impl Send for ReplayPlannerOperatorGlobalVar{}
unsafe impl Sync for ReplayPlannerOperatorGlobalVar{}

//configure from graph file
#[derive(Serialize, Deserialize, ZFState, Clone)]
struct Config {
    pub heading_weight: f64,
    pub min_record_distance: f64,
    pub m_goal_distance_threshold_m: f64,
    pub m_goal_angle_threshold_rad: f64,
}

impl std::fmt::Debug for Config {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Config: heading_weight:{:?} min_record_distance:{:?} m_goal_distance_threshold_m:{:?} m_goal_angle_threshold_rad:{:?}",
            self.heading_weight, self.min_record_distance, self.m_goal_distance_threshold_m, self.m_goal_angle_threshold_rad
        )
    }
}

// C++ ENUM
#[derive(Eq, PartialEq)]
#[repr(C)]
enum RecordReplayState
{
  IDLE = 0,
  RECORDING = 1,
  REPLAYING = 2,
  SUCCESS = 3,
}  // enum RecordReplayState

// C++ Interface
extern {
    // 创建一个recordreplay_planner指针
    fn m_planner_new() -> *mut c_void;

    // 获得recordreplay_planner的状态
    fn m_planner_get_record_replay_state(ptr: *mut c_void) -> RecordReplayState;

    fn m_planner_is_replaying(ptr: *mut c_void) -> bool;

    fn m_planner_start_replaying(ptr: *mut c_void);

    fn m_planner_stop_replaying(ptr: *mut c_void);

    fn m_planner_clear_record(ptr: *mut c_void);

    fn m_planner_record_state(ptr: *mut c_void, state_to_record: *mut CVehicleKinematicState) -> bool;

    fn m_planner_get_record_length(m_planner: *const c_void) -> i32;

    fn m_planner_set_heading_weight(m_planner: *const c_void, heading_weight: f64);

    fn m_planner_get_heading_weight(m_planner: *const c_void) -> f64;

    fn m_planner_set_min_record_distance(m_planner: *const c_void, min_record_distance: f64);

    fn m_planner_get_min_record_distance(m_planner: *const c_void) -> f64;

    fn m_planner_read_trajectory_buffer_from_file(m_planner: *const c_void, replay_path: *const c_char );

    fn m_planner_plan(m_planner: *const c_void, current_state: *const autoware_auto_msgs::msg::CVehicleKinematicState) -> *const autoware_auto_msgs::msg::CTrajectory;

    fn m_planner_reached_goal(m_planner: *const c_void, current_state: *const autoware_auto_msgs::msg::CVehicleKinematicState, distance_thresh: f64, angle_thresh: f64) -> bool;
}


struct ReplayPlannerOperator{
    pub config: Config,
}

impl ReplayPlannerOperator {
    //读取graph file config
    fn init(configuration: HashMap<String, String>) -> ZFResult<Self> {
        let heading_weight: f64 = match configuration.get("heading_weight") {
            Some(heading_weight) => heading_weight.trim().parse().unwrap(),
            None =>  return Err(ZFError::MissingConfiguration),
        };

        let min_record_distance: f64 = match configuration.get("min_record_distance") {
            Some(min_record_distance) => min_record_distance.trim().parse().unwrap(),
            None =>  return Err(ZFError::MissingConfiguration),
        };
        
        let m_goal_distance_threshold_m: f64 = match configuration.get("m_goal_distance_threshold_m") {
            Some(m_goal_distance_threshold_m) => m_goal_distance_threshold_m.trim().parse().unwrap(),
            None =>  return Err(ZFError::MissingConfiguration),
        };

        let m_goal_angle_threshold_rad: f64 = match configuration.get("m_goal_angle_threshold_rad") {
            Some(m_goal_angle_threshold_rad) => m_goal_angle_threshold_rad.trim().parse().unwrap(),
            None =>  return Err(ZFError::MissingConfiguration),
        };
        

        let config = Config {
            heading_weight,
            min_record_distance,
            m_goal_distance_threshold_m,
            m_goal_angle_threshold_rad,
        };

        Ok(Self{config})
    }

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

        //接收启动信号
        let (timestamp, replay_trajectory_goal)= match get_input!(ZFBytes, String::from(LINK_ID_INPUT_REPLAY_TRAJECTORY_GOAL), inputs) {
            Ok((timestamp, replay_trajectory_goal)) => (timestamp, replay_trajectory_goal),
            Err(e) => {
                if e == ZFError::MissingInput(LINK_ID_INPUT_REPLAY_TRAJECTORY_GOAL.to_string()) {
                    println!("Missing Input {}", LINK_ID_INPUT_REPLAY_TRAJECTORY_GOAL.to_string());
                    let buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                    (Timestamp::new(NTP64(0), ID::new(0, buf)), ZFBytes{bytes: Vec::new()})
                } else {
                    return Err(e);
                }
            }
        };

        let mut record_replay_state = RecordReplayState::IDLE;
        let mut remaining_length: i32 = 0;
        if !replay_trajectory_goal.bytes.is_empty() {
            // 收到Record路径消息
            let replay_trajectory_goal: ReplayTrajectoryGoal = bincode::deserialize(&(replay_trajectory_goal.bytes)).unwrap();
            unsafe {
                let m_planner_ptr = GLOBAL_VAR.read().unwrap().m_planner_ptr;
                record_replay_state = m_planner_get_record_replay_state(m_planner_ptr);
                if record_replay_state == RecordReplayState::REPLAYING {
                    println!("{}", "Can't start replaying if already are");
                } else {
                    // 开始replay
                    println!("{}", "start replay");
                    let replay_path = CString::new(replay_trajectory_goal.replay_path).expect("CString::new failed");
                    m_planner_read_trajectory_buffer_from_file(m_planner_ptr, replay_path.as_ptr());
                    m_planner_start_replaying(m_planner_ptr);
                    record_replay_state = RecordReplayState::REPLAYING
                }
            }
        }


        let (timestamp, vehicle_state_msg)= match get_input!(ZFBytes, String::from(LINK_ID_INPUT_VEHICLE_STATE), inputs) {
            Ok((timestamp, vehicle_state_msg)) => (timestamp, vehicle_state_msg),
            Err(e) => {
                if e == ZFError::MissingInput(LINK_ID_INPUT_VEHICLE_STATE.to_string()) {
                    println!("Missing Input {}", LINK_ID_INPUT_VEHICLE_STATE.to_string());
                    let buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                    (Timestamp::new(NTP64(0), ID::new(0, buf)), ZFBytes{bytes: Vec::new()})
                } else {
                    return Err(e);
                }
            }
        };


        if vehicle_state_msg.bytes.is_empty() {
            //没有数据
            return Ok(result)
        }  
        let vehicle_state: VehicleKinematicState = bincode::deserialize(&(vehicle_state_msg.bytes)).unwrap();
        
        // 写全局变量
        unsafe {
            let m_odom_frame_id = GLOBAL_VAR.read().unwrap().m_odom_frame_id;
            if m_odom_frame_id.is_empty() {
                let mut global_var = GLOBAL_VAR.write().unwrap();
                global_var.m_odom_frame_id = vehicle_state.header.frame_id;
            }
        }

        // 发送消息
        unsafe {
            let m_planner_ptr = GLOBAL_VAR.read().unwrap().m_planner_ptr;
            let c_vehicle_state = &CVehicleKinematicState::c_repr_of(vehicle_state).unwrap();
            if m_planner_is_replaying(m_planner_ptr) {
                println!("Replaying recorded ego postion as trajectory");
                let traj_raw = m_planner_plan(m_planner_ptr, c_vehicle_state);
                let trajectory = (*traj_raw).as_rust().unwrap();

                record_replay_state = RecordReplayState::REPLAYING;
                remaining_length = trajectory.points.len() as i32;
                result.insert(String::from(LINK_ID_OUTPUT_PLANNED_TRAJECTORY), zf_data!(trajectory));
            }


            if m_planner_reached_goal(m_planner_ptr, c_vehicle_state,
                 GLOBAL_VAR.read().unwrap().m_goal_distance_threshold_m, 
                 GLOBAL_VAR.read().unwrap().m_goal_angle_threshold_rad) {
                    record_replay_state = RecordReplayState::SUCCESS;
                    remaining_length = 0;
                    m_planner_stop_replaying(m_planner_ptr);
            }
        }

        let replay_trajectory_feedback = ReplayTrajectoryFeedback {
            record_replay_state: record_replay_state as i32,
            remaining_length: remaining_length,
        };
        result.insert(String::from(LINK_ID_OUTPUT_REPLAY_TRAJECTORY_FEEDBACK), zf_data!(replay_trajectory_feedback));
        Ok(result)
    }

    fn output_rule(
        _ctx: ZFContext,
        outputs: HashMap<String, Arc<dyn DataTrait>>,
    ) -> OutputRuleOutput {
        let mut zf_outputs: HashMap<String, ZFComponentOutput> = HashMap::with_capacity(1);

        zf_outputs.insert(
            String::from(LINK_ID_OUTPUT_PLANNED_TRAJECTORY),
            ZFComponentOutput::Data(outputs.get(LINK_ID_OUTPUT_PLANNED_TRAJECTORY).unwrap().clone()),
        );

        Ok(zf_outputs)
    }
}

impl OperatorTrait for ReplayPlannerOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(ReplayPlannerOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(ReplayPlannerOperator::run)
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(ReplayPlannerOperator::output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        Box::new(self.config.clone())
    }
}

export_operator!(register);

extern "C" fn register(
    _configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn OperatorTrait + Send>> {
    match _configuration {
        Some(config) => {
            Ok(Box::new(ReplayPlannerOperator::init(config)?) 
            as Box<dyn OperatorTrait + Send>)
        }
        None => Ok(Box::new(ReplayPlannerOperator::init(HashMap::new())?)
            as Box<dyn OperatorTrait + Send>),
    }
}
