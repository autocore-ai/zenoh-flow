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
use cxx::UniquePtr;
use std::collections::HashMap;
use zenoh_flow::{
    export_operator, get_input, downcast,
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
use libc::c_char;

use autoware_auto_msgs::msg::{VehicleKinematicState, Trajectory};
use autoware_auto_msgs::action::{ReplayTrajectoryGoal, ReplayTrajectoryFeedback};

//use ffi_convert::*;

//use autocxx::include_cpp;

//input and output ID
static LINK_ID_INPUT_VEHICLE_STATE: &str = "vehicle_state"; //车辆状态
static LINK_ID_INPUT_REPLAY_TRAJECTORY_GOAL: &str = "replay_trajectory_goal";   //路径规划目标

static LINK_ID_OUTPUT_PLANNED_TRAJECTORY: &str = "planned_trajectory";  //规划好的轨迹
static LINK_ID_OUTPUT_REPLAY_TRAJECTORY_FEEDBACK: &str = "replay_trajectory_feedback";  //当前规划器状态

use ffi::motion::plannner::recordreplay_planner::RecordReplayPlanner;
//全局变量
pub struct ReplayPlannerOperatorGlobalVar {
    is_initialized: bool,
    m_planner_ptr: UniquePtr<RecordReplayPlanner>,
    m_goal_distance_threshold_m: f64, 
    m_goal_angle_threshold_rad: f64, 
    m_odom_frame_id: String,
}
impl ReplayPlannerOperatorGlobalVar {
    pub fn new(ptr: UniquePtr<RecordReplayPlanner>) -> Self {
        ReplayPlannerOperatorGlobalVar{
            is_initialized: false,
            m_planner_ptr: ptr, 
            m_goal_distance_threshold_m: 0.0,
            m_goal_angle_threshold_rad: 0.0,
            m_odom_frame_id: String::new(),
        }
    }

    pub fn get_m_planner(&self) -> &UniquePtr<RecordReplayPlanner> {
        &self.m_planner_ptr
    }

    pub fn set_m_planner(&mut self, m_planner_ptr: UniquePtr<RecordReplayPlanner> ) {
        self.m_planner_ptr = m_planner_ptr;
    }

    pub fn get_m_goal_distance_threshold_m(&self) -> f64 {
        self.m_goal_distance_threshold_m
    }

    pub fn set_m_goal_distance_threshold_m(&mut self, m_goal_distance_threshold_m: f64) {
        self.m_goal_distance_threshold_m = m_goal_distance_threshold_m;
    }

    pub fn get_m_goal_angle_threshold_rad(&self) -> f64 {
        self.m_goal_angle_threshold_rad
    }
    
    pub fn set_m_goal_angle_threshold_rad(&mut self, m_goal_angle_threshold_rad: f64) {
         self.m_goal_angle_threshold_rad = m_goal_angle_threshold_rad;
    }

//     pub fn get_m_odom_frame_id(&mut self) -> String {
//         self.m_odom_frame_id.clone()
//    }

    pub fn set_m_odom_frame_id(&mut self, m_odom_frame_id: String) {
       self.m_odom_frame_id = m_odom_frame_id;
    }
   
    pub fn get_is_initialized(&mut self) -> bool{
        self.is_initialized
    }

    pub fn set_is_initialized(&mut self, is_initialized: bool) {
        self.is_initialized = is_initialized;
    }

}

#[macro_use]
extern crate lazy_static;
extern crate mut_static;
use mut_static::MutStatic;

lazy_static! {
    pub static ref GLOBAL_VAR: MutStatic<ReplayPlannerOperatorGlobalVar> = {
        unsafe{
            MutStatic::from(ReplayPlannerOperatorGlobalVar::new(UniquePtr::<RecordReplayPlanner>::null()))
        }
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
autocxx::include_cpp! {
    #include "replay_planner/replay_planner_wrapper.hpp"
    #include "replay_planner/msgs/autoware_auto_msgs/CComplex32.hpp"
    #include "replay_planner/msgs/autoware_auto_msgs/CTrajectory.hpp"
    #include "replay_planner/msgs/autoware_auto_msgs/CTrajectoryPoint.hpp"
    #include "replay_planner/msgs/autoware_auto_msgs/CVehicleKinematicState.hpp"

    #include "replay_planner/msgs/builtin_interfaces/CDuration.hpp"
    #include "replay_planner/msgs/builtin_interfaces/CTime.hpp"

    #include "replay_planner/msgs/geometry_msgs/CQuaternion.hpp"
    #include "replay_planner/msgs/geometry_msgs/CTransform.hpp"
    #include "replay_planner/msgs/geometry_msgs/CTransformStamped.hpp"
    #include "replay_planner/msgs/geometry_msgs/CVector3.hpp"

    #include "replay_planner/msgs/std_msgs/CHeader.hpp"

    safety!(unsafe_ffi)

    generate!("m_planner_new")
    generate!("m_planner_get_record_replay_state")
    generate!("m_planner_is_replaying")
    generate!("m_planner_start_replaying")
    generate!("m_planner_stop_replaying")
    generate!("m_planner_set_heading_weight")
    generate!("m_planner_set_min_record_distance")
    generate!("m_planner_read_trajectory_buffer_from_file")
    generate!("m_planner_plan")
    generate!("m_planner_reached_goal")
    
    generate_pod!("CComplex32")
    generate_pod!("CTrajectory")
    generate_pod!("CTrajectoryPoint")
    generate_pod!("CVehicleKinematicState")

    generate_pod!("CDuration")
    generate_pod!("CTime")

    generate_pod!("CQuaternion")
    generate_pod!("CTransform")
    generate_pod!("CTransformStamped")
    generate_pod!("CVector3")

    generate_pod!("CHeader")
}

// #[cxx::bridge]
// mod ffi2 {
//     pub struct CVehicleKinematicState {
//         pub header: CHeader,
//         pub state: CTrajectoryPoint,
//         pub delta: CTransform
//     }

//     pub struct CTrajectoryPoint {
//         pub time_from_start: CDuration,
//         pub x: f32,
//         pub y: f32,
//         pub heading: CComplex32,
//         pub longitudinal_velocity_mps: f32,
//         pub lateral_velocity_mps: f32,
//         pub acceleration_mps2: f32,
//         pub heading_rate_rps: f32,
//         pub front_wheel_angle_rad: f32,
//         pub rear_wheel_angle_rad: f32,   
//     }

//     pub struct CComplex32 {
//         pub real: f32,
//         pub imag: f32,
//     }

//     pub struct CTrajectory {
//         pub header: CHeader, 
//         pub points: Vec<CTrajectoryPoint>,
//     }

//     pub struct CReplayTrajectoryFeedback {
//         pub record_replay_state: i32,
//         pub remaining_length: i32,
//     }

//     pub struct CReplayTrajectoryGoal {
//         pub replay_path: String,
//     }

//     pub struct CTime {
//         pub sec: i32,
//         pub nanosec: u32
//     }
//     pub struct CDuration {
//         pub sec: i32,
//         pub nanosec: u32,
//     }

//     pub struct CTransform {
//         pub translation: CVector3,
//         pub rotation: CQuaternion,
//     }
//     pub struct CVector3 {
//         pub x: f64,
//         pub y: f64,
//         pub z: f64,
//     }
    
//     pub struct CQuaternion {
//         pub x: f64,
//         pub y: f64,
//         pub z: f64,
//         pub w: f64,
//     }
    
//     pub struct CTransformStamped {
//         pub header: CHeader,
//         pub child_frame_id: String,
//         pub transform: CTransform,
//     }
    
//     pub struct CHeader {
//         pub stamp: CTime,
//         pub frame_id: String,
//     }

//     unsafe extern "C++" {
//         include!("zenoh-flow-examples/replay_planner/replay_planner_wrapper.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/autoware_auto_msgs/CComplex32.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/autoware_auto_msgs/CTrajectory.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/autoware_auto_msgs/CTrajectoryPoint.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/autoware_auto_msgs/CVehicleKinematicState.hpp");

//         include!("zenoh-flow-examples/replay_planner/msgs/builtin_interfaces/CDuration.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/builtin_interfaces/CTime.hpp");

//         include!("zenoh-flow-examples/replay_planner/msgs/geometry_msgs/CQuaternion.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/geometry_msgs/CTransform.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/geometry_msgs/CTransformStamped.hpp");
//         include!("zenoh-flow-examples/replay_planner/msgs/geometry_msgs/CVector3.hpp");

//         include!("zenoh-flow-examples/replay_planner/msgs/std_msgs/CHeader.hpp");

//         type CComplex32;
//         type CTrajectory;
//         type CTrajectoryPoint;
//         type CVehicleKinematicState;

//         type CDuration;
//         type CTime;
        
//         type CQuaternion;
//         type CTransform;
//         type CTransformStamped;
//         type CVector3;

//         type CHeader;

//         type RecordReplayPlanner;

//         fn m_planner_new() -> UniquePtr<RecordReplayPlanner>;
//         fn m_planner_get_record_replay_state(m_planner_ptr: &UniquePtr<RecordReplayPlanner>) -> i32;
//         fn m_planner_is_replaying(m_planner_ptr: &UniquePtr<RecordReplayPlanner>) -> bool;
//         fn m_planner_start_replaying(m_planner_ptr: &UniquePtr<RecordReplayPlanner>);
//         fn m_planner_stop_replaying(m_planner_ptr: &UniquePtr<RecordReplayPlanner>);

//         fn m_planner_set_heading_weight(m_planner_ptr: &UniquePtr<RecordReplayPlanner>, heading_weight: f64);
//         fn m_planner_set_min_record_distance(m_planner_ptr: &UniquePtr<RecordReplayPlanner>, min_record_distance: f64);

//         fn m_planner_read_trajectory_buffer_from_file(m_planner_ptr: &UniquePtr<RecordReplayPlanner>, replay_path: String);
//         fn m_planner_plan(m_planner_ptr: &UniquePtr<RecordReplayPlanner>, current_state: &CVehicleKinematicState) -> CTrajectory;
//         fn m_planner_reached_goal(m_planner_ptr: &UniquePtr<RecordReplayPlanner>, current_state: &CVehicleKinematicState, distance_thresh: f64, angle_thresh: f64) -> bool;
//     }
// }

// extern {
//     // 创建一个recordreplay_planner指针
//     fn m_planner_new() -> *mut c_void;

//     // 获得recordreplay_planner的状态
//     fn m_planner_get_record_replay_state(m_planner_ptr: *mut c_void) -> i32;

//     fn m_planner_is_replaying(m_planner_ptr: *mut c_void) -> bool;

//     fn m_planner_start_replaying(m_planner_ptr: *mut c_void);

//     fn m_planner_stop_replaying(m_planner_ptr: *mut c_void);

//     //fn m_planner_clear_record(ptr: *mut c_void);

//     //fn m_planner_record_state(ptr: *mut c_void, state_to_record: *mut CVehicleKinematicState) -> bool;

//     //fn m_planner_get_record_length(m_planner: *const c_void) -> i32;

//     fn m_planner_set_heading_weight(m_planner_ptr: *mut c_void, heading_weight: f64);

//     //fn m_planner_get_heading_weight(m_planner: *const c_void) -> f64;

//     fn m_planner_set_min_record_distance(m_planner_ptr: *mut c_void, min_record_distance: f64);

//     //fn m_planner_get_min_record_distance(m_planner: *const c_void) -> f64;

//     fn m_planner_read_trajectory_buffer_from_file(m_planner_ptr: *mut c_void, replay_path: *const c_char );

//     fn m_planner_plan(m_planner_ptr: *mut c_void, current_state: autoware_auto_msgs::msg::CVehicleKinematicState) -> autoware_auto_msgs::msg::CTrajectory;

//     fn m_planner_reached_goal(m_planner_ptr: *mut c_void, current_state: autoware_auto_msgs::msg::CVehicleKinematicState, distance_thresh: f64, angle_thresh: f64) -> bool;
// }


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
        println!("zenoh-flow operator start");
        if  GLOBAL_VAR.read().unwrap().is_initialized == false {
            println!("init start");
            unsafe {
                let guard = _ctx.lock();
                let config = downcast!(Config, guard.state).unwrap();

                let m_planner_ptr = ffi::m_planner_new();

                ffi::m_planner_set_heading_weight(&m_planner_ptr, config.heading_weight);
                ffi::m_planner_set_min_record_distance(&m_planner_ptr, config.min_record_distance);

                let mut global_var_write = GLOBAL_VAR.write().unwrap();
                global_var_write.set_is_initialized(true);
                global_var_write.set_m_planner(m_planner_ptr);
            }
            println!("init end");
        } else {
            println!("init ???");
        }

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

        let mut record_replay_state = RecordReplayState::IDLE as i32;
        let mut remaining_length: i32 = 0;
        if !replay_trajectory_goal.bytes.is_empty() {
            // 收到Record路径消息
            let replay_trajectory_goal: ReplayTrajectoryGoal = bincode::deserialize(&(replay_trajectory_goal.bytes)).unwrap();
            let m_planner_ptr = &GLOBAL_VAR.read().unwrap().m_planner_ptr;
            unsafe {
                record_replay_state = ffi::m_planner_get_record_replay_state(&m_planner_ptr);
            }
            if record_replay_state == RecordReplayState::REPLAYING as i32{
                println!("{}", "Can't start replaying if already are");
            } else {
                // 开始replay
                println!("start replay");
                unsafe {
                    ffi::m_planner_read_trajectory_buffer_from_file(&m_planner_ptr, &replay_trajectory_goal.replay_path);
                }
                unsafe {ffi::m_planner_start_replaying(&m_planner_ptr);}
                record_replay_state = RecordReplayState::REPLAYING as i32
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
            let replay_trajectory_feedback = ReplayTrajectoryFeedback {
                record_replay_state: record_replay_state as i32,
                remaining_length: remaining_length,
            };
    
            let replay_trajectory_feedback_data = ZFBytes {
                bytes: bincode::serialize(&replay_trajectory_feedback).unwrap(),
            };
            result.insert(String::from(LINK_ID_OUTPUT_REPLAY_TRAJECTORY_FEEDBACK), zf_data!(replay_trajectory_feedback_data));
            return Ok(result)
        }  
        let vehicle_state: VehicleKinematicState = bincode::deserialize(&(vehicle_state_msg.bytes)).unwrap();
        //let vehicle_state_copy: VehicleKinematicState = bincode::deserialize(&(vehicle_state_msg.bytes)).unwrap();
        // 写全局变量
        let m_odom_frame_id = GLOBAL_VAR.read().unwrap().m_odom_frame_id.clone();
        if m_odom_frame_id.is_empty() {
            let mut global_var_write = GLOBAL_VAR.write().unwrap();
            global_var_write.set_m_odom_frame_id(vehicle_state.header.frame_id.clone());
        }

        // 发送消息
        unsafe {
            let m_planner_ptr = &GLOBAL_VAR.read().unwrap().m_planner_ptr;

            let c_stamp= ffi::CTime {
                sec: vehicle_state.header.stamp.sec,
                nanosec: vehicle_state.header.stamp.nanosec,
            };
            let c_header = ffi::CHeader{
                stamp: c_stamp,
                frame_id: vehicle_state.header.frame_id,
            };

            let c_vector = ffi::CVector3 {
                x: vehicle_state.delta.translation.x,
                y: vehicle_state.delta.translation.y,
                z: vehicle_state.delta.translation.z,
            };

            let c_quaternion = ffi::CQuaternion {
                x: vehicle_state.delta.rotation.x,
                y: vehicle_state.delta.rotation.y,
                z: vehicle_state.delta.rotation.z,
                w: vehicle_state.delta.rotation.w,
            };
            let c_delta = ffi::CTransform {
                translation: c_vector,
                rotation: c_quaternion,
            };

            let duration = ffi::CDuration {
                sec: vehicle_state.state.time_from_start.sec,
                nanosec: vehicle_state.state.time_from_start.nanosec,
            };

            let complex = ffi::CComplex32 {
                real: vehicle_state.state.heading.real,
                imag: vehicle_state.state.heading.imag,
            };
            
            let c_state = ffi::CTrajectoryPoint {
                time_from_start: duration,
                x: vehicle_state.state.x,
                y: vehicle_state.state.y,
                heading: complex,
                longitudinal_velocity_mps: vehicle_state.state.longitudinal_velocity_mps,
                lateral_velocity_mps: vehicle_state.state.lateral_velocity_mps,
                acceleration_mps2: vehicle_state.state.acceleration_mps2,
                heading_rate_rps: vehicle_state.state.heading_rate_rps,
                front_wheel_angle_rad: vehicle_state.state.front_wheel_angle_rad,
                rear_wheel_angle_rad: vehicle_state.state.rear_wheel_angle_rad,
            };
            let c_vehicle_state = ffi::CVehicleKinematicState { 
                header: c_header,
                state: c_state,
                delta: c_delta,
            };


            // let ctime = crate::ffi::CTime::make_unique1(vehicle_state.header.stamp.sec, vehicle_state.header.stamp.nanosec);
            // let header = crate::ffi::CHeader::make_unique1(ctime, vehicle_state.header.frame_id);
            // let cduration = crate::ffi::CDuration::make_unique1(vehicle_state.state.time_from_start.sec, vehicle_state.state.time_from_start.nanosec);
            // let ccomplex32 = crate::ffi::CComplex32::make_unique1(vehicle_state.state.heading.real, vehicle_state.state.heading.imag);
            // let state = crate::ffi::CTrajectoryPoint::make_unique1(cduration, vehicle_state.state.x, vehicle_state.state.y, 
            //     ccomplex32, vehicle_state.state.longitudinal_velocity_mps, vehicle_state.state.lateral_velocity_mps, vehicle_state.state.acceleration_mps2, 
            //     vehicle_state.state.heading_rate_rps, vehicle_state.state.front_wheel_angle_rad, vehicle_state.state.rear_wheel_angle_rad);
            // let translation = crate::ffi::CVector3::make_unique1(vehicle_state.delta.translation.x, vehicle_state.delta.translation.y, vehicle_state.delta.translation.z);
            // let rotation = crate::ffi::CQuaternion::make_unique1(vehicle_state.delta.rotation.x, vehicle_state.delta.rotation.y, vehicle_state.delta.rotation.z, vehicle_state.delta.rotation.w);
            // let delta = crate::ffi::CTransform::make_unique1(translation, rotation);
            // let c_vehicle_state = crate::ffi::CVehicleKinematicState::make_unique1(header, state, delta);

            if ffi::m_planner_is_replaying(&m_planner_ptr) {
                println!("Replaying recorded ego postion as trajectory");
            
                let traj_raw = ffi::m_planner_plan(&m_planner_ptr, &c_vehicle_state);

                //println!("received trajectory {}", traj_raw.points.len());
                //let frame_id = crate::ffi::CHeader::get_frame_id(crate::ffi::CTrajectory::get_header(traj_raw.pin_mut()).pin_mut());
                let frame_id = &traj_raw.header.frame_id;
                //let c_str: &CStr = unsafe { CStr::from_ptr(traj_raw.header.frame_id) };
                println!("received frame id: {}", frame_id);
                
                let time_msg = builtin_interfaces::msg::Time {
                    sec: traj_raw.header.stamp.sec,
                    nanosec: traj_raw.header.stamp.nanosec,
                };
                let header_msg = std_msgs::msg::Header {
                    stamp: time_msg,
                    frame_id: traj_raw.header.frame_id,
                };

                let mut trajectory_points = Vec::new();
                for index in 0..traj_raw.points.len() {
                    let trajectory_point_raw = traj_raw.points.get(index).unwrap();
                    let duration = builtin_interfaces::msg::Duration {
                        sec: trajectory_point_raw.time_from_start.sec,
                        nanosec: trajectory_point_raw.time_from_start.nanosec,
                    };

                    let complex = autoware_auto_msgs::msg::Complex32 {
                        real: trajectory_point_raw.heading.real,
                        imag: trajectory_point_raw.heading.imag,
                    };

                    let trajectory_point = autoware_auto_msgs::msg::TrajectoryPoint {
                        time_from_start: duration,
                        x: trajectory_point_raw.x,
                        y: trajectory_point_raw.y,
                        heading: complex,
                        longitudinal_velocity_mps: trajectory_point_raw.longitudinal_velocity_mps,
                        lateral_velocity_mps: trajectory_point_raw.lateral_velocity_mps,
                        acceleration_mps2: trajectory_point_raw.acceleration_mps2,
                        heading_rate_rps: trajectory_point_raw.heading_rate_rps,
                        front_wheel_angle_rad: trajectory_point_raw.front_wheel_angle_rad,
                        rear_wheel_angle_rad: trajectory_point_raw.rear_wheel_angle_rad,
                    };

                    trajectory_points.push(trajectory_point);
                }

               // println!("trajectory number: {}", traj_raw.points.size);

            //    let traj_raw_header = crate::ffi::CTrajectory::get_header(traj_raw.pin_mut());
            //    let traj_raw_points = crate::ffi::CTrajectory::get_points(traj_raw.pin_mut());
            //    let trajectory_points = Vec::new();
               

            //    for index in 0..traj_raw_points.len() {
            //        let ctrajectory_point = crate::ffi::CTrajectory::get_point(traj_raw.pin_mut(), index as i32);
            //        let time_from_start_cpp = crate::ffi::CTrajectoryPoint::get_time_from_start(ctrajectory_point.pin_mut());
            //        let x = crate::ffi::CTrajectoryPoint::get_x(ctrajectory_point.pin_mut());
            //        let y = crate::ffi::CTrajectoryPoint::get_y(ctrajectory_point.pin_mut());
            //        let heading = crate::ffi::CTrajectoryPoint::get_heading(ctrajectory_point.pin_mut());
            //        let longitudinal_velocity_mps = crate::ffi::CTrajectoryPoint::get_longitudinal_velocity_mps(ctrajectory_point.pin_mut());
            //        let lateral_velocity_mps = crate::ffi::CTrajectoryPoint::get_lateral_velocity_mps(ctrajectory_point.pin_mut());
            //        let acceleration_mps2 = crate::ffi::CTrajectoryPoint::get_acceleration_mps2(ctrajectory_point.pin_mut());
            //        let heading_rate_rps = crate::ffi::CTrajectoryPoint::get_heading_rate_rps(ctrajectory_point.pin_mut());
            //        let front_wheel_angle_rad = crate::ffi::CTrajectoryPoint::get_front_wheel_angle_rad(ctrajectory_point.pin_mut());
            //        let rear_wheel_angle_rad = crate::ffi::CTrajectoryPoint::get_rear_wheel_angle_rad(ctrajectory_point.pin_mut());
            //        let heading_real = crate::ffi::CComplex32::get_real(heading.pin_mut());
            //        let heading_imag = crate::ffi::CComplex32::get_imag(heading.pin_mut());
                

            //        let sec = crate::ffi::CDuration::get_sec(time_from_start_cpp.pin_mut());
            //        let nanosec = crate::ffi::CDuration::get_nanosec(time_from_start_cpp.pin_mut());

            //        let heading = autoware_auto_msgs::msg::Complex32 {
            //             real: heading_real,
            //             imag: heading_imag,
            //         };

            //         let time_from_start = builtin_interfaces::msg::Duration {
            //             sec,
            //             nanosec,
            //         };

            //        let trajectory_point_rust = autoware_auto_msgs::msg::TrajectoryPoint {
            //            time_from_start,
            //            x,
            //            y,
            //            heading,
            //            longitudinal_velocity_mps,
            //            lateral_velocity_mps,
            //            acceleration_mps2,
            //            heading_rate_rps,
            //            front_wheel_angle_rad,
            //            rear_wheel_angle_rad,
            //        };

            //        trajectory_points.push(trajectory_point_rust);
            //     }

                // let header_stamp = crate::ffi::CHeader::get_stamp(header.pin_mut());
                // let stamp_sec = crate::ffi::CTime::get_sec(header_stamp.pin_mut());
                // let stamp_nanosec = crate::ffi::CTime::get_nanosec(header_stamp.pin_mut());

                // let frame_id_cpp =  crate::ffi::CHeader::get_frame_id(header.pin_mut());
                // let time = builtin_interfaces::msg::Time {
                //     sec: stamp_sec,
                //     nanosec: stamp_nanosec,
                // };

                // let header_rust = std_msgs::msg::Header {
                //     stamp: time,
                //     frame_id: frame_id_cpp.to_string(),
                // };

                let trajectory = autoware_auto_msgs::msg::Trajectory {
                    header: header_msg,
                    points: trajectory_points,
                };

                println!("end trajectory: {}", trajectory.header.frame_id);

                record_replay_state = RecordReplayState::REPLAYING as i32;
                remaining_length = trajectory.points.len() as i32;

                let trajectory_data = ZFBytes {
                    bytes: bincode::serialize(&trajectory).unwrap(),
                };
                result.insert(String::from(LINK_ID_OUTPUT_PLANNED_TRAJECTORY), zf_data!(trajectory_data));
            }

            if ffi::m_planner_reached_goal(&m_planner_ptr, &c_vehicle_state,
                 GLOBAL_VAR.read().unwrap().m_goal_distance_threshold_m, 
                 GLOBAL_VAR.read().unwrap().m_goal_angle_threshold_rad) == true {
                    record_replay_state = RecordReplayState::SUCCESS as i32;
                    remaining_length = 0;
                    ffi::m_planner_stop_replaying(&m_planner_ptr);
            }
        }

        let replay_trajectory_feedback = ReplayTrajectoryFeedback {
            record_replay_state: record_replay_state as i32,
            remaining_length: remaining_length,
        };

        let replay_trajectory_feedback_data = ZFBytes {
            bytes: bincode::serialize(&replay_trajectory_feedback).unwrap(),
        };
        result.insert(String::from(LINK_ID_OUTPUT_REPLAY_TRAJECTORY_FEEDBACK), zf_data!(replay_trajectory_feedback_data));
        println!("zenoh-flow operator end");
        Ok(result)
    }

    // fn output_rule(
    //     _ctx: ZFContext,
    //     outputs: HashMap<String, Arc<dyn DataTrait>>,
    // ) -> OutputRuleOutput {
    //     let mut zf_outputs: HashMap<String, ZFComponentOutput> = HashMap::with_capacity(1);

    //     zf_outputs.insert(
    //         String::from(LINK_ID_OUTPUT_PLANNED_TRAJECTORY),
    //         ZFComponentOutput::Data(outputs.get(LINK_ID_OUTPUT_PLANNED_TRAJECTORY).unwrap().clone()),
    //     );

    //     Ok(zf_outputs)
    // }
}

impl OperatorTrait for ReplayPlannerOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(ReplayPlannerOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(ReplayPlannerOperator::run)
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(zenoh_flow::default_output_rule)
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
