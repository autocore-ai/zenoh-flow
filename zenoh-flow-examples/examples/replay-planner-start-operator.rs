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

use std::{collections::HashMap, sync::Arc, usize};
use std::io::{self, Read};

use zenoh_flow::{
    export_operator, get_input,
    types::{
        DataTrait, FnInputRule, FnOutputRule, FnRun, InputRuleOutput, OperatorTrait,
        OutputRuleOutput, RunOutput, StateTrait, ZFInput, ZFResult, ZFError
    },
    zf_data, zf_empty_state, ZFContext,Token, 
};
use zenoh_flow_examples::{ZFUsize, ZFBytes, ZFString};

use std::error::Error;
use csv::Reader;
use serde::Deserialize;

use autoware_auto_msgs::action::ReplayTrajectoryGoal;
use autoware_auto_msgs::msg::VehicleKinematicState;
use std_msgs::msg::Header;
use autoware_auto_msgs::msg::TrajectoryPoint;
use geometry_msgs::msg::Transform;
use autoware_auto_msgs::msg::Complex32;
use geometry_msgs::msg::Vector3;
use geometry_msgs::msg::Quaternion;

#[derive(Deserialize)]
pub struct Record {
    t_sec: i32,
    t_nanosec: u32,
    x: f32,
    y: f32, 
    heading_real: f32, 
    heading_imag: f32, 
    longitudinal_velocity_mps: f32,
    lateral_velocity_mps: f32,
    acceleration_mps2: f32, 
    heading_rate_rps: f32, 
    front_wheel_angle_rad: f32, 
    rear_wheel_angle_rad: f32,
}

//全局变量
pub struct ReplayPlannerSourceGlobalVar {
    is_initialized: bool,
    record_vec: Vec<Record>,
    record_index: usize,
}


impl ReplayPlannerSourceGlobalVar {
    pub fn new() -> Self {
        ReplayPlannerSourceGlobalVar{
            is_initialized: false,
            record_vec: Vec::new(),
            record_index: 0,
        }
    }

    pub fn set_is_initialized(&mut self, is_initialized: bool) {
        self.is_initialized = is_initialized;
    }
    
    pub fn get_is_initialized(&self) -> bool {
        self.is_initialized
    }
    
    pub fn set_record_vec(&mut self, record_vec: Vec<Record>) {
        self.record_vec = record_vec;
    }
    
    pub fn get_record(&self, index: usize) -> &Record {
        self.record_vec.get(index).unwrap()
    }

    pub fn set_record_index(&mut self, record_index: usize) {
        self.record_index = record_index;
    }
    
    pub fn get_record_index(&self) -> usize {
        self.record_index
    }
}


#[macro_use]
extern crate lazy_static;
extern crate mut_static;
use mut_static::MutStatic;

lazy_static! {
    pub static ref GLOBAL_VAR: MutStatic<ReplayPlannerSourceGlobalVar> = {
        unsafe{
            MutStatic::from(ReplayPlannerSourceGlobalVar::new())
        }
    };
}

unsafe impl Send for ReplayPlannerSourceGlobalVar{}
unsafe impl Sync for ReplayPlannerSourceGlobalVar{}

struct ReplayPlannerStartOperator;

static LINK_ID_INPUT_COMMAND: &str = "input"; //启动指令
static LINK_ID_OUTPUT_VEHICLE_STATE: &str = "vehicle_state"; //车辆状态
static LINK_ID_OUTPUT_REPLAY_TRAJECTORY_GOAL: &str = "replay_trajectory_goal";   //路径规划目标

impl ReplayPlannerStartOperator {

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
        let mut results: HashMap<String, Arc<dyn DataTrait>> = HashMap::with_capacity(1);

        let (_, input) = get_input!(ZFString, String::from(LINK_ID_INPUT_COMMAND), inputs)?;

        // 刚启动，没开始
        let is_initialized = GLOBAL_VAR.read().unwrap().get_is_initialized();

        if is_initialized == false {
            let replay_trajectory_gaol = ReplayTrajectoryGoal {
                replay_path: "path".to_string(),
            };
            let replay_trajectory_gaol_data = ZFBytes {
                bytes: bincode::serialize(&replay_trajectory_gaol).unwrap(),
            };
            results.insert(String::from(LINK_ID_OUTPUT_REPLAY_TRAJECTORY_GOAL), zf_data!(replay_trajectory_gaol_data));
            //设为已启动
            GLOBAL_VAR.write().unwrap().set_is_initialized(true);
            //读取CSV文件并且放入全局变量
            let mut reader = Reader::from_path("/home/shenmintao/adehome/AutowareAuto/install/autoware_demos/share/autoware_demos/data/autonomoustuff_parking_lot_lgsvl.forward.path").unwrap();

            let mut vec: Vec<Record> = Vec::new();
            for result in reader.records() {
                let record: Record = result.unwrap().deserialize(None).unwrap();
                vec.push(record);
            }
            let mut global_var_write = GLOBAL_VAR.write().unwrap();
            global_var_write.set_record_vec(vec);

            let replay_trajectory_goal = ReplayTrajectoryGoal {
                replay_path: "/home/shenmintao/adehome/AutowareAuto/install/autoware_demos/share/autoware_demos/data/autonomoustuff_parking_lot_lgsvl.forward.path".to_string(),
            };

            let replay_trajectory_goal_data = ZFBytes {
                bytes: bincode::serialize(&replay_trajectory_goal).unwrap(),
            };

            results.insert(String::from(LINK_ID_OUTPUT_REPLAY_TRAJECTORY_GOAL), zf_data!(replay_trajectory_goal_data));
        
        } else {
            // 已启动，发送车辆当前位置
            let record_index = GLOBAL_VAR.read().unwrap().get_record_index();
            let time = builtin_interfaces::msg::Time {
                sec: 0,
                nanosec: 0
            };
            let header = Header{
                stamp: time, 
                frame_id: "vehicle_state".to_string(),
            };
            
            let state = TrajectoryPoint {
                time_from_start: builtin_interfaces::msg::Duration {
                    sec:  GLOBAL_VAR.read().unwrap().get_record(record_index).t_sec,
                    nanosec:  GLOBAL_VAR.read().unwrap().get_record(record_index).t_nanosec,
                },
                x:  GLOBAL_VAR.read().unwrap().get_record(record_index).x,
                y:  GLOBAL_VAR.read().unwrap().get_record(record_index).y,
                heading: Complex32{
                    real:  GLOBAL_VAR.read().unwrap().get_record(record_index).heading_real,
                    imag:  GLOBAL_VAR.read().unwrap().get_record(record_index).heading_imag,
                },
                longitudinal_velocity_mps: GLOBAL_VAR.read().unwrap().get_record(record_index).longitudinal_velocity_mps,
                lateral_velocity_mps:  GLOBAL_VAR.read().unwrap().get_record(record_index).lateral_velocity_mps,
                acceleration_mps2:  GLOBAL_VAR.read().unwrap().get_record(record_index).acceleration_mps2,
                heading_rate_rps:  GLOBAL_VAR.read().unwrap().get_record(record_index).heading_rate_rps,
                front_wheel_angle_rad:  GLOBAL_VAR.read().unwrap().get_record(record_index).front_wheel_angle_rad,
                rear_wheel_angle_rad:  GLOBAL_VAR.read().unwrap().get_record(record_index).rear_wheel_angle_rad,
            };

            let delta = Transform {
                translation: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 0.0,
                }
            };

            let vehicle_state = VehicleKinematicState {
                header: header,
                state: state,
                delta: delta,
            };

            let vehicle_statey_data = ZFBytes {
                bytes: bincode::serialize(&vehicle_state).unwrap(),
            };

            results.insert(String::from(LINK_ID_OUTPUT_VEHICLE_STATE), zf_data!(vehicle_statey_data));
            GLOBAL_VAR.write().unwrap().set_record_index(record_index + 1);
        }
        //results.insert(String::from(LINK_ID_OUTPUT_REPLAY_TRAJECTORY_GOAL), zf_data!(ZFUsize(value)));
        Ok(results)
    }
}

impl OperatorTrait for ReplayPlannerStartOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(ReplayPlannerStartOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(ReplayPlannerStartOperator::run)
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
    Ok(Box::new(ReplayPlannerStartOperator {}) as Box<dyn zenoh_flow::OperatorTrait + Send>)
}
