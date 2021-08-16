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

use async_std::sync::Arc;

use std::{thread, time};
use std::ffi::CString;
use std::os::raw::c_char;

use std::collections::HashMap;
use zenoh_flow::{
    get_input,
    serde::{Deserialize, Serialize},
    types::{
        DataTrait,
        FnInputRule, FnSinkRun, FutSinkResult, InputRuleResult, SinkTrait, StateTrait, Token,
        ZFContext, ZFInput, ZFLinkId,
    },
    zenoh_flow_derive::ZFState,
    zf_empty_state, ZFResult,
};
use zenoh_flow_examples::ZFString;

#[derive(Serialize, Deserialize, Debug, ZFState)]
struct Lanelet2MapOutput {}


extern {
    fn start_maploader_service(map_osm_file_ptr: *const c_char, origin_offset_lat: f64, origin_offset_lon: f64, latitude: f64, 
        longitude: f64, elevation: f64);
    fn stop_maploader_service();
}


#[macro_use]
extern crate lazy_static;
extern crate mut_static;

pub struct IsServiceStarted { value: usize }

impl IsServiceStarted {
    pub fn new(value: usize) -> Self {
        IsServiceStarted{ value: value }
    }

    pub fn get_value(&self) -> usize {
        self.value
    }

    pub fn set_value(&mut self, value: usize) {
        self.value = value
    }
}

#[derive(Serialize, Deserialize, ZFState, Clone)]
struct MapConfig {
    pub map_osm_file: String,
    pub origin_offset_lat: f64,
    pub origin_offset_lon: f64,
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub origin_alt: f64,
}

impl std::fmt::Debug for MapConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "MapConfig: map_osm_file:{:?} origin_offset_lat:{:?} origin_offset_lon:{:?} latitude:{:?} longitude:{:?} elevation:{:?}",
            self.map_osm_file, self.origin_offset_lat, self.origin_offset_lon, self.origin_lat, self.origin_lon, self.origin_alt
        )
    }
}

lazy_static! {
    pub static ref IS_SERVICE_STARTED: MutStatic<IsServiceStarted> = {
        MutStatic::from(IsServiceStarted::new(0))
    };
}
use mut_static::MutStatic;

static LINK_ID_INPUT_STR: &str = "Config";

impl Lanelet2MapOutput {
    pub fn ir_1(_ctx: ZFContext, _inputs: &mut HashMap<ZFLinkId, Token>) -> InputRuleResult {
        Ok(true)
    }

    pub async fn run_1(_ctx: ZFContext, mut inputs: ZFInput) -> ZFResult<()> {
        let config_serialized = get_input!(ZFString, String::from(LINK_ID_INPUT_STR), inputs)?;

        let config: MapConfig = serde_json::from_str(&(config_serialized.0)).unwrap();
        let is_service_started = IS_SERVICE_STARTED.read().unwrap().get_value();
        if(is_service_started == 0) {
            unsafe {
                let map_osm_file = CString::new(config.map_osm_file).expect("CString::new failed");
                
                start_maploader_service(map_osm_file.as_ptr(), config.origin_offset_lat, config.origin_offset_lon, 
                    config.origin_lat, config.origin_lon, config.origin_alt);
                let mut is_service_started_modify = IS_SERVICE_STARTED.write().unwrap();
                is_service_started_modify.set_value(1);
            }
            println!("Service Started!");
        } else {
            println!("Service Closing!");
            unsafe {
                stop_maploader_service();
                let sleep_time = time::Duration::from_millis(1000);
                thread::sleep(sleep_time);

                println!("Service Restarting!");
                let map_osm_file = CString::new(config.map_osm_file).expect("CString::new failed");
                start_maploader_service(map_osm_file.as_ptr(), config.origin_offset_lat, config.origin_offset_lon, 
                    config.origin_lat, config.origin_lon, config.origin_alt);
                thread::sleep(sleep_time);
            }
            println!("Service Started!");
        }
        Ok(())
    }
}

impl SinkTrait for Lanelet2MapOutput {
    fn get_input_rule(&self, ctx: ZFContext) -> Box<FnInputRule> {
        println!("get_input_rule_1");
        let gctx = ctx.lock();
        match gctx.mode {
            0 => Box::new(Self::ir_1),
            _ => panic!("No way"),
        }
    }

    fn get_run(&self, ctx: ZFContext) -> FnSinkRun {
        println!("get_run_1");
        let gctx = ctx.lock();
        match gctx.mode {
            0 => Box::new(|ctx: ZFContext, inputs: ZFInput| -> FutSinkResult {
                Box::pin(Self::run_1(ctx, inputs))
            }),
            _ => panic!("No way"),
        }
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        zf_empty_state!()
    }
}

zenoh_flow::export_sink!(register);

extern "C" fn register(
    _configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::SinkTrait + Send>> {
    println!("ZFResult_1");
    Ok(Box::new(Lanelet2MapOutput {}) as Box<dyn zenoh_flow::SinkTrait + Send>)
}
