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

use std::{collections::HashMap, sync::Arc, usize};

use serde_json::{Value};

use zenoh_flow::runtime::runner::ZFSourceRegistrarTrait;
use zenoh_flow::{
    types::{
        DataTrait, FnOutputRule, FnSourceRun, FutRunResult, RunResult, SourceTrait, StateTrait,
    },
    zf_data, ZFContext, ZFError, ZFLinkId, ZFResult,downcast,serde::{Deserialize, Serialize},zenoh_flow_derive::ZFState,
};
use zenoh_flow_examples::ZFString;


extern crate reqwest;

use std::fs::File;
use std::io::prelude::*;
use regex::Regex;

struct Lanelet2MapInput{
    pub config: MapConfig,
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

static LINK_ID_INPUT_STR: &str = "Config";

impl Lanelet2MapInput {
    fn init(configuration: HashMap<String, String>) -> ZFResult<Self> {
        let map_osm_file: String = match configuration.get("map_osm_file") {
            Some(map_osm_file) => map_osm_file.to_string(),
            None =>  return Err(ZFError::GenericError),
        };

        let origin_offset_lat: f64 = match configuration.get("origin_offset_lat") {
            Some(origin_offset_lat) => origin_offset_lat.trim().parse().unwrap(),
            None => 0.0,
        };

        let origin_offset_lon: f64 = match configuration.get("origin_offset_lon") {
            Some(origin_offset_lon) => origin_offset_lon.trim().parse().unwrap(),
            None => 0.0,
        };

        let origin_lat: f64 = match configuration.get("latitude") {
            Some(origin_lat) => origin_lat.trim().parse().unwrap(),
            None => return Err(ZFError::GenericError),
        };

        let origin_lon: f64 = match configuration.get("longitude") {
            Some(origin_lon) => origin_lon.trim().parse().unwrap(),
            None => return Err(ZFError::GenericError),
        };

        let origin_alt: f64 = match configuration.get("elevation") {
            Some(origin_alt) => origin_alt.trim().parse().unwrap(),
            None => return Err(ZFError::GenericError),
        };

        let config = MapConfig{
            map_osm_file,
            origin_offset_lat,
            origin_offset_lon,
            origin_lat,
            origin_lon,
            origin_alt,
        };

        Ok(Self {config})
    }

    async fn run(_ctx: ZFContext) -> RunResult {
        let mut results: HashMap<ZFLinkId, Arc<dyn DataTrait>> = HashMap::with_capacity(2);

        println!("> Please input number to start map load: ");
        let mut numbers = String::new();
        async_std::io::stdin()
            .read_line(&mut numbers)
            .await
            .expect("Could not read number.");

        let guard = _ctx.lock();

        let config = downcast!(MapConfig, guard.state).unwrap(); //downcasting to right type

        //下载地图
        let resp = reqwest::blocking::get("http://192.168.10.43:8000/sdv/2/map/lanelet2_map.osm").unwrap();
        if resp.status().is_success() {
            println!("success!");
            let mut file = File::create("/home/shenmintao/lanelet2_map.osm")?;
            let mut text = resp.text().unwrap();

            //删除奇怪的东西，变成一个XML字符串
            let start_index = text.find("<?xml version").unwrap();
            let end_index = text.find("</osm>").unwrap();

            println!("start: {} end: {}",start_index, end_index);
            let xml_string = &text[start_index..end_index + 6];

            file.write_all(xml_string.as_bytes());
        } else if resp.status().is_server_error() {
            println!("server error!");
        } else {
            println!("Something else happened. Status: {:?}", resp.status());
        }
        

        let map_config = MapConfig{
            map_osm_file: "/home/shenmintao/lanelet2_map.osm".to_string(),
            origin_offset_lat: config.origin_offset_lat,
            origin_offset_lon: config.origin_offset_lon,
            origin_lat: config.origin_lat,
            origin_lon: config.origin_lon,
            origin_alt: config.origin_alt,
        };
        
        let config_serialized = serde_json::to_string(&map_config).unwrap();
        println!("config is {:?}", config_serialized);

        results.insert(String::from(LINK_ID_INPUT_STR), zf_data!(ZFString(config_serialized)));

        Ok(results)
    }
}

impl SourceTrait for Lanelet2MapInput {
    fn get_run(&self, _ctx: ZFContext) -> FnSourceRun {
        Box::new(|ctx: ZFContext| -> FutRunResult { Box::pin(Self::run(ctx)) })
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(zenoh_flow::default_output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        Box::new(self.config.clone())
    }
}

zenoh_flow::export_source!(register);

extern "C" fn register(
    configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::SourceTrait + Send>> {
    match configuration {
        Some(config) => {
            Ok(Box::new(Lanelet2MapInput::init(config)?) as Box<dyn zenoh_flow::SourceTrait + Send>)
        }
        None => Ok(Box::new(Lanelet2MapInput::init(HashMap::new())?)
            as Box<dyn zenoh_flow::SourceTrait + Send>),
    }
}
