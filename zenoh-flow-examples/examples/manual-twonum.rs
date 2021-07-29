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

use zenoh_flow::runtime::runner::ZFSourceRegistrarTrait;
use zenoh_flow::{
    types::{
        DataTrait, FnOutputRule, FnSourceRun, FutRunResult, RunResult, SourceTrait, StateTrait,
    },
    zf_data, zf_empty_state, ZFContext, ZFError, ZFLinkId, ZFResult,downcast,serde::{Deserialize, Serialize},zenoh_flow_derive::ZFState,
};
use zenoh_flow_examples::ZFString;

struct ManualSource{
    pub state: ExtraNumState,
}

#[derive(Serialize, Deserialize, ZFState, Clone)]
struct ExtraNumState {
    pub extra_num1: f64,
    pub extra_num2: f64,
}


impl std::fmt::Debug for ExtraNumState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "ExtraNumState: extra_num1:{:?} extra_num2:{:?}",
            self.extra_num1, self.extra_num2
        )
    }
}

static LINK_ID_INPUT_STR: &str = "Str";

impl ManualSource {
    fn init(configuration: HashMap<String, String>) -> ZFResult<Self> {
        let extra_num1: f64 = match configuration.get("extra-num1") {
            Some(num1) => num1.trim().parse().unwrap(),
            None => 0.0,
        };

        let extra_num2: f64 = match configuration.get("extra-num2") {
            Some(num2) => num2.trim().parse().unwrap(),
            None => 0.0,
        };

        let state = ExtraNumState{
            extra_num1,
            extra_num2,
        };
        Ok(Self {state})
    }

    async fn run(_ctx: ZFContext) -> RunResult {
        let mut results: HashMap<ZFLinkId, Arc<dyn DataTrait>> = HashMap::with_capacity(2);

        println!("> Please input two numbers with space: ");
        let mut numbers = String::new();
        async_std::io::stdin()
            .read_line(&mut numbers)
            .await
            .expect("Could not read numbers.");

        
        let v: Vec<&str> = numbers.split(' ').collect();
    
        match v.len() {
            2 => println!("received numbers are: {}, {}", v[0], v[1]),
            _ => panic!("could not read two numbers."),
        }

        let guard = _ctx.lock();
        let _state = downcast!(ExtraNumState, guard.state).unwrap(); //downcasting to right type

        let _extra_num1 = _state.extra_num1;
        let _extra_num2 = _state.extra_num2;

        println!("extra config numbers are: {}, {}", _extra_num1, _extra_num2);
        
        let value0: f64 = match v[0].trim().parse() {
            Ok(value0) => value0,
            Err(_) => return Err(ZFError::GenericError),
        };

        let value1: f64 = match v[1].trim().parse() {
            Ok(value1) => value1,
            Err(_) => return Err(ZFError::GenericError),
        };

        let new_numbers = format!("{} {}", value0 + _extra_num1, value1 + _extra_num2);

        results.insert(String::from(LINK_ID_INPUT_STR), zf_data!(ZFString(new_numbers)));

        Ok(results)
    }
}

impl SourceTrait for ManualSource {
    fn get_run(&self, _ctx: ZFContext) -> FnSourceRun {
        Box::new(|ctx: ZFContext| -> FutRunResult { Box::pin(Self::run(ctx)) })
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(zenoh_flow::default_output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        Box::new(self.state.clone())
    }
}

zenoh_flow::export_source!(register);

extern "C" fn register(
    configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::SourceTrait + Send>> {
    match configuration {
        Some(config) => {
            Ok(Box::new(ManualSource::init(config)?) as Box<dyn zenoh_flow::SourceTrait + Send>)
        }
        None => Ok(Box::new(ManualSource::init(HashMap::new())?)
            as Box<dyn zenoh_flow::SourceTrait + Send>),
    }
}
