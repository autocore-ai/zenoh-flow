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

use std::collections::HashMap;
use zenoh_flow::{
    serde::{Deserialize, Serialize},
    types::{
        FnInputRule, FnSinkRun, FutSinkResult, InputRuleResult, SinkTrait, StateTrait, Token,
        ZFContext, ZFInput, ZFLinkId,
    },
    zenoh_flow_derive::ZFState,
    zf_empty_state, ZFResult,
};

#[derive(Serialize, Deserialize, Debug, ZFState)]
struct GenericDisp {}

#[macro_use]
extern crate lazy_static;
extern crate mut_static;

pub struct MyStruct { value: usize }

impl MyStruct {
    pub fn new(value: usize) -> Self {
        MyStruct{ value: value }
    }

    pub fn get_value(&self) -> usize {
        self.value
    }

    pub fn set_value(&mut self, value: usize) {
        self.value = value
    }
}

lazy_static! {
    pub static ref LAST_TIME: MutStatic<MyStruct> = {
        MutStatic::from(MyStruct::new(0))
    };
}
use mut_static::MutStatic;

impl GenericDisp {
    pub fn ir_1(_ctx: ZFContext, _inputs: &mut HashMap<ZFLinkId, Token>) -> InputRuleResult {
        println!("ir_1");
        Ok(true)
    }

    pub async fn run_1(_ctx: ZFContext, inputs: ZFInput) -> ZFResult<()> {
        println!("run_1");
        println!("#######");
        for (k, v) in inputs.into_iter() {
            println!("Calculate Result Received on LinkId {:?} -> {:?}", k, v);
        }
        println!("#######");

        println!("run_1.5");
        {
            let last_time_temp = LAST_TIME.read().unwrap().get_value();
            println!("last_time = {}", last_time_temp);
            let mut last_time_mod = LAST_TIME.write().unwrap();
            last_time_mod.set_value(last_time_temp + 1);
        }
        println!("run_2");
        Ok(())
    }
}

impl SinkTrait for GenericDisp {
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
    Ok(Box::new(GenericDisp {}) as Box<dyn zenoh_flow::SinkTrait + Send>)
}
