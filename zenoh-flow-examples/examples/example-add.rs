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
use std::collections::HashMap;
use zenoh_flow_examples::{ZFString, ZFUsize, ZFBytes, ZFDouble};

use zenoh_flow::{
    downcast, export_operator, get_input,
    runtime::message::ZFMessage,
    types::{
        DataTrait, FnInputRule, FnOutputRule, FnRun, InputRuleResult, OperatorTrait,
        OutputRuleResult, RunResult, StateTrait, ZFInput, ZFResult,
    },
    zf_data, zf_empty_state, Token, ZFContext, ZFError, ZFLinkId,
};

struct AddOperator;

static LINK_ID_INPUT_STR: &str = "Str";

static LINK_ID_OUTPUT_DOUBLE0: &str = "Double0";
static LINK_ID_OUTPUT_DOUBLE1: &str = "Double1";
static LINK_ID_OUTPUT_DOUBLE2: &str = "Double2";

extern {
    fn add_lib_wrapper(a: f64, b: f64) -> f64;
}

impl AddOperator {
    fn input_rule(_ctx: ZFContext, inputs: &mut HashMap<ZFLinkId, Token>) -> InputRuleResult {
        for token in inputs.values() {
            match token {
                Token::Ready(_) => continue,
                Token::NotReady(_) => return Ok(false),
            }
        }

        Ok(true)
    }

    fn run(_ctx: ZFContext, mut inputs: ZFInput) -> RunResult {
        let mut results = HashMap::<ZFLinkId, Arc<dyn DataTrait>>::with_capacity(3);

        let numbers = get_input!(ZFString, String::from(LINK_ID_INPUT_STR), inputs)?;

        let v: Vec<&str> = numbers.0.split(' ').collect();

        let double0: f64 = match v[0].trim().parse() {
            Ok(value0) => value0,
            Err(_) => return Err(ZFError::GenericError),
        };

        let double1: f64 = match v[1].trim().parse() {
            Ok(value1) => value1,
            Err(_) => return Err(ZFError::GenericError),
        };

        let add_result = unsafe { let d = add_lib_wrapper(double0, double1); d };

        results.insert(String::from(LINK_ID_OUTPUT_DOUBLE0), zf_data!(ZFDouble(double0)));
        results.insert(String::from(LINK_ID_OUTPUT_DOUBLE1), zf_data!(ZFDouble(double1)));
        results.insert(String::from(LINK_ID_OUTPUT_DOUBLE2), zf_data!(ZFDouble(add_result)));

        Ok(results)
    }

    fn output_rule(
        _ctx: ZFContext,
        outputs: HashMap<ZFLinkId, Arc<dyn DataTrait>>,
    ) -> OutputRuleResult {
        let mut zf_outputs: HashMap<ZFLinkId, Arc<ZFMessage>> = HashMap::with_capacity(3);

        zf_outputs.insert(
            String::from(LINK_ID_OUTPUT_DOUBLE0),
            Arc::new(ZFMessage::from_data(
                outputs.get(LINK_ID_OUTPUT_DOUBLE0).unwrap().clone(),
            )),
        );
        zf_outputs.insert(
            String::from(LINK_ID_OUTPUT_DOUBLE1),
            Arc::new(ZFMessage::from_data(
                outputs.get(LINK_ID_OUTPUT_DOUBLE1).unwrap().clone(),
            )),
        );
        zf_outputs.insert(
            String::from(LINK_ID_OUTPUT_DOUBLE2),
            Arc::new(ZFMessage::from_data(
                outputs.get(LINK_ID_OUTPUT_DOUBLE2).unwrap().clone(),
            )),
        );

        Ok(zf_outputs)
    }
}

impl OperatorTrait for AddOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(AddOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(AddOperator::run)
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(AddOperator::output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        zf_empty_state!()
    }
}

export_operator!(register);

extern "C" fn register(
    configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::OperatorTrait + Send>> {
    Ok(Box::new(AddOperator) as Box<dyn OperatorTrait + Send>)
}
