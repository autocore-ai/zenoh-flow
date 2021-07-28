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

struct MulOperator;

static LINK_ID_INPUT_DOUBLE0: &str = "Double0";
static LINK_ID_INPUT_DOUBLE1: &str = "Double1";
static LINK_ID_INPUT_DOUBLE2: &str = "Double2";

static LINK_ID_OUTPUT_RESULT: &str = "Str";

extern {
    fn mul_lib_wrapper(a: f64, b: f64) -> f64;
}

impl MulOperator {
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

        let double0 = get_input!(ZFDouble, String::from(LINK_ID_INPUT_DOUBLE0), inputs)?.clone();
        let double1 = get_input!(ZFDouble, String::from(LINK_ID_INPUT_DOUBLE1), inputs)?.clone();
        let add_result = get_input!(ZFDouble, String::from(LINK_ID_INPUT_DOUBLE2), inputs)?.0;

        let mul_result = unsafe { let d = mul_lib_wrapper(double0.0, double1.0); d };

        let result_string = format!("add_result:{}, mul_result:{}", add_result, mul_result);

        results.insert(String::from(LINK_ID_OUTPUT_RESULT), zf_data!(ZFString(result_string)));
    

        Ok(results)
    }

    fn output_rule(
        _ctx: ZFContext,
        outputs: HashMap<ZFLinkId, Arc<dyn DataTrait>>,
    ) -> OutputRuleResult {
        let mut zf_outputs: HashMap<ZFLinkId, Arc<ZFMessage>> = HashMap::with_capacity(2);

        zf_outputs.insert(
            String::from(LINK_ID_OUTPUT_RESULT),
            Arc::new(ZFMessage::from_data(
                outputs.get(LINK_ID_OUTPUT_RESULT).unwrap().clone(),
            )),
        );
        Ok(zf_outputs)
    }
}

impl OperatorTrait for MulOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(MulOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(MulOperator::run)
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(MulOperator::output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        zf_empty_state!()
    }
}

export_operator!(register);

extern "C" fn register(
    configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::OperatorTrait + Send>> {
    Ok(Box::new(MulOperator) as Box<dyn OperatorTrait + Send>)
}
