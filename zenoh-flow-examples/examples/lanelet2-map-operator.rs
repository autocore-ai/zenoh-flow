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
    runtime::message::Message,
    types::{
        DataTrait, FnInputRule, FnOutputRule, FnRun, InputRuleResult, OperatorTrait,
        OutputRuleResult, RunResult, StateTrait, ZFInput, ZFResult,
    },
    zf_data, zf_empty_state, Token, ZFContext, ZFError,
};

struct Lanelet2MapOperator;

static LINK_ID_INPUT_STR: &str = "Str";
static LINK_ID_OUTPUT_STR: &str = "Str";

impl Lanelet2MapOperator {
    fn input_rule(_ctx: ZFContext, inputs: &mut HashMap<String, Token>) -> InputRuleResult {
        for token in inputs.values() {
            match token {
                Token::Ready(_) => continue,
                _ => return Ok(false),
            }
        }
        Ok(true)
    }

    fn run(_ctx: ZFContext, mut inputs: ZFInput) -> RunResult {
        let mut result = HashMap::<String, Arc<dyn DataTrait>>::with_capacity(1);

        let (_, config) = get_input!(ZFString, String::from(LINK_ID_INPUT_STR), inputs)?.clone();

        result.insert(String::from(LINK_ID_OUTPUT_STR), zf_data!(ZFString(config.0)));

        Ok(result)
    }

    fn output_rule(
        _ctx: ZFContext,
        outputs: HashMap<String, Arc<dyn DataTrait>>,
    ) -> OutputRuleResult {
        let mut zf_outputs: HashMap<String, Message> = HashMap::with_capacity(1);

        zf_outputs.insert(
            String::from(LINK_ID_OUTPUT_STR),
            Message::from_data(outputs.get(LINK_ID_OUTPUT_STR).unwrap().clone()),
        );

        Ok(zf_outputs)
    }
}

impl OperatorTrait for Lanelet2MapOperator {
    fn get_input_rule(&self, _ctx: ZFContext) -> Box<FnInputRule> {
        Box::new(Lanelet2MapOperator::input_rule)
    }

    fn get_run(&self, _ctx: ZFContext) -> Box<FnRun> {
        Box::new(Lanelet2MapOperator::run)
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(Lanelet2MapOperator::output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        zf_empty_state!()
    }
}

export_operator!(register);

extern "C" fn register(
    configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::OperatorTrait + Send>> {
    Ok(Box::new(Lanelet2MapOperator) as Box<dyn OperatorTrait + Send>)
}
