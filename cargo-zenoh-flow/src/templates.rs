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

use crate::{CZFError, CZFResult};
use serde::Serialize;
use tinytemplate::TinyTemplate;

static CARGO_OPERATOR_TEMPLATE: &str = r#"
zenoh-flow = \{ git = "https://github.com/eclipse-zenoh/zenoh-flow.git", branch = "master"}

[lib]
name = "{name}"
crate-type=["cdylib"]
path="src/lib.rs"

[package.metadata.zenohflow]
id = "{name}"
kind = "operator"
inputs=[ \{id ="INPUT", type="bytes"}]
outputs=[ \{id ="OUTPUT", type="bytes"}]

"#;

static CARGO_SOURCE_TEMPLATE: &str = r#"
zenoh-flow = \{ git = "https://github.com/eclipse-zenoh/zenoh-flow.git", branch = "master"}
async-trait = "0.1"

[lib]
name = "{name}"
crate-type=["cdylib"]
path="src/lib.rs"

[package.metadata.zenohflow]
id = "{name}"
kind = "source"
outputs=[ \{id ="Data", type="bytes"}]

"#;

static CARGO_SINK_TEMPLATE: &str = r#"
zenoh-flow = \{ git = "https://github.com/eclipse-zenoh/zenoh-flow.git", branch = "master"}
async-trait = "0.1"

[lib]
name = "{name}"
crate-type=["cdylib"]
path="src/lib.rs"

[package.metadata.zenohflow]
id = "{name}"
kind = "sink"
inputs=[ \{id ="Data", type="bytes"}]

"#;

static LIB_OPERATOR_TEMPLATE: &str = r#"
use zenoh_flow::async_std::sync::Arc;
use std::collections::HashMap;
use zenoh_flow::zenoh_flow_derive::ZFState;
use zenoh_flow::\{
    default_input_rule, default_output_rule, downcast_mut, Node, NodeOutput, Data, Operator,
    ZFResult, ZFState, PortId
};

#[derive(Debug)]
struct {name};

static INPUT: &str = "INPUT";
static OUTPUT: &str = "OUTPUT";

impl Operator for {name} \{

    fn input_rule(
        &self,
        _context: &mut zenoh_flow::Context,
        state: &mut Box<dyn zenoh_flow::ZFState>,
        tokens: &mut HashMap<PortId, zenoh_flow::Token>,
    ) -> ZFResult<bool> \{
        default_input_rule(state, tokens)
    }

    fn run(
        &self,
        _context: &mut zenoh_flow::Context,
        state: &mut Box<dyn zenoh_flow::ZFState>,
        inputs: &mut HashMap<PortId, zenoh_flow::runtime::message::DataMessage>,
    ) -> ZFResult<HashMap<PortId, Data>> \{
        todo!()
    }

    fn output_rule(
        &self,
        _context: &mut zenoh_flow::Context,
        state: &mut Box<dyn zenoh_flow::ZFState>,
        outputs: HashMap<PortId, Data>,
    ) -> ZFResult<HashMap<PortId, NodeOutput>> \{
        default_output_rule(state, outputs)
    }
}

impl Node for {name} \{
    fn initialize(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn zenoh_flow::ZFState> \{
        zenoh_flow::zf_empty_state!()
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> \{
        Ok(())
    }
}

// Also generated by macro
zenoh_flow::export_operator!(register);

fn register() -> ZFResult<Arc<dyn Operator>> \{
    Ok(Arc::new({name}) as Arc<dyn Operator>)
}

"#;

static LIB_SOURCE_TEMPLATE: &str = r#"
use zenoh_flow::async_std::sync::Arc;
use async_trait::async_trait;
use std::collections::HashMap;
use zenoh_flow::zenoh_flow_derive::ZFState;
use zenoh_flow::\{downcast_mut, Node, Data, Source,
    ZFResult, ZFState, PortId
};

#[derive(Debug)]
struct {name};

#[async_trait]
impl Source for {name} \{
    async fn run(
        &self,
        _context: &mut zenoh_flow::Context,
        state: &mut Box<dyn zenoh_flow::ZFState>,
    ) -> ZFResult<Data> \{
        todo!()
    }

}


impl Node for {name} \{
    fn initialize(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn zenoh_flow::ZFState> \{
        zenoh_flow::zf_empty_state!()
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> \{
        Ok(())
    }
}

// Also generated by macro
zenoh_flow::export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> \{
    Ok(Arc::new({name}) as Arc<dyn Source>)
}

"#;

static LIB_SINK_TEMPLATE: &str = r#"
use zenoh_flow::async_std::sync::Arc;
use async_trait::async_trait;
use std::collections::HashMap;
use zenoh_flow::zenoh_flow_derive::ZFState;
use zenoh_flow::\{downcast_mut, Node, Data, Sink,
    ZFResult, ZFState, PortId
};

#[derive(Debug)]
struct {name};

#[async_trait]
impl Sink for {name} \{
    async fn run(
        &self,
        _context: &mut zenoh_flow::Context,
        state: &mut Box<dyn zenoh_flow::ZFState>,
        mut input: zenoh_flow::runtime::message::DataMessage,
    ) -> ZFResult<()> \{
        todo!()
    }
}


impl Node for {name} \{
    fn initialize(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn zenoh_flow::ZFState> \{
        zenoh_flow::zf_empty_state!()
    }

    fn finalize(&self, _state: &mut State) -> ZFResult<()> \{
        Ok(())
    }
}

// Also generated by macro
zenoh_flow::export_sink!(register);

fn register() -> ZFResult<Arc<dyn Sink>> \{
    Ok(Arc::new({name}) as Arc<dyn Sink>)
}

"#;

#[derive(Serialize)]
struct OperatorContext {
    name: String,
}

fn some_kind_of_uppercase_first_letter(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        None => String::new(),
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
    }
}

pub fn operator_template_cargo(name: String) -> CZFResult<String> {
    let mut tt = TinyTemplate::new();
    tt.add_template("operator", CARGO_OPERATOR_TEMPLATE)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))?;

    let ctx = OperatorContext { name };

    tt.render("operator", &ctx)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))
}

pub fn operator_template_lib(name: String) -> CZFResult<String> {
    let mut tt = TinyTemplate::new();
    tt.add_template("operator", LIB_OPERATOR_TEMPLATE)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))?;

    let ctx = OperatorContext {
        name: some_kind_of_uppercase_first_letter(&name),
    };

    tt.render("operator", &ctx)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))
}

pub fn source_template_lib(name: String) -> CZFResult<String> {
    let mut tt = TinyTemplate::new();
    tt.add_template("source", LIB_SOURCE_TEMPLATE)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))?;

    let ctx = OperatorContext {
        name: some_kind_of_uppercase_first_letter(&name),
    };

    tt.render("source", &ctx)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))
}

pub fn source_template_cargo(name: String) -> CZFResult<String> {
    let mut tt = TinyTemplate::new();
    tt.add_template("source", CARGO_SOURCE_TEMPLATE)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))?;

    let ctx = OperatorContext { name };

    tt.render("source", &ctx)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))
}

pub fn sink_template_lib(name: String) -> CZFResult<String> {
    let mut tt = TinyTemplate::new();
    tt.add_template("sink", LIB_SINK_TEMPLATE)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))?;

    let ctx = OperatorContext {
        name: some_kind_of_uppercase_first_letter(&name),
    };

    tt.render("sink", &ctx)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))
}

pub fn sink_template_cargo(name: String) -> CZFResult<String> {
    let mut tt = TinyTemplate::new();
    tt.add_template("sink", CARGO_SINK_TEMPLATE)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))?;

    let ctx = OperatorContext { name };

    tt.render("sink", &ctx)
        .map_err(|e| CZFError::GenericError(format!("{}", e)))
}
