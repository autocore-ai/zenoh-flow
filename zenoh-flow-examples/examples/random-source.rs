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
use zenoh_flow::{
    serde::{Deserialize, Serialize},
    types::{
        DataTrait, FnOutputRule, FnSourceRun, FutRunOutput, RunOutput, SourceTrait, StateTrait,
        ZFContext, ZFResult,
    },
    zenoh_flow_derive::ZFState,
    zf_data, zf_empty_state, ZFPortID,
};
use zenoh_flow_examples::RandomData;

use zenoh_flow_examples::{ZFString};
use autocxx::include_cpp;

use cxx::{CxxString, CxxVector, UniquePtr};

static SOURCE: &str = "Random";

// #[cxx::bridge]
// mod ffi {
//     unsafe extern "C++" {
//         include!("zenoh-flow-examples/examples/include/cxx_test_rust.hpp");

//         fn cxx_test_string_rust(a: f64, b: f64) -> String;
//         fn cxx_test_vector_rust(a: f64, b: f64) -> Vec<f64>;
//         //fn cxx_test_string_wdnm(a: f64, b: f64) -> String;
//     }
// }

#[derive(Serialize, Deserialize, Debug, ZFState)]
struct ExampleRandomSource {}


autocxx::include_cpp! {
    #include "include/cxx_test/cxx_test_wrapper_cpp.hpp"
    safety!(unsafe_ffi)
    generate!("cxx_test_string_cpp")
    generate!("cxx_test_vector_cpp")
}

impl ExampleRandomSource {
    async fn run_1(_ctx: ZFContext) -> RunOutput {
        let mut results: HashMap<ZFPortID, Arc<dyn DataTrait>> = HashMap::new();


        let a: f64 = 11.0;
        let b: f64 = 25.0;

        let mut result_string = "".to_string();
        let mut result_vector:Vec<f64> = vec![0.0, 0.0];

        println!("a = {}, b = {}", a, b);

        let cxx_string:UniquePtr<CxxString> = ffi::cxx_test_string_cpp(a, b);
        let cxx_vector:UniquePtr<CxxVector<f64>> = ffi::cxx_test_vector_cpp(a, b);

        result_string = cxx_string.to_string();
        result_vector = cxx_vector.iter().map(|s| *s).collect();
        //result_vector = cxx_vector.
        // unsafe {
        //    result_string = ffi::cxx_test_string_rust(a, b);
        //    result_vector =  ffi::cxx_test_vector_rust(a, b);    
        // }

        // unsafe{result_string = ffi::cxx_test_string_wdnm(a, b);}
        let result_zfstring = ZFString::from(result_string);

        println!("add result = {}, mul result = {}", result_vector[0], result_vector[1]);

        results.insert(String::from(SOURCE), zf_data!(result_zfstring));
        async_std::task::sleep(std::time::Duration::from_secs(1)).await;
        Ok(results)
    }
}

impl SourceTrait for ExampleRandomSource {
    fn get_run(&self, _ctx: ZFContext) -> FnSourceRun {
        Box::new(|ctx: ZFContext| -> FutRunOutput { Box::pin(Self::run_1(ctx)) })
    }

    fn get_output_rule(&self, _ctx: ZFContext) -> Box<FnOutputRule> {
        Box::new(zenoh_flow::default_output_rule)
    }

    fn get_state(&self) -> Box<dyn StateTrait> {
        zf_empty_state!()
    }
}

// //Also generated by macro
zenoh_flow::export_source!(register);

extern "C" fn register(
    _configuration: Option<HashMap<String, String>>,
) -> ZFResult<Box<dyn zenoh_flow::SourceTrait + Send>> {
    Ok(Box::new(ExampleRandomSource {}) as Box<dyn zenoh_flow::SourceTrait + Send>)
}
