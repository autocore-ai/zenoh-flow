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

use crate::model::link::{LinkFromDescriptor, LinkToDescriptor};
use crate::DurationDescriptor;
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeadlineDescriptor {
    pub(crate) from: LinkFromDescriptor,
    pub(crate) to: LinkToDescriptor,
    pub(crate) duration: DurationDescriptor,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeadlineRecord {
    pub(crate) from: LinkFromDescriptor,
    pub(crate) to: LinkToDescriptor,
    pub(crate) duration: Duration,
}

impl From<DeadlineDescriptor> for DeadlineRecord {
    fn from(desc: DeadlineDescriptor) -> Self {
        Self {
            from: desc.from,
            to: desc.to,
            duration: desc.duration.to_duration(),
        }
    }
}
