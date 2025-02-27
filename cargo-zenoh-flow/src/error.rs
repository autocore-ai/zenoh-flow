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

#[derive(Debug)]
pub enum CZFError {
    PackageNotFoundInWorkspace(String, String),
    NoRootFoundInWorkspace(String),
    CrateTypeNotCompatible(String),
    CommandFailed(std::io::Error, &'static str),
    CommandError(&'static str, String, Vec<u8>),
    ParseTOML(toml::de::Error),
    ParseJSON(serde_json::Error),
    ParseYAML(serde_yaml::Error),
    MissingField(String, &'static str),
    IoFile(&'static str, std::io::Error, std::path::PathBuf),
    ParsingError(&'static str),
    BuildFailed,
    #[cfg(feature = "local_registry")]
    ZenohError(zenoh::ZError),
    ZenohFlowError(zenoh_flow::ZFError),
    GenericError(String),
    IoError(std::io::Error),
}

impl From<std::io::Error> for CZFError {
    fn from(err: std::io::Error) -> Self {
        Self::IoError(err)
    }
}

impl From<toml::de::Error> for CZFError {
    fn from(err: toml::de::Error) -> Self {
        Self::ParseTOML(err)
    }
}

impl From<serde_json::Error> for CZFError {
    fn from(err: serde_json::Error) -> Self {
        Self::ParseJSON(err)
    }
}

impl From<serde_yaml::Error> for CZFError {
    fn from(err: serde_yaml::Error) -> Self {
        Self::ParseYAML(err)
    }
}

#[cfg(feature = "local_registry")]
impl From<zenoh::ZError> for CZFError {
    fn from(err: zenoh::ZError) -> Self {
        Self::ZenohError(err)
    }
}

impl From<zenoh_flow::ZFError> for CZFError {
    fn from(err: zenoh_flow::ZFError) -> Self {
        Self::ZenohFlowError(err)
    }
}

impl std::fmt::Display for CZFError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

pub type CZFResult<T> = Result<T, CZFError>;
