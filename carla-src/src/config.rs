use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

use crate::CarlaSrc;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub c_compiler: String,
    pub cxx_compiler: String,
    pub carla: Carla,
    pub recast: Recast,
    pub rpclib: Rpclib,
    pub libpng: Libpng,
    pub boost: Boost,
    pub out_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Carla {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Recast {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Rpclib {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Libpng {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Boost {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
    pub bootstrap_toolset: String,
    pub b2_toolset: String,
}

impl Config {
    pub fn build<P>(self) -> CarlaSrc
    where
        P: AsRef<Path>,
    {
        crate::build(self)
    }
}

impl Default for Config {
    fn default() -> Self {
        let text = include_str!("../default_config.json5");
        json5::from_str(text).unwrap()
    }
}
