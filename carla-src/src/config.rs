use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
    pub out_dir: Option<PathBuf>,
}

impl Config {
    pub fn load(self) -> Result<PathBuf> {
        crate::load(self)
    }
}

impl Default for Config {
    fn default() -> Self {
        let text = include_str!("../default_config.json5");
        json5::from_str(text).unwrap()
    }
}
