use anyhow::{ensure, Result};
use flate2::bufread::GzDecoder;
use log::info;
use serde::{Deserialize, Serialize};
use std::{
    env,
    fs::{File, OpenOptions},
    io::{self, prelude::*, BufReader, BufWriter},
    path::{Path, PathBuf},
    sync::Arc,
};
use tar::Archive;

use crate::utils::{sha256sum, skip_or_run};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Download {
    pub url: String,
    pub dir: PathBuf,
    #[serde(with = "hex::serde")]
    pub sha256sum: Vec<u8>,
    pub out_dir: Option<PathBuf>,
}

impl Download {
    pub fn run(self) -> Result<PathBuf> {
        let config = self;
        let out_dir = config.out_dir.clone().unwrap_or_else(|| {
            let var = env::var_os("OUT_DIR").expect("OUT_DIR is not set");
            PathBuf::from(var)
        });
        let source_ready_file = out_dir.join("source_ready");
        let src_dir = out_dir.join(&config.dir);

        println!("cargo:rerun-if-changed={}", out_dir.to_str().unwrap());

        // Prepare source files
        skip_or_run(&source_ready_file, || {
            prepare_tar_gz(&config.url, &config.sha256sum, &out_dir)?;
            ensure!(src_dir.is_dir(), "'{}' does not exist", src_dir.display());
            Ok(())
        })?;

        Ok(src_dir)
    }
}

impl Default for Download {
    fn default() -> Self {
        let text = include_str!("../default_config.json5");
        json5::from_str(text).unwrap()
    }
}

fn prepare_tar_gz(url: &str, sha256_digest: &[u8], dir: &Path) -> Result<()> {
    let tarball_path = dir.join("source.tar.gz");

    // Download source file if it's not ready
    let is_source_ready = tarball_path.is_file() && sha256_digest == sha256sum(&tarball_path)?;

    if is_source_ready {
        info!("Source file is ready. Skip downloading.");
    } else {
        info!("Downloading file from '{}'", url);

        let mut tarball_file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(&tarball_path)?;

        let mut writer = BufWriter::new(&mut tarball_file);
        let mut reader = ureq::AgentBuilder::new()
            .tls_connector(Arc::new(native_tls::TlsConnector::new()?))
            .build()
            .get(url)
            .call()?
            .into_reader();
        io::copy(&mut reader, &mut writer)?;
        writer.flush()?;
    }

    // Verify file
    {
        let digest = sha256sum(&tarball_path)?;
        ensure!(
            sha256_digest == digest,
            "Checksum mismatch: expect {:X?}, but get {:x?}",
            sha256_digest,
            digest
        );
    }

    // unpack tar.gz
    {
        info!("Unpacking {}", tarball_path.display());
        let mut tarball_file = BufReader::new(File::open(&tarball_path)?);
        let mut reader = BufReader::new(&mut tarball_file);
        let tar = GzDecoder::new(&mut reader);
        let mut archive = Archive::new(tar);
        archive.unpack(dir)?;
    }

    Ok(())
}
