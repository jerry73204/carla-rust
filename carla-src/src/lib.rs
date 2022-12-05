pub mod config;
pub use config::Config;

use anyhow::{ensure, Context, Result};
use cfg_if::cfg_if;
use flate2::bufread::GzDecoder;
use itertools::chain;
use log::info;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, File, OpenOptions},
    io::{self, prelude::*, BufReader, BufWriter},
    path::{Path, PathBuf},
    process::Command,
    sync::Arc,
    thread,
};
use tar::Archive;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarlaSrc {
    pub carla_src_dir: PathBuf,
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
    pub libs: Vec<String>,
}

pub(crate) fn load(config: Config) -> Result<PathBuf> {
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

fn sha256sum(path: &Path) -> Result<[u8; 32]> {
    let mut hasher = Sha256::new();
    let mut reader = BufReader::new(File::open(path)?);

    loop {
        let mut buf = [0u8; 8192];
        let len = reader.read(&mut buf)?;
        if len == 0 {
            break;
        }
        hasher.update(&buf[0..len]);
    }

    let hash = hasher.finalize();
    Ok(hash.as_slice().try_into().unwrap())
}

fn touch(path: &Path) -> Result<()> {
    OpenOptions::new().create(true).write(true).open(path)?;
    Ok(())
}

fn skip_or_run<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    with_target(target_path, || {
        let output = (callback)()?;
        touch(target_path)?;
        Ok(output)
    })
}

fn with_target<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    with_targets(&[target_path], callback)
}

fn with_targets<T, F>(target_paths: &[&Path], callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    let ok = target_paths.iter().all(|path| path.exists());
    if ok {
        return Ok(None);
    }

    (callback)().map(Some)
}

// fn llvm8_include_dir() -> Result<PathBuf> {
//     let mut stdout = Command::new("llvm-config-8")
//         .arg("--includedir")
//         .output()
//         .with_context(|| "Is llvm-config-8 installed on your system?")?
//         .stdout;

//     // Remove a trailing '\n'
//     assert!(stdout.pop() == Some(b'\n'));

//     let path = bytes_to_path(stdout)?;
//     Ok(path)
// }

// fn llvm8_lib_dir() -> Result<PathBuf> {
//     let mut stdout = Command::new("llvm-config-8")
//         .arg("--libdir")
//         .output()
//         .with_context(|| "Is llvm-config-8 installed on your system?")?
//         .stdout;

//     // Remove a trailing '\n'
//     assert!(stdout.pop() == Some(b'\n'));

//     let path = bytes_to_path(stdout)?;
//     Ok(path)
// }

// fn bytes_to_path(bytes: Vec<u8>) -> Result<PathBuf> {
//     let path = {
//         cfg_if! {
//             if #[cfg(unix)] {
//                 use std::os::unix::ffi::OsStringExt;
//                 PathBuf::from(OsString::from_vec(bytes))
//             } else if #[cfg(windows)] {
//                 use std::os::windows::ffi::OsStringExt;
//                 let bytes: &[u16] = safe_transmute::transmute_many_pedantic(&stdout)?;
//                 PathBuf::from(OsString::from_wide(bytes))
//             } else if #[cfg(wasm)] {
//                 use std::os::wasi::ffi::OsStringExt;
//                 PathBuf::from(OsString::from_vec(bytes))
//             }
//         }
//     };
//     Ok(path)
// }
