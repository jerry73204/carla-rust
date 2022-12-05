use anyhow::{Context, Result};
use once_cell::sync::OnceCell;
use std::{
    fs::OpenOptions,
    path::{Path, PathBuf},
    process::Command,
};

/// Prepares the Carla simulator source code and build client library
/// when called.
pub fn carla_dir() -> Result<&'static Path> {
    static ONCE: OnceCell<PathBuf> = OnceCell::new();

    let carla_dir = ONCE.get_or_try_init(|| {
        let out_dir = Path::new(include_str!(concat!(env!("OUT_DIR"), "/OUT_DIR")));
        let build_ready_file = out_dir.join("carla_build_ready");

        // Download source code
        let carla_dir = carla_src::Config {
            out_dir: Some(out_dir.to_path_buf()),
            ..Default::default()
        }
        .load()
        .with_context(|| "Failed to prepare LibCarla.client C++ library")?;

        // Build
        skip_or_run(&build_ready_file, || {
            build_libcarla_client_library(&carla_dir)
        })?;

        anyhow::Ok(carla_dir)
    })?;

    Ok(carla_dir)
}

fn build_libcarla_client_library(carla_src_dir: &Path) -> Result<()> {
    // make LibCarla.client.release
    Command::new("make")
        .arg("LibCarla.client.release")
        .current_dir(carla_src_dir)
        .status()
        .with_context(|| "Failed to run `make LibCarla.client`")?;
    Ok(())
}

fn skip_or_run<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    if target_path.exists() {
        return Ok(None);
    }
    let output = (callback)()?;
    touch(target_path)?;
    Ok(Some(output))
}

fn touch(path: &Path) -> Result<()> {
    OpenOptions::new().create(true).write(true).open(path)?;
    Ok(())
}
