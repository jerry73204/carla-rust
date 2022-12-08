use anyhow::{Context, Result};
use std::{path::Path, process::Command};

pub fn build_libcarla_client_library(carla_src_dir: &Path) -> Result<()> {
    // make LibCarla.client.release
    Command::new("make")
        .arg("LibCarla.client.release")
        .current_dir(carla_src_dir)
        .status()
        .with_context(|| "Failed to run `make LibCarla.client`")?;
    Ok(())
}
