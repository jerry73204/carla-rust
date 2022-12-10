use anyhow::{Context, Result};
use carla_src::{libcarla_client, probe};
use once_cell::sync::OnceCell;
use std::{
    fs::{self, OpenOptions},
    path::{Path, PathBuf},
};

pub struct CarlaBuild {
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
}

const PREBUILD_DIR_NAME: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/PREBUILD_DIR_NAME"));

/// Prepares the Carla simulator source code and build client library
/// when called.
pub fn build_carla() -> Result<&'static CarlaBuild> {
    static ONCE: OnceCell<CarlaBuild> = OnceCell::new();
    let build = ONCE.get_or_try_init(cached_or_build)?;
    Ok(build)
}

fn cached_or_build() -> Result<CarlaBuild> {
    let out_dir = Path::new(env!("OUT_DIR"));
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let build_ready_file = out_dir.join("carla_build_ready");
    let tag_file = out_dir.join("TAG");
    let tag = fs::read_to_string(&tag_file)?;
    let prebuild_dir = manifest_dir.join(PREBUILD_DIR_NAME).join(&tag);

    // Try to use cached files
    #[cfg(not(feature = "force-rebuild"))]
    {
        if prebuild_dir.is_dir() {
            let include_dir = prebuild_dir.join("include");
            let lib_dir = prebuild_dir.join("lib");

            return Ok(CarlaBuild {
                include_dirs: vec![include_dir],
                lib_dirs: vec![lib_dir],
            });
        }
    }

    // Download source code
    let src_dir = carla_src::Download::default()
        .run()
        .with_context(|| "Failed to prepare LibCarla.client C++ library")?;
    let probe = probe(&src_dir);

    // Build if it wasn't done before
    skip_or_run(&build_ready_file, || {
        libcarla_client::build(&src_dir)?;
        anyhow::Ok(())
    })?;

    Ok(CarlaBuild {
        include_dirs: probe.include_dirs.into_vec(),
        lib_dirs: probe.lib_dirs.into_vec(),
    })
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
