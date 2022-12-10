use anyhow::{ensure, Context, Result};
use carla_src::{libcarla_client, probe};
use once_cell::sync::OnceCell;
use std::{
    fs::{self, OpenOptions},
    io::{self, BufReader},
    path::{Path, PathBuf},
};
use tar::Archive;
use xz::bufread::XzDecoder;

pub struct CarlaBuild {
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
}

const PREBUILD_DIR_NAME: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/PREBUILD_DIR_NAME"));
const DOWNLOAD_URL: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/DOWNLOAD_URL"));
const OUT_DIR: &str = env!("OUT_DIR");
const TAG: &str = include_str!(concat!(env!("OUT_DIR"), "/TAG"));
const BUILD_READY_FILE: &str = concat!(env!("OUT_DIR"), "/BUILD_READY");
const DOWNLOAD_READY_FILE: &str = concat!(env!("OUT_DIR"), "/DOWNLOAD_READY");

/// Prepares the Carla simulator source code and build client library
/// when called.
pub fn build_carla() -> Result<&'static CarlaBuild> {
    static ONCE: OnceCell<CarlaBuild> = OnceCell::new();
    let build = ONCE.get_or_try_init(cached_or_build)?;
    Ok(build)
}

fn cached_or_build() -> Result<CarlaBuild> {
    // Try to use cached files
    #[cfg(not(feature = "force-rebuild"))]
    if let Some(build) = load_cache()? {
        return Ok(build);
    }

    // Download source code
    let src_dir = carla_src::Download::default()
        .run()
        .with_context(|| "Failed to prepare LibCarla.client C++ library")?;
    let probe = probe(&src_dir);

    // Build if it wasn't done before
    let build_ready_file = Path::new(BUILD_READY_FILE);
    skip_or_run(build_ready_file, || {
        libcarla_client::build(&src_dir)?;
        anyhow::Ok(())
    })?;

    Ok(CarlaBuild {
        include_dirs: probe.include_dirs.into_vec(),
        lib_dirs: probe.lib_dirs.into_vec(),
    })
}

fn load_cache() -> Result<Option<CarlaBuild>> {
    cached_or_download()?;
    let install_dir = install_dir();

    if !install_dir.is_dir() {
        return Ok(None);
    }

    let include_dir = install_dir.join("include");
    let lib_dir = install_dir.join("lib");

    Ok(Some(CarlaBuild {
        include_dirs: vec![include_dir],
        lib_dirs: vec![lib_dir],
    }))
}

fn cached_or_download() -> Result<()> {
    let download_ready_file = Path::new(DOWNLOAD_READY_FILE);
    skip_or_run(download_ready_file, download_prebuild)?;
    Ok(())
}

fn download_prebuild() -> Result<()> {
    let prebuild_dir = prebuild_dir();

    // Clean the dir to extract to
    let result = fs::remove_dir_all(&prebuild_dir);
    match result {
        Ok(()) => {}
        Err(err) if err.kind() == io::ErrorKind::NotFound => {}
        Err(err) => return Err(err.into()),
    }

    // Download and extract to prebuild_dir
    let reader = ureq::get(DOWNLOAD_URL)
        .call()
        .with_context(|| format!("Failed to download from URL '{}'", DOWNLOAD_URL))?
        .into_reader();
    let reader = BufReader::new(reader);
    let mut archive = Archive::new(XzDecoder::new(reader));
    archive
        .unpack(OUT_DIR)
        .with_context(|| format!("Failed to unpack file downloaded from '{}'", DOWNLOAD_URL))?;

    // Check if desired extracted files exist
    ensure!(
        prebuild_dir.is_dir(),
        "'{}' does not exist. Is the URL '{}' correct?",
        prebuild_dir.display(),
        DOWNLOAD_URL
    );

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

fn prebuild_dir() -> PathBuf {
    Path::new(OUT_DIR).join(PREBUILD_DIR_NAME)
}

fn install_dir() -> PathBuf {
    prebuild_dir().join(TAG)
}
