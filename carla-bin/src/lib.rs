use anyhow::{Context, Result};
use carla_src::{build_libcarla_client_library, probe};
use once_cell::sync::OnceCell;
use std::{
    fs::{self, OpenOptions},
    path::{Path, PathBuf},
};

pub struct CarlaBuild {
    pub src_dir: PathBuf,
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
}

/// Prepares the Carla simulator source code and build client library
/// when called.
pub fn build_carla() -> Result<&'static CarlaBuild> {
    static ONCE: OnceCell<CarlaBuild> = OnceCell::new();
    let build = ONCE.get_or_try_init(cached_or_build)?;
    Ok(build)
}

fn cached_or_build() -> Result<CarlaBuild> {
    let out_dir = Path::new(include_str!(concat!(env!("OUT_DIR"), "/OUT_DIR")));
    let manifest_dir = Path::new(include_str!(concat!(
        env!("OUT_DIR"),
        "/CARGO_MANIFEST_DIR"
    )));
    let build_ready_file = out_dir.join("carla_build_ready");
    let tag = {
        let tag_file = out_dir.join("TAG");
        fs::read_to_string(&tag_file)?
    };
    let cache_dir = manifest_dir.join("cache").join(&tag);

    // Download source code
    let src_dir = carla_src::Download {
        out_dir: Some(out_dir.to_path_buf()),
        ..Default::default()
    }
    .run()
    .with_context(|| "Failed to prepare LibCarla.client C++ library")?;
    let probe = probe(&src_dir);

    // Try to use cache
    #[cfg(not(feature = "force-rebuild"))]
    {
        if cache_dir.is_dir() {
            return Ok(CarlaBuild {
                src_dir,
                lib_dirs: vec![cache_dir],
                include_dirs: probe.include_dirs.into_vec(),
            });
        }
    }

    skip_or_run(&build_ready_file, || {
        build_libcarla_client_library(&src_dir)?;
        anyhow::Ok(())
    })?;

    Ok(CarlaBuild {
        src_dir,
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
