use crate::{probe, Probe};
use anyhow::{bail, Context, Result};
use fs_extra::dir::CopyOptions;
use std::{fs, path::Path, process::Command};

/// Get the CARLA version from environment variable or use default
pub fn version() -> &'static str {
    // This is evaluated at compile time via build script
    option_env!("CARLA_VERSION").unwrap_or("0.9.16")
}

pub const LIBS: &[&str] = &[
    "static=carla_client",
    "static=Recast",
    "static=Detour",
    "static=DetourCrowd",
    "static=rpc",
    "static=png",
    "static=boost_filesystem",
];

pub fn build<P>(src_dir: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let src_dir = src_dir.as_ref();
    let err = || {
        format!(
            "`make LibCarla.client.release` failed in {}",
            src_dir.display()
        )
    };

    // make LibCarla.client.release
    let status = Command::new("make")
        .arg("LibCarla.client.release")
        .current_dir(src_dir)
        .status()
        .with_context(err)?;

    if !status.success() {
        bail!("{}", err());
    }

    Ok(())
}

pub fn install<S, T>(src_dir: S, tgt_dir: T) -> Result<()>
where
    S: AsRef<Path>,
    T: AsRef<Path>,
{
    let tgt_dir = tgt_dir.as_ref();

    let Probe {
        include_dirs,
        lib_dirs,
        ..
    } = probe(src_dir)?;

    {
        let tgt_include_dir = tgt_dir.join("include");
        fs::create_dir_all(&tgt_include_dir)?;

        for dir in include_dirs.into_vec() {
            fs_extra::dir::copy(
                &dir,
                &tgt_include_dir,
                &CopyOptions {
                    overwrite: true,
                    skip_exist: false,
                    copy_inside: true,
                    content_only: true,
                    ..Default::default()
                },
            )
            .with_context(|| {
                format!(
                    "Unable to copy directory from\
                           '{}'\
                           to\
                           '{}'",
                    dir.display(),
                    tgt_include_dir.display()
                )
            })?;
        }
    }

    {
        let tgt_lib_dir = tgt_dir.join("lib");
        fs::create_dir_all(&tgt_lib_dir)?;

        for dir in lib_dirs.into_vec() {
            fs_extra::dir::copy(
                &dir,
                &tgt_lib_dir,
                &CopyOptions {
                    overwrite: true,
                    skip_exist: false,
                    copy_inside: true,
                    content_only: true,
                    ..Default::default()
                },
            )
            .with_context(|| {
                format!(
                    "Unable to copy directory from\
                           '{}'\
                           to\
                           '{}'",
                    dir.display(),
                    tgt_lib_dir.display()
                )
            })?;
        }
    }

    Ok(())
}

pub fn clean<P>(src_dir: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let src_dir = src_dir.as_ref();
    let build_dir = src_dir.join("Build");
    let err = || format!("`make clean.LibCarla` failed in {}", src_dir.display());

    if build_dir.exists() {
        fs::remove_dir_all(build_dir)?;
    }

    // make LibCarla.client.release
    let status = Command::new("make")
        .arg("clean.LibCarla")
        .current_dir(src_dir)
        .status()
        .with_context(err)?;

    if !status.success() {
        bail!("{}", err());
    }

    Ok(())
}
