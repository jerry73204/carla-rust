use crate::{probe, Probe};
use anyhow::{bail, Context, Result};
use fs_extra::dir::CopyOptions;
use std::{fs, path::Path, process::Command};

pub const LIBS: &[&str] = &[
    "static=carla_client",
    "static=Recast",
    "static=Detour",
    "static=DetourCrowd",
    "static=rpc",
    "static=png",
    "static=boost_filesystem",
];

pub fn build(src_dir: &Path) -> Result<()> {
    let err = || format!("`make LibCarla.client` failed in {}", src_dir.display());

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

pub fn install(src_dir: &Path, tgt_dir: &Path) -> Result<()> {
    let Probe {
        include_dirs,
        lib_dirs,
        ..
    } = probe(src_dir);

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
