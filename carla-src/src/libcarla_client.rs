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
    let carla_version = std::env::var("CARLA_VERSION").unwrap_or_else(|_| version().to_string());

    match carla_version.as_str() {
        "0.9.14" => build_0914(src_dir),
        "0.9.16" => build_0916(src_dir),
        _ => bail!(
            "Unsupported CARLA version: {}. Supported versions: 0.9.14, 0.9.16",
            carla_version
        ),
    }
}

fn build_0914<P>(src_dir: P) -> Result<()>
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

fn build_0916<P>(src_dir: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let src_dir = src_dir.as_ref();
    let ue4_root = src_dir.join("Unreal").join("UnrealEngine");

    // Set UE4_ROOT environment variable
    std::env::set_var("UE4_ROOT", &ue4_root);

    // Clone UnrealEngine if not present
    if !ue4_root.join("Engine").exists() {
        eprintln!("Cloning UnrealEngine 4.26 (shallow clone to save space)...");
        eprintln!("Note: We only need the toolchain, not the full build");

        // Remove incomplete directory if exists
        if ue4_root.exists() {
            fs::remove_dir_all(&ue4_root)?;
        }

        // Create parent directory
        fs::create_dir_all(ue4_root.parent().unwrap())?;

        // Clone CARLA's fork of Unreal Engine (shallow clone)
        let status = Command::new("git")
            .args([
                "clone",
                "--depth",
                "1",
                "-b",
                "carla",
                "https://github.com/CarlaUnreal/UnrealEngine.git",
                ue4_root.to_str().unwrap(),
            ])
            .status()
            .context("Failed to clone UnrealEngine")?;

        if !status.success() {
            bail!("Failed to clone UnrealEngine");
        }
    }

    // Download UE4 toolchain if not already done
    if !ue4_root.join("Build/OneTimeSetupPerformed").exists() {
        eprintln!("Downloading UE4 toolchain (~734 MB)...");
        eprintln!("Note: We skip the full UnrealEngine build to save ~91GB");

        // Workaround: If .git is a file (gitdir reference from submodule setup),
        // Setup.sh cannot create ../.git/ue4-sdks/ cache directory. Temporarily rename it.
        let git_file = ue4_root.join(".git");
        let git_tmp = ue4_root.join(".git.tmp");
        let git_file_renamed = git_file.is_file() && !git_file.is_dir();

        if git_file_renamed {
            eprintln!("Note: Detected .git file reference - temporarily renaming for Setup.sh");
            fs::rename(&git_file, &git_tmp)?;
        }

        let status = Command::new("./Setup.sh")
            .current_dir(&ue4_root)
            .status()
            .context("Failed to run UnrealEngine Setup.sh")?;

        // Restore the .git file if we renamed it
        if git_file_renamed {
            fs::rename(&git_tmp, &git_file)?;
            eprintln!("Restored .git file reference");
        }

        if !status.success() {
            bail!("UnrealEngine Setup.sh failed");
        }

        eprintln!("Toolchain downloaded successfully!");
    }

    // Run CARLA setup to regenerate toolchain files with correct UE4_ROOT paths
    eprintln!("Running CARLA setup...");
    let status = Command::new("make")
        .arg("setup")
        .current_dir(src_dir)
        .env("UE4_ROOT", &ue4_root)
        .status()
        .context("Failed to run `make setup`")?;

    if !status.success() {
        bail!("`make setup` failed in {}", src_dir.display());
    }

    eprintln!("CARLA setup complete!");

    // Build LibCarla Client
    eprintln!("Building LibCarla Client...");
    let status = Command::new("make")
        .arg("LibCarla.client.release")
        .current_dir(src_dir)
        .env("UE4_ROOT", &ue4_root)
        .status()
        .context("Failed to run `make LibCarla.client.release`")?;

    if !status.success() {
        bail!(
            "`make LibCarla.client.release` failed in {}",
            src_dir.display()
        );
    }

    eprintln!("Build complete!");
    eprintln!("Output locations:");
    eprintln!("  Build folder:   Build/libcarla-client-build.release");
    eprintln!("  Install folder: PythonAPI/carla/dependencies");

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
