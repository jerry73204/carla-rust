use anyhow::{bail, Context, Result};
use std::{env, path::PathBuf, str::FromStr};

#[cfg(not(feature = "docs-only"))]
use std::path::Path;

#[cfg(not(feature = "docs-only"))]
use std::fs::File;

#[cfg(not(feature = "docs-only"))]
use std::fs;

use once_cell::sync::Lazy;

#[cfg(not(feature = "docs-only"))]
use anyhow::anyhow;

#[cfg(not(feature = "docs-only"))]
use carla_src::libcarla_client;

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
use std::{collections::HashMap, io};

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
use tar::Archive;

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
use std::io::BufReader;

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
use sha2::{Digest, Sha256};

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
#[derive(Debug, serde::Deserialize)]
struct PrebuiltEntry {
    url: String,
    sha256: String,
}

#[cfg(not(feature = "docs-only"))]
static TAG: Lazy<String> = Lazy::new(|| {
    format!(
        "{}-{}",
        libcarla_client::version(),
        env::var("TARGET").expect("TARGET environment variable not set"),
    )
});

#[cfg(not(feature = "docs-only"))]
static OUT_DIR: Lazy<PathBuf> = Lazy::new(|| {
    PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR environment variable not set"))
});

#[cfg(not(feature = "docs-only"))]
static PREBUILD_NAME: Lazy<String> = Lazy::new(|| format!("libcarla_client.{}", *TAG));

#[cfg(all(feature = "build-prebuilt", not(feature = "docs-only")))]
static SAVE_PREBUILT_TARBALL: Lazy<PathBuf> = Lazy::new(|| {
    let file_name = format!("{}.tar.zstd", *PREBUILD_NAME);
    GENERATED_DIR.join(file_name)
});

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
static DOWNLOAD_PREBUILT_TARBALL: Lazy<PathBuf> = Lazy::new(|| {
    let file_name = format!("{}.tar.zstd", *PREBUILD_NAME);
    OUT_DIR.join(file_name)
});

#[cfg(not(feature = "docs-only"))]
static CARGO_MANIFEST_DIR: Lazy<&Path> = Lazy::new(|| Path::new(env!("CARGO_MANIFEST_DIR")));

#[cfg(all(feature = "build-prebuilt", not(feature = "docs-only")))]
static GENERATED_DIR: Lazy<PathBuf> = Lazy::new(|| CARGO_MANIFEST_DIR.join("generated"));

static CARLA_VERSION: Lazy<CarlaVersion> = Lazy::new(|| {
    parse_carla_version().expect("Failed to parse CARLA_VERSION environment variable")
});

/// Returns the cached CARLA version
fn carla_version() -> CarlaVersion {
    *CARLA_VERSION
}

fn main() -> Result<()> {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/bindings.rs");
    println!("cargo:rerun-if-env-changed=CARLA_DIR");
    println!("cargo:rerun-if-env-changed=CARLA_VERSION");

    // Pass CARLA_VERSION to compiler
    let version = carla_version();
    println!("cargo:rustc-env=CARLA_VERSION={}", version.as_str());

    // Configure LLVM/clang for compatibility with newer systems (Ubuntu 22.04+)
    configure_llvm();

    // Skip build if docs-only feature presents.
    #[cfg(feature = "docs-only")]
    {
        Ok(())
    }

    #[cfg(not(feature = "docs-only"))]
    {
        // Prepare CARLA installation
        let install_dir = load_carla_install_dir()?;
        let carla_lib_dir = install_dir.join("lib");
        let carla_include_dir = install_dir.join("include");

        #[cfg(feature = "build-prebuilt")]
        create_tarball(&install_dir, &*SAVE_PREBUILT_TARBALL)?;

        // Add library search paths
        println!("cargo:rustc-link-search=native={}", carla_lib_dir.display());

        // Link libraries
        for lib in libcarla_client::LIBS {
            println!("cargo:rustc-link-lib={lib}");
        }

        // Generate bindings
        let csrc_dir = CARGO_MANIFEST_DIR.join("csrc");

        // Use headers from install directory (either from CARLA build or extracted tarball)
        let include_dirs = [carla_include_dir, csrc_dir];

        // Export version for dependent crates (will be available as DEP_CARLA_SYS_CARLA_VERSION)
        println!("cargo:carla_version={}", version.as_str());

        // Set version-specific compiler flags before autocxx processes headers
        let extra_clang_args = match version {
            CarlaVersion::V0_9_16 => vec!["-DCARLA_VERSION_0916"],
            CarlaVersion::V0_9_15 => vec![],
            CarlaVersion::V0_9_14 => vec![],
        };

        let mut builder = autocxx_build::Builder::new("src/bindings.rs", &include_dirs)
            .extra_clang_args(&extra_clang_args)
            .build()?;

        builder.flag_if_supported("-std=c++14");

        // Also define for the final compilation
        if matches!(version, CarlaVersion::V0_9_16) {
            builder.define("CARLA_VERSION_0916", None);
        }

        builder.compile("carla_rust");

        // Save generated bindings
        #[cfg(feature = "build-prebuilt")]
        save_bindings()?;

        Ok(())
    }
}

#[cfg(not(feature = "docs-only"))]
fn load_carla_install_dir() -> Result<PathBuf> {
    #[cfg(feature = "build-prebuilt")]
    let install_dir = {
        let src_dir = env::var_os("CARLA_DIR").map(PathBuf::from).ok_or_else(|| {
            anyhow!(
                "CARLA_DIR environment variable is required when using 'build-prebuilt' feature"
            )
        })?;
        build_libcarla_client(&src_dir)?;
        install_libcarla_client(&src_dir)?
    };

    #[cfg(not(feature = "build-prebuilt"))]
    let install_dir = {
        match env::var_os("CARLA_DIR") {
            Some(src_dir) => install_libcarla_client(src_dir)?,
            None => {
                extract_prebuilt_libcarla_client()?
                    .ok_or_else(|| anyhow!("No prebuild binaries for profile {}. \
                                            Please use 'build-prebuilt' feature to compile from source code", *TAG))?
            }
        }
    };

    Ok(install_dir)
}

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
fn extract_prebuilt_libcarla_client() -> Result<Option<PathBuf>> {
    let Some(tarball) = download_tarball()? else {
        return Ok(None);
    };
    let tgt_dir = OUT_DIR.join("libcarla_client");
    extract_tarball(&tarball, &tgt_dir)?;

    let install_dir = tgt_dir.join(&*PREBUILD_NAME);
    Ok(Some(install_dir))
}

#[cfg(all(feature = "build-prebuilt", not(feature = "docs-only")))]
fn build_libcarla_client(src_dir: impl AsRef<Path>) -> Result<()> {
    libcarla_client::clean(&src_dir)?;
    libcarla_client::build(&src_dir)?;
    Ok(())
}

#[cfg(not(feature = "docs-only"))]
fn install_libcarla_client(src_dir: impl AsRef<Path>) -> Result<PathBuf> {
    let install_dir = CARGO_MANIFEST_DIR.join("generated").join(&*PREBUILD_NAME);
    fs::create_dir_all(&install_dir)?;
    libcarla_client::install(&src_dir, &install_dir)?;
    Ok(install_dir)
}

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
fn extract_tarball(tarball: &Path, tgt_dir: &Path) -> io::Result<()> {
    let reader = BufReader::new(File::open(tarball)?);
    let dec = zstd::Decoder::new(reader)?;
    let mut archive = Archive::new(dec);
    archive.unpack(tgt_dir)?;
    Ok(())
}

#[cfg(all(feature = "build-prebuilt", not(feature = "docs-only")))]
fn save_bindings() -> Result<()> {
    let src_file = OUT_DIR.join("autocxx-build-dir/rs/autocxx-ffi-default-gen.rs");
    let tgt_dir = CARGO_MANIFEST_DIR.join("generated");

    // Get CARLA version for versioned bindings (cached from first call)
    let version = carla_version();

    let tgt_file = tgt_dir.join(format!("bindings.{}.rs", version.as_str()));
    fs::create_dir_all(&tgt_dir)?;
    fs::copy(src_file, tgt_file)?;
    Ok(())
}

#[cfg(all(feature = "build-prebuilt", not(feature = "docs-only")))]
fn create_tarball(src_dir: &Path, tarball: &Path) -> Result<()> {
    use std::io::BufWriter;

    let prefix = {
        macro_rules! invalid_file_name {
            ($reason:expr) => {
                anyhow!("Invalid tarlball path '{}'. {}", tarball.display(), $reason)
            };
        }

        let file_name = tarball
            .file_name()
            .ok_or_else(|| invalid_file_name!("It does not have a file name."))?;
        let file_name = file_name.to_str().ok_or_else(|| {
            invalid_file_name!("The file name should only contains ASCII characters.")
        })?;
        file_name
            .strip_suffix(".tar.zstd")
            .ok_or_else(|| anyhow!("The path does not ends in .tar.zstd"))?
    };

    let file_writer = BufWriter::new(File::create(tarball)?);

    let n_threads = std::thread::available_parallelism()?.get();

    let enc = {
        let mut enc = zstd::Encoder::new(file_writer, 3)?;
        enc.include_checksum(true)?;
        enc.multithread(n_threads as u32)?;
        enc.auto_finish()
    };
    let mut tar = tar::Builder::new(enc);
    tar.append_dir_all(format!("{prefix}/"), src_dir)?;
    tar.finish()?;

    Ok(())
}

#[cfg(all(not(feature = "build-prebuilt"), not(feature = "docs-only")))]
fn download_tarball() -> Result<Option<PathBuf>> {
    use std::io::{prelude::*, BufWriter};

    let index_file = CARGO_MANIFEST_DIR.join("index.json5");

    let text = fs::read_to_string(index_file)?;
    let index: HashMap<String, Option<PrebuiltEntry>> = json5::from_str(&text)?;

    // Check if entry exists in index
    let entry = match index.get(&*TAG) {
        Some(Some(entry)) => entry,
        Some(None) => {
            // Entry exists but is null (prebuilt not available yet)
            return Ok(None);
        }
        None => {
            // Entry doesn't exist in index
            return Ok(None);
        }
    };

    // Download the tarball
    let mut reader = ureq::get(&entry.url).call()?.into_reader();
    let mut writer = BufWriter::new(File::create(&*DOWNLOAD_PREBUILT_TARBALL)?);
    io::copy(&mut reader, &mut writer)?;
    writer.flush()?;
    drop(writer);

    // Verify SHA256 checksum
    let mut file = File::open(&*DOWNLOAD_PREBUILT_TARBALL)?;
    let mut hasher = Sha256::new();
    io::copy(&mut file, &mut hasher)?;
    let hash = format!("{:x}", hasher.finalize());

    if hash != entry.sha256 {
        bail!(
            "SHA256 verification failed for downloaded tarball.\nExpected: {}\nGot:      {}",
            entry.sha256,
            hash
        );
    }

    println!(
        "cargo:warning=SHA256 verification passed for {}",
        TAG.as_str()
    );

    Ok(Some(DOWNLOAD_PREBUILT_TARBALL.to_path_buf()))
}

/// Configure LLVM/clang environment variables for compatibility with newer systems.
///
/// On Ubuntu 22.04+ and similar systems with clang >= 14, carla-sys fails to build
/// due to autocxx compatibility issues. This function automatically detects and
/// configures a compatible LLVM version (11, 12, or 13) if available.
///
/// Supported LLVM versions: 11, 12, 13
/// Unsupported LLVM versions: >= 14
fn configure_llvm() {
    // Only configure if user hasn't already set these variables
    if env::var_os("LLVM_CONFIG_PATH").is_some() {
        return;
    }

    // Try versions in order of preference: 13, 12, 11 (newest compatible first)
    for version in [13, 12, 11] {
        let clang_path = PathBuf::from(format!("/usr/bin/clang-{}", version));
        let llvm_lib = PathBuf::from(format!("/usr/lib/llvm-{}/lib", version));
        let llvm_config = PathBuf::from(format!("/usr/bin/llvm-config-{}", version));

        if clang_path.exists() && llvm_lib.exists() && llvm_config.exists() {
            eprintln!(
                "carla-sys: Detected and configured LLVM {} for compatibility (versions 11-13 supported)",
                version
            );

            env::set_var("LLVM_CONFIG_PATH", llvm_config);
            env::set_var("LIBCLANG_PATH", &llvm_lib);
            env::set_var("LIBCLANG_STATIC_PATH", &llvm_lib);
            env::set_var("CLANG_PATH", clang_path);
            return;
        }
    }

    // No compatible LLVM version found - print warning with instructions
    // Using cargo:warning to ensure the message is visible
    println!("cargo:warning=");
    println!("cargo:warning==============================================================================");
    println!("cargo:warning=WARNING: No compatible LLVM version (11-13) detected!");
    println!("cargo:warning==============================================================================");
    println!("cargo:warning=The build requires LLVM/clang versions 11, 12, or 13.");
    println!("cargo:warning=LLVM 14+ is not supported due to autocxx compatibility issues.");
    println!("cargo:warning=");
    println!("cargo:warning=To fix this issue:");
    println!("cargo:warning=  1. Install a compatible LLVM version:");
    println!("cargo:warning=     sudo apt install clang-13 libclang-13-dev");
    println!("cargo:warning=");
    println!("cargo:warning=  2. Set environment variables to use it:");
    println!("cargo:warning=     export LLVM_CONFIG_PATH=/usr/bin/llvm-config-13");
    println!("cargo:warning=     export LIBCLANG_PATH=/usr/lib/llvm-13/lib");
    println!("cargo:warning=     export LIBCLANG_STATIC_PATH=/usr/lib/llvm-13/lib");
    println!("cargo:warning=     export CLANG_PATH=/usr/bin/clang-13");
    println!("cargo:warning=");
    println!("cargo:warning=The build will now continue and may fail if LLVM >= 14 is used.");
    println!("cargo:warning==============================================================================");
    println!("cargo:warning=");
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
enum CarlaVersion {
    #[default]
    V0_9_16,
    V0_9_15,
    V0_9_14,
}

impl FromStr for CarlaVersion {
    type Err = anyhow::Error;

    fn from_str(ver_text: &str) -> Result<Self, Self::Err> {
        let ver = match ver_text {
            "0.9.16" => Self::V0_9_16,
            "0.9.15" => Self::V0_9_15,
            "0.9.14" => Self::V0_9_14,
            _ => bail!(
                "unsupported CARLA version: '{}'. Supported versions: 0.9.14, 0.9.15, 0.9.16",
                ver_text
            ),
        };
        Ok(ver)
    }
}

impl CarlaVersion {
    fn as_str(&self) -> &str {
        match self {
            CarlaVersion::V0_9_16 => "0.9.16",
            CarlaVersion::V0_9_15 => "0.9.15",
            CarlaVersion::V0_9_14 => "0.9.14",
        }
    }
}

fn parse_carla_version() -> Result<CarlaVersion> {
    let Some(ver) = env::var_os("CARLA_VERSION") else {
        // CARLA_VERSION not set, using default version
        return Ok(CarlaVersion::default());
    };
    let Ok(ver) = ver.into_string() else {
        bail!("CARLA_VERSION environment variable contains invalid UTF-8")
    };
    let ver: CarlaVersion = ver.parse().with_context(|| {
        format!(
            "failed to parse CARLA_VERSION='{}' from environment variable",
            ver
        )
    })?;
    Ok(ver)
}
