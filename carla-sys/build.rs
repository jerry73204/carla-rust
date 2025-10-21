use anyhow::{anyhow, Result};
use carla_src::libcarla_client;
use once_cell::sync::Lazy;
#[allow(unused)]
use std::path::Path;
use std::{env, fs, path::PathBuf};

#[cfg(not(feature = "build-lib"))]
use std::{collections::HashMap, io};

#[cfg(any(not(feature = "build-lib"), feature = "save-lib"))]
use std::{fs::File, io::BufReader};

#[cfg(not(feature = "build-lib"))]
use tar::Archive;

static TAG: Lazy<String> = Lazy::new(|| {
    format!(
        "{}-{}",
        libcarla_client::VERSION,
        env::var("TARGET").expect("TARGET environment variable not set"),
    )
});
static OUT_DIR: Lazy<PathBuf> = Lazy::new(|| {
    PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR environment variable not set"))
});
static PREBUILD_NAME: Lazy<String> = Lazy::new(|| format!("libcarla_client.{}", *TAG));
#[cfg(feature = "save-lib")]
static SAVE_PREBUILT_TARBALL: Lazy<PathBuf> = Lazy::new(|| {
    let file_name = format!("{}.tar.zstd", *PREBUILD_NAME);
    GENERATED_DIR.join(file_name)
});
#[cfg(not(feature = "build-lib"))]
static DOWNLOAD_PREBUILT_TARBALL: Lazy<PathBuf> = Lazy::new(|| {
    let file_name = format!("{}.tar.zstd", *PREBUILD_NAME);
    OUT_DIR.join(file_name)
});
static CARGO_MANIFEST_DIR: Lazy<&Path> = Lazy::new(|| Path::new(env!("CARGO_MANIFEST_DIR")));
#[cfg(feature = "save-lib")]
static GENERATED_DIR: Lazy<PathBuf> = Lazy::new(|| CARGO_MANIFEST_DIR.join("generated"));

fn main() -> Result<()> {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/bindings.rs");
    println!("cargo:rerun-if-env-changed=CARLA_DIR");

    // Configure LLVM/clang for compatibility with newer systems (Ubuntu 22.04+)
    configure_llvm();

    // Skip build if docs-only feature presents.
    #[cfg(feature = "docs-only")]
    {
        return Ok(());
    }

    #[cfg(not(feature = "docs-only"))]
    {
        // Prepare CARLA installation
        let install_dir = load_carla_install_dir()?;
        let carla_lib_dir = install_dir.join("lib");
        let carla_include_dir = install_dir.join("include");

        #[cfg(feature = "save-lib")]
        create_tarball(&install_dir, &*SAVE_PREBUILT_TARBALL)?;

        // return;

        // Add library search paths
        println!("cargo:rustc-link-search=native={}", carla_lib_dir.display());

        // Link libraries
        for lib in libcarla_client::LIBS {
            println!("cargo:rustc-link-lib={lib}");
        }

        // Generate bindings
        let csrc_dir = CARGO_MANIFEST_DIR.join("csrc");
        let include_dirs = [carla_include_dir, csrc_dir];

        autocxx_build::Builder::new("src/bindings.rs", &include_dirs)
            .build()?
            .flag_if_supported("-std=c++14")
            .compile("carla_rust");

        // Save generated bindings
        #[cfg(feature = "save-bindgen")]
        save_bindings()?;

        Ok(())
    }
}

fn load_carla_install_dir() -> Result<PathBuf> {
    #[cfg(feature = "build-lib")]
    let install_dir = {
        let src_dir = env::var_os("CARLA_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| CARGO_MANIFEST_DIR.join("..").join("carla-simulator"));
        build_libcarla_client(&src_dir)?;
        install_libcarla_client(&src_dir)?
    };

    #[cfg(not(feature = "build-lib"))]
    let install_dir = {
        match env::var_os("CARLA_DIR") {
            Some(src_dir) => install_libcarla_client(src_dir)?,
            None => {
                extract_prebuilt_libcarla_client()?
                    .ok_or_else(|| anyhow!("No prebuild binaries for profile {}. \
                                            Please use 'build-lib' feature to compile from source code", *TAG))?
            }
        }
    };

    Ok(install_dir)
}

#[cfg(not(feature = "build-lib"))]
fn extract_prebuilt_libcarla_client() -> Result<Option<PathBuf>> {
    let Some(tarball) = download_tarball()? else {
        return Ok(None);
    };
    let tgt_dir = OUT_DIR.join("libcarla_client");
    extract_tarball(&tarball, &tgt_dir)?;

    let install_dir = tgt_dir.join(&*PREBUILD_NAME);
    Ok(Some(install_dir))
}

#[cfg(feature = "build-lib")]
fn build_libcarla_client(src_dir: impl AsRef<Path>) -> Result<()> {
    libcarla_client::clean(&src_dir)?;
    libcarla_client::build(&src_dir)?;
    Ok(())
}

fn install_libcarla_client(src_dir: impl AsRef<Path>) -> Result<PathBuf> {
    let install_dir = CARGO_MANIFEST_DIR.join("generated").join(&*PREBUILD_NAME);
    fs::create_dir_all(&install_dir)?;
    libcarla_client::install(&src_dir, &install_dir)?;
    Ok(install_dir)
}

#[cfg(not(feature = "build-lib"))]
fn extract_tarball(tarball: &Path, tgt_dir: &Path) -> io::Result<()> {
    let reader = BufReader::new(File::open(tarball)?);
    let dec = zstd::Decoder::new(reader)?;
    let mut archive = Archive::new(dec);
    archive.unpack(tgt_dir)?;
    Ok(())
}

#[cfg(feature = "save-bindgen")]
fn save_bindings() -> Result<()> {
    let src_file = OUT_DIR.join("autocxx-build-dir/rs/autocxx-ffi-default-gen.rs");
    let tgt_dir = CARGO_MANIFEST_DIR.join("generated");
    let tgt_file = tgt_dir.join("bindings.rs");
    fs::create_dir_all(&tgt_dir)?;
    fs::copy(src_file, tgt_file)?;
    Ok(())
}

#[cfg(feature = "save-lib")]
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

#[cfg(not(feature = "build-lib"))]
fn download_tarball() -> Result<Option<PathBuf>> {
    use std::io::{prelude::*, BufWriter};

    let index_file = CARGO_MANIFEST_DIR.join("index.json5");

    let text = fs::read_to_string(index_file)?;
    let index: HashMap<String, String> = json5::from_str(&text)?;
    let Some(url) = index.get(&*TAG) else {
        return Ok(None);
    };

    let mut reader = ureq::get(url).call()?.into_reader();
    let mut writer = BufWriter::new(File::create(&*DOWNLOAD_PREBUILT_TARBALL)?);
    io::copy(&mut reader, &mut writer)?;
    writer.flush()?;

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
