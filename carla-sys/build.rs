use anyhow::{anyhow, Result};
use carla_src::libcarla_client;
use once_cell::sync::Lazy;
#[allow(unused)]
use std::path::Path;
use std::{
    collections::HashMap,
    env, fs,
    fs::File,
    io::{self, BufReader},
    path::PathBuf,
};
use tar::Archive;

static TAG: Lazy<String> = Lazy::new(|| {
    format!(
        "{}-{}",
        libcarla_client::VERSION,
        env::var("TARGET").unwrap(),
    )
});
static OUT_DIR: Lazy<PathBuf> = Lazy::new(|| PathBuf::from(env::var_os("OUT_DIR").unwrap()));
static PREBUILD_NAME: Lazy<String> = Lazy::new(|| format!("libcarla_client.{}", *TAG));
static PREBUILT_TARBALL: Lazy<PathBuf> = Lazy::new(|| {
    let file_name = format!("{}.tar.zstd", *PREBUILD_NAME);
    CARGO_MANIFEST_DIR.join("generated").join(file_name)
});
static CARGO_MANIFEST_DIR: Lazy<&Path> = Lazy::new(|| Path::new(env!("CARGO_MANIFEST_DIR")));

fn main() -> Result<()> {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-env-changed=CARLA_DIR");

    // Skip build if docs-only feature presents.
    #[cfg(feature = "docs-only")]
    return;

    // Prepare CARLA installation
    let install_dir = load_carla_install_dir()?;
    let carla_lib_dir = install_dir.join("lib");
    let carla_include_dir = install_dir.join("include");

    #[cfg(feature = "save-lib")]
    create_tarball(&install_dir, &*PREBUILT_TARBALL)?;

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

    autocxx_build::Builder::new("src/ffi.rs", &include_dirs)
        .build()?
        .flag_if_supported("-std=c++14")
        .compile("carla_rust");

    // Save generated bindings
    #[cfg(feature = "save-bindgen")]
    save_bindings();

    Ok(())
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
    let mut writer = BufWriter::new(File::create(&*PREBUILT_TARBALL)?);
    io::copy(&mut reader, &mut writer)?;
    writer.flush()?;

    Ok(Some(PREBUILT_TARBALL.to_path_buf()))
}
