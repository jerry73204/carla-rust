use carla_src::{libcarla_client, probe};
use once_cell::sync::Lazy;
#[allow(unused)]
use std::path::Path;
use std::{
    env, fs,
    fs::File,
    io::{BufReader, BufWriter},
    path::PathBuf,
};
use tar::Archive;

static TAG: Lazy<String> = Lazy::new(|| {
    format!(
        "{}.{}.{}",
        env::var("CARGO_PKG_VERSION").unwrap(),
        env::var("TARGET").unwrap(),
        env::var("PROFILE").unwrap()
    )
});
static OUT_DIR: Lazy<PathBuf> = Lazy::new(|| PathBuf::from(env::var_os("OUT_DIR").unwrap()));
static PREBUILT_TARBALL: Lazy<PathBuf> = Lazy::new(|| {
    let file_name = format!("prebuild.{}.tar.zstd", *TAG);
    Path::new(CARGO_MANIFEST_DIR)
        .join("generated")
        .join(file_name)
});
const CARGO_MANIFEST_DIR: &str = env!("CARGO_MANIFEST_DIR");

struct CarlaDirs {
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
}

fn main() {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-env-changed=CARLA_DIR");

    // Skip build if docs-only feature presents.
    #[cfg(feature = "docs-only")]
    return;

    // Prepare Carla source code
    let CarlaDirs {
        include_dirs,
        lib_dirs,
    } = load_carla_dirs();

    // Add library search paths
    for dir in &lib_dirs {
        println!("cargo:rustc-link-search=native={}", dir.to_str().unwrap());
    }

    // Link libraries
    for lib in carla_src::libcarla_client::LIBS {
        println!("cargo:rustc-link-lib={lib}");
    }

    // Generate bindings
    let csrc_dir = Path::new(CARGO_MANIFEST_DIR).join("csrc");
    let include_dirs = {
        let mut carla_include_dirs = include_dirs;
        carla_include_dirs.push(csrc_dir);
        carla_include_dirs
    };

    autocxx_build::Builder::new("src/ffi.rs", &include_dirs)
        .build()
        .unwrap()
        .flag_if_supported("-std=c++14")
        .compile("carla_rust");

    // Save generated bindings
    #[cfg(feature = "save-bindgen")]
    save_bindings();
}

fn load_carla_dirs() -> CarlaDirs {
    let install_dir = match env::var_os("CARLA_DIR") {
        Some(dir) => PathBuf::from(dir),

        #[cfg(feature = "save-built-lib")]
        None => {
            let dir = build_libcarla_client();
            create_tarball(&dir, &PREBUILT_TARBALL).unwrap();
            dir
        }

        #[cfg(not(feature = "save-built-lib"))]
        None => match extract_prebuilt_libcarla_client() {
            Some(dir) => dir,
            None => build_libcarla_client(),
        },
    };

    probe_install_dir(&install_dir)
}

fn probe_install_dir(install_dir: &Path) -> CarlaDirs {
    // If CARLA_DIR env var is set, set the rerun checkpoint on the directory.
    let src_dir = PathBuf::from(install_dir);
    println!("cargo:rerun-if-changed={}", src_dir.display());

    let probe = probe(&src_dir);
    let include_dirs = probe.include_dirs.into_vec();
    let lib_dirs = probe.lib_dirs.into_vec();

    CarlaDirs {
        include_dirs,
        lib_dirs,
    }
}

fn extract_prebuilt_libcarla_client() -> Option<PathBuf> {
    if !PREBUILT_TARBALL.exists() {
        return None;
    }
    let tgt_dir = OUT_DIR.join("prebuild");
    extract_tarball(&PREBUILT_TARBALL, &tgt_dir).unwrap();
    Some(tgt_dir)
}

fn build_libcarla_client() -> PathBuf {
    let cargo_manifest_dir = Path::new(CARGO_MANIFEST_DIR);
    let src_dir = cargo_manifest_dir.join("..").join("carla-simulator");
    let install_dir = cargo_manifest_dir
        .join("generated")
        .join("libcarla_install");
    fs::create_dir_all(&install_dir).unwrap();
    libcarla_client::clean(&src_dir).unwrap();
    libcarla_client::build(&src_dir).unwrap();
    libcarla_client::install(&src_dir, &install_dir).unwrap();
    install_dir
}

fn extract_tarball(src_dir: &Path, tarball: &Path) -> std::io::Result<()> {
    let writer = BufWriter::new(File::create(tarball)?);
    let enc = zstd::Encoder::new(writer, 4)?.finish()?;
    let mut tar = tar::Builder::new(enc);
    tar.append_dir_all(".", src_dir)?;
    Ok(())
}

fn create_tarball(tarball: &Path, tgt_dir: &Path) -> std::io::Result<()> {
    let reader = BufReader::new(File::open(tarball)?);
    let tar = zstd::Decoder::new(reader)?.finish();
    let mut archive = Archive::new(tar);
    archive.unpack(tgt_dir)?;
    Ok(())
}

#[cfg(feature = "save-bindgen")]
fn save_bindings() {
    use std::fs;
    let src_file = OUT_DIR.join("autocxx-build-dir/rs/autocxx-ffi-default-gen.rs");
    let tgt_dir = CARGO_MAMIFEST_DIR.join("generated");
    let tgt_file = tgt_dir.join("bindings.rs");
    fs::create_dir_all(&tgt_dir).unwrap();
    fs::copy(src_file, tgt_file).unwrap();
}
