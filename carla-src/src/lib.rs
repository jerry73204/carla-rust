pub mod config;
pub use config::Config;

use anyhow::{ensure, Context, Result};
use cfg_if::cfg_if;
use flate2::bufread::GzDecoder;
use itertools::chain;
use log::info;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, File, OpenOptions},
    io::{self, prelude::*, BufReader, BufWriter},
    path::{Path, PathBuf},
    process::Command,
    sync::Arc,
    thread,
};
use tar::Archive;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarlaSrc {
    pub carla_src_dir: PathBuf,
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
    pub libs: Vec<String>,
}

struct Installation {
    pub include_dirs: Vec<PathBuf>,
    pub lib_dirs: Vec<PathBuf>,
    pub libs: Vec<String>,
}

impl Installation {
    pub fn merge(self, other: Self) -> Self {
        Self {
            include_dirs: chain!(self.include_dirs, other.include_dirs).collect(),
            lib_dirs: chain!(self.lib_dirs, other.lib_dirs).collect(),
            libs: chain!(self.libs, other.libs).collect(),
        }
    }
}

pub(crate) fn build(config: Config) -> CarlaSrc {
    let out_dir = config.out_dir.clone().unwrap_or_else(|| {
        let var = env::var_os("OUT_DIR").expect("OUT_DIR is not set");
        PathBuf::from(var)
    });
    let config = Arc::new(config);
    println!("cargo:rerun-if-changed={}", out_dir.to_str().unwrap());

    let carla_dir = out_dir.join("carla");
    let recast_dir = out_dir.join("recast");
    let rpclib_dir = out_dir.join("rpclib");
    let libpng_dir = out_dir.join("libpng");
    let boost_dir = out_dir.join("boost");

    let carla_task = thread::spawn({
        let config = config.clone();
        move || prepare_carla(&config.carla, &carla_dir).unwrap()
    });
    let recast_task = thread::spawn({
        let config = config.clone();
        move || {
            prepare_recast(
                &config.recast,
                &config.c_compiler,
                &config.cxx_compiler,
                &recast_dir,
            )
            .unwrap()
        }
    });
    let rpclib_task = thread::spawn({
        let config = config.clone();
        move || {
            prepare_rpclib(
                &config.rpclib,
                &config.c_compiler,
                &config.cxx_compiler,
                &rpclib_dir,
            )
            .unwrap()
        }
    });
    let libpng_task = thread::spawn({
        let config = config.clone();
        move || {
            prepare_libpng(
                &config.libpng,
                &config.c_compiler,
                &config.cxx_compiler,
                &libpng_dir,
            )
            .unwrap()
        }
    });
    let boost_task = thread::spawn({
        // let config = config.clone();
        move || prepare_boost(&config.boost, &config.c_compiler, &boost_dir).unwrap()
    });

    let carla_src_dir = carla_task.join().unwrap();
    let recast_install = recast_task.join().unwrap();
    let rpclib_install = rpclib_task.join().unwrap();
    let libpng_install = libpng_task.join().unwrap();
    let boost_install = boost_task.join().unwrap();

    let dep_install = recast_install
        .merge(rpclib_install)
        .merge(libpng_install)
        .merge(boost_install);

    CarlaSrc {
        carla_src_dir,
        include_dirs: dep_install.include_dirs,
        lib_dirs: dep_install.lib_dirs,
        libs: dep_install.libs,
    }
}

fn prepare_carla(config: &config::Carla, work_dir: &Path) -> Result<PathBuf> {
    let src_dir = work_dir.join(&config.dir);
    let source_ready_path = work_dir.join("source_ready");

    // Prepare source files
    skip_or_run(&source_ready_path, || {
        fs::create_dir_all(work_dir)?;
        prepare_tar_gz(&config.url, &config.sha256sum, work_dir)?;
        ensure!(src_dir.is_dir(), "'{}' does not exist", src_dir.display());
        Ok(())
    })?;

    Ok(src_dir)
}

fn prepare_recast(
    config: &config::Recast,
    c_compiler: &str,
    cxx_compiler: &str,
    work_dir: &Path,
) -> Result<Installation> {
    let source_ready_path = work_dir.join("source_ready");
    let build_ready_path = work_dir.join("build_ready");
    let src_dir = work_dir.join(&config.dir);
    let install_dir = work_dir.join("install");

    skip_or_run(&build_ready_path, || {
        // Prepare source files
        skip_or_run(&source_ready_path, || {
            fs::create_dir_all(work_dir)?;
            prepare_tar_gz(&config.url, &config.sha256sum, work_dir)?;
            ensure!(src_dir.is_dir(), "'{}' does not exist", src_dir.display());
            Ok(())
        })?;

        // Build
        info!("Building recast");
        fs::create_dir_all(&install_dir)?;
        cmake::Config::new(&src_dir)
            .generator("Ninja")
            .define("CMAKE_C_COMPILER", c_compiler)
            .define("CMAKE_CXX_COMPILER", cxx_compiler)
            .define("CMAKE_CXX_FLAGS", "-std=c++14 -fPIC")
            .define("RECASTNAVIGATION_DEMO", "False")
            .define("RECASTNAVIGATION_TEST", "False")
            .out_dir(&install_dir)
            .build();
        Ok(())
    })?;

    Ok(Installation {
        include_dirs: vec![src_dir.join("include")],
        lib_dirs: vec![src_dir.join("lib")],
        libs: vec![
            "static=Recast".to_string(),
            "static=Detour".to_string(),
            "static=DetourCrowd".to_string(),
            // "static=DebugUtils".to_string(),
            // "static=DetourTileCache".to_string(),
        ],
    })
}

fn prepare_rpclib(
    config: &config::Rpclib,
    c_compiler: &str,
    cxx_compiler: &str,
    work_dir: &Path,
) -> Result<Installation> {
    let source_ready_path = work_dir.join("source_ready");
    let build_ready_path = work_dir.join("build_ready");
    let src_dir = work_dir.join(&config.dir);
    let install_dir = work_dir.join("install");

    skip_or_run(&build_ready_path, || {
        // Prepare source files
        skip_or_run(&source_ready_path, || {
            fs::create_dir_all(work_dir)?;
            prepare_tar_gz(&config.url, &config.sha256sum, work_dir)?;
            ensure!(src_dir.is_dir(), "'{}' does not exist", src_dir.display());
            Ok(())
        })?;

        // Build
        info!("Building rpclib");

        fs::create_dir_all(&install_dir)?;
        cmake::Config::new(&src_dir)
            .generator("Ninja")
            .define("CMAKE_C_COMPILER", c_compiler)
            .define("CMAKE_CXX_COMPILER", cxx_compiler)
            .define("CLANG_DEFAULT_CXX_STDLIB", "libc++")
            .define(
                "CMAKE_CXX_FLAGS",
                "-fPIC \
                 -std=c++14 \
                 -DBOOST_NO_EXCEPTIONS \
                 -DASIO_NO_EXCEPTIONS",
            )
            .out_dir(&install_dir)
            .build();
        Ok(())
    })?;

    Ok(Installation {
        include_dirs: vec![src_dir.join("include")],
        lib_dirs: vec![src_dir.join("lib")],
        libs: vec!["static=rpc".to_string()],
    })
}

fn prepare_libpng(
    config: &config::Libpng,
    c_compiler: &str,
    cxx_compiler: &str,
    work_dir: &Path,
) -> Result<Installation> {
    let source_ready_path = work_dir.join("source_ready");
    let build_ready_path = work_dir.join("build_ready");
    let src_dir = work_dir.join(&config.dir);
    let install_dir = work_dir.join("install");

    skip_or_run(&build_ready_path, || {
        // Prepare source files
        skip_or_run(&source_ready_path, || {
            fs::create_dir_all(work_dir)?;
            prepare_tar_gz(&config.url, &config.sha256sum, work_dir)?;
            ensure!(src_dir.is_dir(), "'{}' does not exist", src_dir.display());
            Ok(())
        })?;

        // Build
        info!("Building libpng");
        fs::create_dir_all(&install_dir)?;
        autotools::Config::new(&src_dir)
            .out_dir(&install_dir)
            .enable_static()
            .env("CC", c_compiler)
            .env("CXX", cxx_compiler)
            .build();
        Ok(())
    })?;

    Ok(Installation {
        include_dirs: vec![src_dir.join("include")],
        lib_dirs: vec![src_dir.join("lib")],
        libs: vec!["static=png".to_string()],
    })
}

fn prepare_boost(
    config: &config::Boost,
    c_compiler: &str,
    work_dir: &Path,
) -> Result<Installation> {
    let config::Boost {
        bootstrap_toolset,
        b2_toolset,
        ..
    } = config;
    let n_procs = thread::available_parallelism().unwrap();
    let source_ready_path = work_dir.join("source_ready");
    let build_ready_path = work_dir.join("build_ready");
    let src_dir = work_dir.join(&config.dir);
    let install_dir = work_dir.join("install");

    skip_or_run(&build_ready_path, || {
        // Prepare source files
        skip_or_run(&source_ready_path, || {
            fs::create_dir_all(work_dir)?;
            prepare_tar_gz(&config.url, &config.sha256sum, work_dir)?;
            ensure!(src_dir.is_dir(), "'{}' does not exist", src_dir.display());
            Ok(())
        })?;

        // Start to build
        info!("Building boost");
        fs::create_dir_all(&install_dir)?;

        // Bootstrap boost
        {
            let bootstrap_name = {
                cfg_if! {
                    if #[cfg(unix)] {
                        "bootstrap.sh"
                    } else if #[cfg(windows)] {
                        "bootstrap.bat"
                    } else {
                        warn!("Not a UNIX system. Try to run bootstrap.sh from boost");
                        "bootstrap.sh"
                    }
                }
            };

            let bootstrap_path = src_dir.join(bootstrap_name);
            Command::new(bootstrap_path)
                .current_dir(&src_dir)
                .args([
                    &format!("--with-toolset={bootstrap_toolset}"),
                    "--prefix=../boost-install",
                    "--with-libraries=filesystem,system,program_options",
                ])
                .status()
                .with_context(|| format!("Failed to run {bootstrap_name}"))?;
        }

        // Write user-config.jam
        {
            let user_config_path = src_dir
                .join("tools")
                .join("build")
                .join("src")
                .join("user-config.jam");
            let mut writer = OpenOptions::new()
                .create(true)
                .append(true)
                .write(true)
                .open(&user_config_path)?;
            writeln!(writer, "using clang : 8.0 : {c_compiler} ;")?;
        }

        // Run b2
        {
            let b2_name = {
                cfg_if! {
                    if #[cfg(unix)] {
                        "b2"
                    } else if #[cfg(windows)] {
                        "b2.exe"
                    } else {
                        warn!("Not a UNIX system. Try to run 'b2' binary");
                        "b2"
                    }
                }
            };

            let b2_path = src_dir.join(b2_name);
            Command::new(&b2_path)
                .current_dir(&src_dir)
                .args([
                    &format!("toolset={b2_toolset}"),
                    "cxxflags=-fPIC -std=c++14 -DBOOST_ERROR_CODE_HEADER_ONLY",
                    &format!("--prefix={}", install_dir.to_str().unwrap()),
                    &format!("-j{}", n_procs),
                    "stage",
                    "release",
                ])
                .status()
                .with_context(|| format!("Failed to run {b2_name}"))?;
            Command::new(&b2_path)
                .current_dir(&src_dir)
                .args([
                    "toolset=clang-8.0",
                    "cxxflags=-fPIC -std=c++14 -DBOOST_ERROR_CODE_HEADER_ONLY",
                    &format!("--prefix={}", install_dir.to_str().unwrap()),
                    &format!("-j{}", n_procs),
                    "install",
                ])
                .status()
                .with_context(|| format!("Failed to run {b2_name}"))?;
        }

        Ok(())
    })?;

    Ok(Installation {
        include_dirs: vec![src_dir.join("include")],
        lib_dirs: vec![src_dir.join("lib")],
        libs: vec!["static=boost_filesystem".to_string()],
    })
}

fn prepare_tar_gz(url: &str, sha256_digest: &[u8], dir: &Path) -> Result<()> {
    let tarball_path = dir.join("source.tar.gz");

    // Download source file if it's not ready
    let is_source_ready = tarball_path.is_file() && sha256_digest == sha256sum(&tarball_path)?;

    if is_source_ready {
        info!("Source file is ready. Skip downloading.");
    } else {
        info!("Downloading file from '{}'", url);

        let mut tarball_file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(&tarball_path)?;

        let mut writer = BufWriter::new(&mut tarball_file);
        let mut reader = ureq::AgentBuilder::new()
            .tls_connector(Arc::new(native_tls::TlsConnector::new()?))
            .build()
            .get(url)
            .call()?
            .into_reader();
        io::copy(&mut reader, &mut writer)?;
        writer.flush()?;
    }

    // Verify file
    {
        let digest = sha256sum(&tarball_path)?;
        ensure!(
            sha256_digest == digest,
            "Checksum mismatch: expect {:X?}, but get {:x?}",
            sha256_digest,
            digest
        );
    }

    // unpack tar.gz
    {
        info!("Unpacking {}", tarball_path.display());
        let mut tarball_file = BufReader::new(File::open(&tarball_path)?);
        let mut reader = BufReader::new(&mut tarball_file);
        let tar = GzDecoder::new(&mut reader);
        let mut archive = Archive::new(tar);
        archive.unpack(dir)?;
    }

    Ok(())
}

fn sha256sum(path: &Path) -> Result<[u8; 32]> {
    let mut hasher = Sha256::new();
    let mut reader = BufReader::new(File::open(path)?);

    loop {
        let mut buf = [0u8; 8192];
        let len = reader.read(&mut buf)?;
        if len == 0 {
            break;
        }
        hasher.update(&buf[0..len]);
    }

    let hash = hasher.finalize();
    Ok(hash.as_slice().try_into().unwrap())
}

fn touch(path: &Path) -> Result<()> {
    OpenOptions::new().create(true).write(true).open(path)?;
    Ok(())
}

fn skip_or_run<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    with_target(target_path, || {
        let output = (callback)()?;
        touch(target_path)?;
        Ok(output)
    })
}

fn with_target<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    with_targets(&[target_path], callback)
}

fn with_targets<T, F>(target_paths: &[&Path], callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    let ok = target_paths.iter().all(|path| path.exists());
    if ok {
        return Ok(None);
    }

    (callback)().map(Some)
}

// fn llvm8_include_dir() -> Result<PathBuf> {
//     let mut stdout = Command::new("llvm-config-8")
//         .arg("--includedir")
//         .output()
//         .with_context(|| "Is llvm-config-8 installed on your system?")?
//         .stdout;

//     // Remove a trailing '\n'
//     assert!(stdout.pop() == Some(b'\n'));

//     let path = bytes_to_path(stdout)?;
//     Ok(path)
// }

// fn llvm8_lib_dir() -> Result<PathBuf> {
//     let mut stdout = Command::new("llvm-config-8")
//         .arg("--libdir")
//         .output()
//         .with_context(|| "Is llvm-config-8 installed on your system?")?
//         .stdout;

//     // Remove a trailing '\n'
//     assert!(stdout.pop() == Some(b'\n'));

//     let path = bytes_to_path(stdout)?;
//     Ok(path)
// }

// fn bytes_to_path(bytes: Vec<u8>) -> Result<PathBuf> {
//     let path = {
//         cfg_if! {
//             if #[cfg(unix)] {
//                 use std::os::unix::ffi::OsStringExt;
//                 PathBuf::from(OsString::from_vec(bytes))
//             } else if #[cfg(windows)] {
//                 use std::os::windows::ffi::OsStringExt;
//                 let bytes: &[u16] = safe_transmute::transmute_many_pedantic(&stdout)?;
//                 PathBuf::from(OsString::from_wide(bytes))
//             } else if #[cfg(wasm)] {
//                 use std::os::wasi::ffi::OsStringExt;
//                 PathBuf::from(OsString::from_vec(bytes))
//             }
//         }
//     };
//     Ok(path)
// }
