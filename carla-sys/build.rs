use std::{
    env,
    fs::{self, OpenOptions},
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-env-changed=CARLA_CIR");

    // Prepare Carla source code
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is not set"));
    let carla_dir = match env::var_os("CARLA_DIR") {
        Some(dir) => {
            // If CARLA_DIR env var is set, set the rerun checkpoint on the directory.
            let dir = PathBuf::from(dir);
            println!("cargo:rerun-if-changed={}", dir.display());
            dir
        }
        None => {
            let prepare_dir = out_dir.join("prepare");
            fs::create_dir_all(&prepare_dir).unwrap();

            carla_src::Config {
                out_dir: Some(prepare_dir),
                ..Default::default()
            }
            .load()
            .unwrap()
        }
    };

    // Prepare paths
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let csrc_dir = manifest_dir.join("csrc");
    // let carla_dir = env::var_os("CARLA_DIR").expect("CARLA_DIR is not set");
    // let carla_dir = PathBuf::from(carla_dir);
    let libcarla_dir = carla_dir.join("LibCarla");
    let build_dir = carla_dir.join("Build");
    let carla_source_dir = libcarla_dir.join("source");
    let carla_third_party_dir = carla_source_dir.join("third-party");
    let carla_lib_dir = build_dir
        .join("libcarla-client-build.release")
        .join("LibCarla")
        .join("cmake")
        .join("client");
    let recast_dir = build_dir.join("recast-0b13b0-c8-install");
    let rpclib_dir = build_dir.join("rpclib-v2.2.1_c5-c8-libstdcxx-install");
    let boost_dir = build_dir.join("boost-1.80.0-c8-install");
    let libpng_dir = build_dir.join("libpng-1.6.37-install");

    // Build LibCarla.client library
    build_libcarla_client_library(&out_dir, &carla_dir);

    let mut include_dirs = vec![];

    // Set include dirs
    include_dirs.extend([csrc_dir, carla_source_dir, carla_third_party_dir]);

    // link libcarla_client library
    println!(
        "cargo:rustc-link-search=native={}",
        carla_lib_dir.to_str().unwrap()
    );
    println!("cargo:rustc-link-lib=static=carla_client");

    // Add dependency include and library dirs
    add_library(
        &recast_dir,
        &[
            "static=Recast",
            "static=Detour",
            "static=DetourCrowd",
            // "static=DetourTileCache",
            // "static=DebugUtils",
        ],
        &mut include_dirs,
    );
    add_library(&rpclib_dir, &["static=rpc"], &mut include_dirs);
    add_library(&libpng_dir, &["static=png"], &mut include_dirs);
    add_library(&boost_dir, &["static=boost_filesystem"], &mut include_dirs);

    // Generate bindings
    autocxx_build::Builder::new("src/lib.rs", &include_dirs)
        .build()
        .unwrap()
        .flag_if_supported("-std=c++14")
        .compile("carla_rust");
}

fn build_libcarla_client_library(out_dir: &Path, carla_src_dir: &Path) {
    let ready_file = out_dir.join("carla_build_ready");
    if ready_file.exists() {
        return;
    }

    // make LibCarla.client.release
    Command::new("make")
        .arg("LibCarla.client.release")
        .current_dir(carla_src_dir)
        .status()
        .expect("Failed to run `make LibCarla.client`");

    // Mark the step is done
    touch(&ready_file);
}

fn add_library(dir: &Path, libs: &[&str], include_dirs: &mut Vec<PathBuf>) {
    let include_dir = dir.join("include");
    let lib_dir = dir.join("lib");

    include_dirs.push(include_dir);
    println!(
        "cargo:rustc-link-search=native={}",
        lib_dir.to_str().unwrap()
    );

    for lib in libs {
        println!("cargo:rustc-link-lib={lib}");
    }
}

fn touch(path: &Path) {
    OpenOptions::new()
        .create(true)
        .write(true)
        .open(path)
        .unwrap();
}
