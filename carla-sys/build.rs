use std::{
    env,
    path::{Path, PathBuf},
};

fn main() {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-env-changed=CARLA_CIR");

    // Skip build if docs-only feature presents.
    #[cfg(feature = "docs-only")]
    return;

    // Prepare Carla source code
    let carla_dir = match env::var_os("CARLA_DIR") {
        Some(dir) => {
            // If CARLA_DIR env var is set, set the rerun checkpoint on the directory.
            let dir = PathBuf::from(dir);
            println!("cargo:rerun-if-changed={}", dir.display());
            dir
        }
        None => carla_bin::carla_dir().unwrap().to_path_buf(),
    };

    // Prepare paths
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let csrc_dir = manifest_dir.join("csrc");
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

    // link libcarla_client library
    println!(
        "cargo:rustc-link-search=native={}",
        carla_lib_dir.to_str().unwrap()
    );
    println!("cargo:rustc-link-lib=static=carla_client");

    // Add dependency include and library dirs
    let mut include_dirs = vec![];
    include_dirs.extend([csrc_dir, carla_source_dir, carla_third_party_dir]);
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
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is not set"));
    autocxx_build::Builder::new("src/ffi.rs", &include_dirs)
        .build()
        .unwrap()
        .flag_if_supported("-std=c++14")
        .compile("carla_rust");

    // Save generated bindings
    save_bindings(&out_dir, &manifest_dir);
}

fn save_bindings(out_dir: &Path, manifest_dir: &Path) {
    use std::fs;
    let src_file = out_dir.join("autocxx-build-dir/rs/autocxx-ffi-default-gen.rs");
    let tgt_dir = manifest_dir.join("generated");
    let tgt_file = tgt_dir.join("bindings.rs");
    fs::create_dir_all(&tgt_dir).unwrap();
    fs::copy(src_file, tgt_file).unwrap();
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
