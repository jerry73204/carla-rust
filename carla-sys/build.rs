use carla_bin::CarlaBuild;
use carla_src::probe;
#[allow(unused)]
use std::path::Path;
use std::{env, path::PathBuf};

fn main() {
    // Set rerun triggers
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-env-changed=CARLA_DIR");

    // Skip build if docs-only feature presents.
    #[cfg(feature = "docs-only")]
    return;

    // Prepare Carla source code
    let (carla_include_dirs, carla_lib_dirs) = match env::var_os("CARLA_DIR") {
        Some(src_dir) => {
            // If CARLA_DIR env var is set, set the rerun checkpoint on the directory.
            let src_dir = PathBuf::from(src_dir);
            println!("cargo:rerun-if-changed={}", src_dir.display());

            let probe = probe(&src_dir);
            let include_dirs = probe.include_dirs.into_vec();
            let lib_dirs = probe.lib_dirs.into_vec();
            (include_dirs, lib_dirs)
        }
        None => {
            let CarlaBuild {
                include_dirs,
                lib_dirs,
                ..
            } = carla_bin::build_carla().unwrap();
            (include_dirs.clone(), lib_dirs.clone())
        }
    };

    // Add library search paths
    for dir in &carla_lib_dirs {
        println!("cargo:rustc-link-search=native={}", dir.to_str().unwrap());
    }

    // Link libraries
    for lib in carla_src::libcarla_client::LIBS {
        println!("cargo:rustc-link-lib={lib}");
    }

    // Generate bindings
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let csrc_dir = manifest_dir.join("csrc");
    let include_dirs = {
        let mut carla_include_dirs = carla_include_dirs;
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
    save_bindings(&manifest_dir);
}

#[cfg(feature = "save-bindgen")]
fn save_bindings(manifest_dir: &Path) {
    use std::fs;
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let src_file = out_dir.join("autocxx-build-dir/rs/autocxx-ffi-default-gen.rs");
    let tgt_dir = manifest_dir.join("generated");
    let tgt_file = tgt_dir.join("bindings.rs");
    fs::create_dir_all(&tgt_dir).unwrap();
    fs::copy(src_file, tgt_file).unwrap();
}
