use std::{env, path::PathBuf};

fn main() -> anyhow::Result<()> {
    // Find CARLA installation directory
    let carla_root = env::var("CARLA_ROOT")
        .or_else(|_| env::var("CARLA_DIR"))
        .unwrap_or_else(|_| {
            // Default to the carla-simulator install directory
            let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
            let manifest_path = PathBuf::from(&manifest_dir);
            let project_root = manifest_path.parent().unwrap();
            project_root
                .join("carla-simulator/install")
                .to_string_lossy()
                .to_string()
        });

    let carla_root = PathBuf::from(&carla_root);
    if !carla_root.exists() {
        panic!(
            "CARLA installation not found at {:?}. Please set CARLA_ROOT or CARLA_DIR environment variable.",
            carla_root
        );
    }

    let carla_include = carla_root.join("include");
    let carla_lib = carla_root.join("lib");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=include/carla_cxx_bridge.h");
    println!("cargo:rerun-if-changed=cpp/carla_cxx_bridge.cpp");

    // Build CXX bridge
    let mut bridge = cxx_build::bridge("src/lib.rs");

    bridge
        .file("cpp/carla_cxx_bridge.cpp")
        .include(&carla_include)
        .include("include")
        .std("c++20") // Use C++20 for std::identity
        .flag_if_supported("-std=c++20")
        .flag("-DCARLA_CXX_BRIDGE")
        .flag("-w"); // Suppress warnings for now

    // Add compiler flags for Linux
    if cfg!(target_os = "linux") {
        bridge.flag("-pthread").flag("-fPIC");
    }

    bridge.compile("carla-cxx");

    // Link CARLA libraries
    println!("cargo:rustc-link-search=native={}", carla_lib.display());

    // Core CARLA libraries (based on actual files in lib directory)
    println!("cargo:rustc-link-lib=static=carla-client");
    println!("cargo:rustc-link-lib=static=rpc");

    // Navigation libraries
    println!("cargo:rustc-link-lib=static=Recast");
    println!("cargo:rustc-link-lib=static=Detour");
    println!("cargo:rustc-link-lib=static=DetourCrowd");

    // Image libraries
    println!("cargo:rustc-link-lib=static=png16");

    // System libraries
    println!("cargo:rustc-link-lib=static=z");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=stdc++");

    Ok(())
}
