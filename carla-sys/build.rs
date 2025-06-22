use carla_src::CarlaSource;
use std::{
    env,
    path::{Path, PathBuf},
};

fn main() -> anyhow::Result<()> {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=include/carla_sys_bridge.h");
    println!("cargo:rerun-if-changed=cpp/carla_sys_bridge.cpp");
    println!("cargo:rerun-if-env-changed=CARLA_ROOT");

    // Get CARLA source using carla-src
    let carla_source = CarlaSource::new().expect("Failed to obtain CARLA source");

    println!(
        "Using CARLA source from: {}",
        carla_source.source_dir().display()
    );

    // Build LibCarla if needed
    let (lib_dir, include_dir) = build_libcarla(&carla_source)?;

    // Build CXX bridge
    let mut bridge = cxx_build::bridge("src/ffi.rs");

    // Add include directories
    bridge.include(&include_dir);
    for include in carla_source.include_dirs() {
        bridge.include(include);
    }
    bridge.include("include");

    // Configure compiler
    bridge
        .file("cpp/carla_sys_bridge.cpp")
        .std("c++20") // Use C++20 for std::identity
        .flag_if_supported("-std=c++20")
        .flag("-DCARLA_SYS_BRIDGE")
        .flag("-w"); // Suppress warnings for now

    // Add compiler flags for Linux
    if cfg!(target_os = "linux") {
        bridge.flag("-pthread").flag("-fPIC");
    }

    bridge.compile("carla-sys");

    // Link libraries
    println!("cargo:rustc-link-search=native={}", lib_dir.display());

    // Core CARLA libraries
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

/// Build LibCarla from source
fn build_libcarla(source: &CarlaSource) -> anyhow::Result<(PathBuf, PathBuf)> {
    let out_dir = env::var("OUT_DIR")?;
    let build_dir = Path::new(&out_dir).join("carla-build");

    // Check if we have pre-built libraries in the source
    let prebuilt_lib = source.source_dir().join("install/lib");
    let prebuilt_include = source.source_dir().join("install/include");

    if prebuilt_lib.exists() && prebuilt_include.exists() {
        println!("Using pre-built LibCarla from source directory");
        return Ok((prebuilt_lib, prebuilt_include));
    }

    // Check if already built
    let lib_path = build_dir.join("lib/libcarla-client.a");
    let include_path = build_dir.join("include");

    if lib_path.exists() && include_path.exists() {
        println!("Using cached LibCarla build");
        return Ok((build_dir.join("lib"), include_path));
    }

    // Build LibCarla using CMake
    println!("Building LibCarla from source...");

    // Set locale environment variables to avoid UTF-8 extraction issues
    env::set_var("LC_ALL", "C.UTF-8");
    env::set_var("LANG", "C.UTF-8");

    // The CMakeLists.txt is in the root of CARLA source, not in LibCarla
    let mut cmake_config = cmake::Config::new(source.source_dir());

    // Set environment variables for the cmake configuration as well
    cmake_config.env("LC_ALL", "C.UTF-8");
    cmake_config.env("LANG", "C.UTF-8");

    cmake_config
        .define("BUILD_CARLA_CLIENT", "ON")
        .define("BUILD_CARLA_SERVER", "OFF")
        .define("BUILD_PYTHON_API", "OFF")
        .define("BUILD_CARLA_UNREAL", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_TOOLS", "OFF")
        .define("ENABLE_ROS2", "OFF")
        .define("CMAKE_POLICY_VERSION_MINIMUM", "3.5")
        .define("CMAKE_BUILD_TYPE", "Release")
        .define("CMAKE_INSTALL_PREFIX", &build_dir)
        .define("BOOST_ENABLE_PYTHON", "ON") // Enable Boost.Python since it's required by LibCarla
        .define(
            "BOOST_INCLUDE_LIBRARIES",
            "asio;iterator;python;date_time;geometry;container;variant2;gil",
        ) // Include all needed libs
        .define("BOOST_EXCLUDE_LIBRARIES", "mpi") // Exclude only MPI
        .out_dir(&build_dir)
        .build_target("carla-client");

    // Set C++ standard
    cmake_config.cxxflag("-std=c++14");

    // Add include paths for dependencies
    if source.build_dir().exists() {
        // If dependencies are pre-built in source
        cmake_config.define(
            "BOOST_ROOT",
            source.build_dir().join("boost-1.80.0-c10-install"),
        );
        cmake_config.define("RPCLIB_ROOT", source.build_dir().join("rpclib-c10-install"));
        cmake_config.define(
            "RECAST_ROOT",
            source.build_dir().join("recast-22dfcb-c10-install"),
        );
        cmake_config.define("ZLIB_ROOT", source.build_dir().join("zlib-install"));
        cmake_config.define(
            "LIBPNG_ROOT",
            source.build_dir().join("libpng-1.6.37-install"),
        );
    }

    // Build the carla-client target
    let cmake_dest = cmake_config.build();

    // The actual build directory is in a subdirectory called "build"
    let actual_build_dir = cmake_dest.join("build");

    // Now run the install step
    println!("Installing LibCarla...");
    println!("Install from: {}", actual_build_dir.display());

    let install_status = std::process::Command::new("cmake")
        .args([
            "--install",
            actual_build_dir.to_str().unwrap(),
            "--component",
            "carla-client",
        ])
        .env("LC_ALL", "C.UTF-8")
        .env("LANG", "C.UTF-8")
        .status()?;

    if !install_status.success() {
        anyhow::bail!("Failed to install LibCarla");
    }

    // Verify build succeeded - check in the install prefix directory
    let installed_lib = build_dir.join("lib/libcarla-client.a");
    let installed_include = build_dir.join("include");

    if !installed_lib.exists() {
        anyhow::bail!(
            "LibCarla build failed: library not found at {}",
            installed_lib.display()
        );
    }

    Ok((build_dir.join("lib"), installed_include))
}
