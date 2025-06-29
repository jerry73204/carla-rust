use std::{
    env,
    path::{Path, PathBuf},
};

fn main() -> anyhow::Result<()> {
    // Handle docs-only feature first
    if cfg!(feature = "docs-only") {
        println!("Building in docs-only mode - skipping C++ compilation");
        return Ok(());
    }

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=include/carla_sys_bridge.h");
    println!("cargo:rerun-if-changed=cpp/carla_sys_bridge.cpp");
    println!("cargo:rerun-if-changed=cpp/sensor_callback_bridge.h");
    println!("cargo:rerun-if-changed=cpp/sensor_callback_bridge.cpp");

    // Use carla-src for CARLA_ROOT approach
    use carla_src::CarlaSource;
    println!("cargo:rerun-if-env-changed=CARLA_ROOT");
    let carla_source = CarlaSource::new().expect("Failed to obtain CARLA source");
    println!(
        "Using CARLA source from: {}",
        carla_source.source_dir().display()
    );
    let source_dir = carla_source.source_dir().to_path_buf();

    // Build LibCarla if needed
    let (lib_dir, include_dir) = build_libcarla(&source_dir)?;

    // Build CXX bridge
    let mut bridge = cxx_build::bridge("src/ffi.rs");

    // Add include directories
    bridge.include(&include_dir);
    // For carla-src, use the existing include method
    for include in carla_source.include_dirs() {
        bridge.include(include);
    }
    bridge.include("include");

    // Configure compiler
    bridge
        .file("cpp/carla_sys_bridge.cpp")
        // TODO: Fix sensor callback bridge C++ API compatibility issues
        // .file("cpp/sensor_callback_bridge.cpp")
        .flag("-DCARLA_SYS_BRIDGE")
        .flag("-w"); // Suppress warnings for now

    // Use C++20 for std::identity in carla-src
    bridge.std("c++20").flag_if_supported("-std=c++20");

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

    // Generate bindings if bindgen feature is enabled
    if cfg!(feature = "bindgen") {
        generate_bindings(&source_dir, &include_dir)?;
    }

    // Generate Rust types from Python API YAML
    generate_rust_types(&source_dir)?;

    Ok(())
}

/// Build LibCarla from source
fn build_libcarla(source_dir: &Path) -> anyhow::Result<(PathBuf, PathBuf)> {
    let out_dir = env::var("OUT_DIR")?;
    let build_dir = Path::new(&out_dir).join("carla-build");

    // Check if we have pre-built libraries in the source
    let prebuilt_lib = source_dir.join("install/lib");
    let prebuilt_include = source_dir.join("install/include");

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

    let mut cmake_config = cmake::Config::new(source_dir);

    // Full configuration for carla-src builds
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
        .define("BOOST_ENABLE_PYTHON", "ON")
        .define(
            "BOOST_INCLUDE_LIBRARIES",
            "asio;iterator;python;date_time;geometry;container;variant2;gil",
        )
        .define("BOOST_EXCLUDE_LIBRARIES", "mpi")
        .out_dir(&build_dir)
        .build_target("carla-client");

    // Set environment variables for the cmake configuration as well
    cmake_config.env("LC_ALL", "C.UTF-8");
    cmake_config.env("LANG", "C.UTF-8");

    // Set C++ standard
    cmake_config.cxxflag("-std=c++14");

    // Add include paths for dependencies
    let build_dir_path = source_dir.join("Build");
    if build_dir_path.exists() {
        // If dependencies are pre-built in source
        cmake_config.define(
            "BOOST_ROOT",
            build_dir_path.join("boost-1.80.0-c10-install"),
        );
        cmake_config.define("RPCLIB_ROOT", build_dir_path.join("rpclib-c10-install"));
        cmake_config.define(
            "RECAST_ROOT",
            build_dir_path.join("recast-22dfcb-c10-install"),
        );
        cmake_config.define("ZLIB_ROOT", build_dir_path.join("zlib-install"));
        cmake_config.define("LIBPNG_ROOT", build_dir_path.join("libpng-1.6.37-install"));
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

/// Generate bindings using bindgen (when feature is enabled)
#[cfg(feature = "bindgen")]
fn generate_bindings(source_dir: &Path, include_dir: &Path) -> anyhow::Result<()> {
    use std::io::Write;

    println!("Generating bindings with bindgen...");

    // Create a wrapper header that includes our bridge
    let wrapper_header = include_dir.join("wrapper.h");
    let mut wrapper_file = std::fs::File::create(&wrapper_header)?;
    writeln!(wrapper_file, "#include \"carla_sys_bridge.h\"")?;
    drop(wrapper_file);

    let bindings = bindgen::Builder::default()
        .header(wrapper_header.to_string_lossy())
        .clang_arg(format!("-I{}", include_dir.display()))
        .clang_arg(format!(
            "-I{}",
            source_dir.join("LibCarla/source").display()
        ))
        .clang_arg("-DCARLA_SYS_BRIDGE")
        .clang_arg("-std=c++20")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .map_err(|e| anyhow::anyhow!("Failed to generate bindings: {}", e))?;

    // Write bindings to src/bindings/generated.rs
    let bindings_dir = Path::new("src/bindings");
    std::fs::create_dir_all(bindings_dir)?;
    let bindings_path = bindings_dir.join("generated.rs");
    bindings
        .write_to_file(&bindings_path)
        .map_err(|e| anyhow::anyhow!("Failed to write bindings: {}", e))?;

    println!("Generated bindings written to: {}", bindings_path.display());
    Ok(())
}

#[cfg(not(feature = "bindgen"))]
fn generate_bindings(_source_dir: &Path, _include_dir: &Path) -> anyhow::Result<()> {
    // No-op when bindgen feature is not enabled
    Ok(())
}

/// Generate Rust types from Python API YAML documentation
fn generate_rust_types(source_dir: &Path) -> anyhow::Result<()> {
    // Skip generation if using pre-generated code (default behavior)
    if !cfg!(feature = "save-bindgen") {
        println!("cargo:warning=Using pre-generated code from src/generated/");
        return Ok(());
    }

    println!("cargo:warning=Generating Rust types from Python API YAML...");

    // Use carla-codegen to generate types
    use carla_codegen::{config::Config, Generator};

    let yaml_dir = source_dir.join("PythonAPI/docs");
    if !yaml_dir.exists() {
        println!(
            "cargo:warning=CARLA Python API docs not found at {yaml_dir:?}, skipping code generation"
        );
        return Ok(());
    }

    // Load configuration
    let config_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("carla-codegen.toml");
    let mut config = if config_path.exists() {
        Config::from_file(&config_path)?
    } else {
        Config::default()
    };

    // Configure output directory for pre-generated code
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/generated");

    // Generate both FFI and stub versions
    if cfg!(feature = "docs-only") {
        // Generate stub implementations
        config.output_dir = src_dir.join("stubs");
        config.stub_mode = true;
    } else {
        // Generate FFI implementations
        config.output_dir = src_dir.join("ffi");
        config.stub_mode = false;
    }

    // Create and run generator
    let mut generator = Generator::new(config);

    match generator.add_yaml_dir(&yaml_dir) {
        Ok(_) => {
            match generator.generate() {
                Ok(_) => {
                    println!(
                        "cargo:warning=Generated code saved to {}",
                        generator.config.output_dir.display()
                    );
                }
                Err(e) => {
                    println!("cargo:warning=Code generation failed: {e}");
                    // Don't fail the build, just skip generation
                }
            }
        }
        Err(e) => {
            println!("cargo:warning=Failed to parse YAML files: {e}");
            // Don't fail the build, just skip generation
        }
    }

    Ok(())
}
