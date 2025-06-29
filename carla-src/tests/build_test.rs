//! Build test for carla-src
//!
//! This test verifies that the build system correctly identifies
//! and uses CARLA source for compilation.

use carla_src::CarlaSource;
use std::env;

#[test]
fn test_carla_root_build_configuration() {
    // This test verifies that when CARLA_ROOT is set to the carla-simulator
    // directory, all paths are correctly resolved

    let carla_simulator = env::current_dir()
        .unwrap()
        .parent()
        .unwrap()
        .join("carla-simulator");

    if carla_simulator.exists() {
        env::set_var("CARLA_ROOT", &carla_simulator);

        let source = CarlaSource::new().unwrap();

        // Verify source directory
        assert_eq!(source.source_dir(), carla_simulator);

        // Verify CMakeLists.txt exists
        assert!(source.source_dir().join("CMakeLists.txt").exists());

        // Verify LibCarla source directory
        assert!(source.libcarla_source_dir().exists());
        assert!(source.libcarla_source_dir().join("carla").exists());

        // If install directory exists, verify it has the expected structure
        let install_dir = source.source_dir().join("install");
        if install_dir.exists() {
            assert!(install_dir.join("lib").exists());
            assert!(install_dir.join("include").exists());
            println!("Pre-built libraries found in install directory");
        }

        env::remove_var("CARLA_ROOT");
    } else {
        println!("Skipping test - carla-simulator directory not found");
    }
}

#[test]
fn test_build_directory_structure() {
    // Test that the expected build dependencies paths exist
    let temp_dir = tempfile::tempdir().unwrap();
    let mock_source = temp_dir.path();

    // Create a more complete mock structure including build dependencies
    let dirs = vec![
        "LibCarla/source/carla",
        "LibCarla/source/carla/client",
        "LibCarla/source/carla/rpc",
        "LibCarla/source/carla/sensor",
        "LibCarla/source/carla/streaming",
        "LibCarla/source/carla/road",
        "LibCarla/source/carla/geom",
        "LibCarla/source/carla/trafficmanager",
        "LibCarla/source/third-party",
        "LibCarla/source/third-party/rpclib/include",
        "build/boost-1.80.0-c10-install/include",
        "build/boost-1.80.0-c10-install/lib",
        "build/rpclib-c10-install/include",
        "build/rpclib-c10-install/lib",
        "build/recast-22dfcb-c10-install/include",
        "build/recast-22dfcb-c10-install/lib",
        "build/zlib-install/include",
        "build/zlib-install/lib",
        "build/libpng-1.6.37-install/include",
        "build/libpng-1.6.37-install/lib",
    ];

    for dir in dirs {
        std::fs::create_dir_all(mock_source.join(dir)).unwrap();
    }

    // Create CMakeLists.txt in root
    std::fs::write(mock_source.join("CMakeLists.txt"), "# Mock CMake").unwrap();
    std::fs::write(mock_source.join("CHANGELOG.md"), "# CARLA 0.10.0").unwrap();

    let source = CarlaSource::from_local(mock_source).unwrap();

    // Verify all include directories exist
    for include_dir in source.include_dirs() {
        assert!(include_dir.exists(), "Include dir missing: {include_dir:?}");
    }

    // Verify all library directories exist
    for lib_dir in source.lib_dirs() {
        assert!(lib_dir.exists(), "Library dir missing: {lib_dir:?}");
    }
}
