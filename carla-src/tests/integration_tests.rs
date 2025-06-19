//! Integration tests for carla-src
//!
//! These tests verify CARLA_ROOT mode.

use carla_src::{CarlaSource, CarlaSourceError};
use std::{env, fs, path::Path};

/// Create a mock CARLA source directory structure for testing
fn create_mock_carla_source(root: &Path) -> std::io::Result<()> {
    // Create required directories
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
        "build/boost-1.80.0-c10-install/include",
        "build/boost-1.80.0-c10-install/lib",
        "install/lib",
        "install/include",
    ];

    for dir in dirs {
        fs::create_dir_all(root.join(dir))?;
    }

    // Create required files
    fs::write(root.join("CMakeLists.txt"), "# Mock CMakeLists.txt")?;
    fs::write(root.join("CHANGELOG.md"), "# CARLA 0.10.0 Release")?;

    Ok(())
}

#[test]
fn test_carla_root_mode() {
    // Create a temporary directory for mock CARLA source
    let temp_dir = tempfile::tempdir().unwrap();
    let mock_source = temp_dir.path();

    // Set up mock CARLA source
    create_mock_carla_source(mock_source).unwrap();

    // Set CARLA_ROOT environment variable
    env::set_var("CARLA_ROOT", mock_source);

    // Create CarlaSource - should use CARLA_ROOT
    let source = CarlaSource::new().unwrap();

    // Verify it's using the local source
    assert_eq!(source.source_dir(), mock_source);

    // Check include directories
    let include_dirs = source.include_dirs();
    assert!(!include_dirs.is_empty());
    assert!(include_dirs.iter().any(|p| p.ends_with("LibCarla/source")));

    // Check library directories
    let lib_dirs = source.lib_dirs();
    assert!(!lib_dirs.is_empty());

    // Clean up
    env::remove_var("CARLA_ROOT");
}

#[test]
fn test_from_local_with_valid_source() {
    // Create a temporary directory for mock CARLA source
    let temp_dir = tempfile::tempdir().unwrap();
    let mock_source = temp_dir.path();

    // Set up mock CARLA source
    create_mock_carla_source(mock_source).unwrap();

    // Create CarlaSource from local path
    let source = CarlaSource::from_local(mock_source).unwrap();

    // Verify paths
    assert_eq!(source.source_dir(), mock_source);
    assert_eq!(
        source.libcarla_source_dir(),
        mock_source.join("LibCarla/source")
    );
    // CMakeLists.txt is now in the root
    assert_eq!(source.source_dir(), mock_source);
}

#[test]
fn test_from_local_with_invalid_source() {
    // Test with non-existent directory
    let result = CarlaSource::from_local("/tmp/non_existent_carla_source");
    assert!(result.is_err());
    match result {
        Err(CarlaSourceError::InvalidSource(msg)) => {
            assert!(msg.contains("Directory does not exist"));
        }
        _ => panic!("Expected InvalidSource error"),
    }

    // Test with directory missing required structure
    let temp_dir = tempfile::tempdir().unwrap();
    let incomplete_source = temp_dir.path();
    fs::create_dir_all(incomplete_source.join("LibCarla/source")).unwrap();

    let result = CarlaSource::from_local(incomplete_source);
    assert!(result.is_err());
    match result {
        Err(CarlaSourceError::InvalidSource(msg)) => {
            assert!(msg.contains("Missing required path"));
        }
        _ => panic!("Expected InvalidSource error"),
    }
}

#[test]
fn test_path_resolution() {
    // Create a temporary directory for mock CARLA source
    let temp_dir = tempfile::tempdir().unwrap();
    let mock_source = temp_dir.path();

    // Set up mock CARLA source
    create_mock_carla_source(mock_source).unwrap();

    // Create CarlaSource
    let source = CarlaSource::from_local(mock_source).unwrap();

    // Test include directories
    let include_dirs = source.include_dirs();
    assert_eq!(include_dirs.len(), 8);
    assert!(include_dirs[0].ends_with("LibCarla/source"));
    assert!(include_dirs[1].ends_with("LibCarla/source/third-party"));

    // Test library directories
    let lib_dirs = source.lib_dirs();
    assert_eq!(lib_dirs.len(), 5);
    assert!(lib_dirs[0].ends_with("boost-1.80.0-c10-install/lib"));

    // Test specific paths
    assert!(source.build_dir().ends_with("build"));
    assert!(source.libcarla_source_dir().ends_with("LibCarla/source"));
}

#[test]
fn test_carla_root_not_set() {
    // Make sure CARLA_ROOT is not set
    env::remove_var("CARLA_ROOT");

    // Should return an error
    let result = CarlaSource::new();
    assert!(result.is_err());
    match result {
        Err(CarlaSourceError::Environment(msg)) => {
            assert!(msg.contains("CARLA_ROOT environment variable must be set"));
        }
        _ => panic!("Expected Environment error"),
    }
}

/// Test version checking functionality
#[test]
fn test_version_checking() {
    // Create a temporary directory for mock CARLA source
    let temp_dir = tempfile::tempdir().unwrap();
    let mock_source = temp_dir.path();

    // Set up mock CARLA source
    create_mock_carla_source(mock_source).unwrap();

    // Create different version files
    fs::write(
        mock_source.join("CHANGELOG.md"),
        "# CARLA 0.10.0\nThis is the 0.10.0 release",
    )
    .unwrap();

    // Should succeed with correct version
    let result = CarlaSource::from_local(mock_source);
    assert!(result.is_ok());

    // Test with wrong version (this currently only warns)
    fs::write(
        mock_source.join("CHANGELOG.md"),
        "# CARLA 0.9.14\nThis is the 0.9.14 release",
    )
    .unwrap();

    // Should still succeed but with warning
    let result = CarlaSource::from_local(mock_source);
    assert!(result.is_ok());
}
