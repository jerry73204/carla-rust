//! CARLA source code management utilities.
//!
//! This crate provides functionality to validate and use CARLA simulator
//! source code for building the Rust bindings.
//!
//! The CARLA_ROOT environment variable must be set to point to a valid
//! CARLA source directory.

use std::{
    env,
    path::{Path, PathBuf},
};
use thiserror::Error;

/// Expected CARLA version
pub const CARLA_VERSION: &str = "0.10.0";

/// Errors that can occur during source management
#[derive(Debug, Error)]
pub enum CarlaSourceError {
    /// Invalid CARLA source directory structure
    #[error("Invalid CARLA source: {0}")]
    InvalidSource(String),

    /// IO error
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Environment variable error
    #[error("Environment error: {0}")]
    Environment(String),

    /// Version mismatch
    #[error("Version mismatch: expected {expected}, found {found}")]
    VersionMismatch { expected: String, found: String },
}

/// Represents a validated CARLA source directory
pub struct CarlaSource {
    /// Root directory of CARLA source
    source_dir: PathBuf,
}

impl CarlaSource {
    /// Create a new CarlaSource instance.
    ///
    /// This requires the `CARLA_ROOT` environment variable to be set to a valid
    /// CARLA source directory.
    pub fn new() -> Result<Self, CarlaSourceError> {
        let root = env::var("CARLA_ROOT").map_err(|_| {
            CarlaSourceError::Environment(
                "CARLA_ROOT environment variable must be set to the CARLA source directory"
                    .to_string(),
            )
        })?;

        log::info!("Using CARLA source from CARLA_ROOT: {}", root);
        Self::from_local(root)
    }

    /// Create from a local directory path
    pub fn from_local<P: AsRef<Path>>(path: P) -> Result<Self, CarlaSourceError> {
        let source_dir = path.as_ref().to_path_buf();

        // Validate the source directory
        Self::validate_source(&source_dir)?;

        // Check version if possible
        Self::check_version(&source_dir)?;

        Ok(CarlaSource { source_dir })
    }

    /// Validate that the source directory contains required CARLA structure
    fn validate_source(path: &Path) -> Result<(), CarlaSourceError> {
        log::debug!("Validating CARLA source at: {}", path.display());

        // Check if directory exists
        if !path.exists() {
            return Err(CarlaSourceError::InvalidSource(format!(
                "Directory does not exist: {}",
                path.display()
            )));
        }

        // Required directories and files for LibCarla
        let required_paths = vec![
            // Core LibCarla directories
            "LibCarla/source/carla",
            "LibCarla/source/carla/client",
            "LibCarla/source/carla/rpc",
            "LibCarla/source/carla/sensor",
            "LibCarla/source/carla/streaming",
            "LibCarla/source/carla/road",
            "LibCarla/source/carla/geom",
            "LibCarla/source/carla/trafficmanager",
            // Third-party dependencies
            "LibCarla/source/third-party",
            // Build files - CMakeLists.txt is in the root
            "CMakeLists.txt",
        ];

        for required in &required_paths {
            let full_path = path.join(required);
            if !full_path.exists() {
                return Err(CarlaSourceError::InvalidSource(format!(
                    "Missing required path: {} (looked at: {})",
                    required,
                    full_path.display()
                )));
            }
        }

        log::debug!("Source validation successful");
        Ok(())
    }

    /// Check CARLA version if possible
    fn check_version(path: &Path) -> Result<(), CarlaSourceError> {
        // Try to read version from various possible locations
        let version_files = vec![
            path.join("Util/ContentVersions.txt"),
            path.join("Version.txt"),
            path.join("CHANGELOG.md"),
        ];

        for version_file in version_files {
            if version_file.exists() {
                match std::fs::read_to_string(&version_file) {
                    Ok(content) => {
                        // Simple version check - look for version string
                        if content.contains(CARLA_VERSION) {
                            log::debug!(
                                "Found matching version {} in {}",
                                CARLA_VERSION,
                                version_file.display()
                            );
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        log::warn!(
                            "Failed to read version file {}: {}",
                            version_file.display(),
                            e
                        );
                    }
                }
            }
        }

        // If we can't verify version, log a warning but continue
        log::warn!("Could not verify CARLA version, assuming {}", CARLA_VERSION);
        Ok(())
    }

    /// Get the root source directory
    pub fn source_dir(&self) -> &Path {
        &self.source_dir
    }

    /// Get the LibCarla source directory
    pub fn libcarla_source_dir(&self) -> PathBuf {
        self.source_dir.join("LibCarla/source")
    }

    /// Get the LibCarla CMake directory (deprecated - CMakeLists.txt is in root)
    #[deprecated(note = "CMakeLists.txt is in the root directory, use source_dir() instead")]
    pub fn libcarla_cmake_dir(&self) -> PathBuf {
        self.source_dir.to_path_buf()
    }

    /// Get include directories for compilation
    pub fn include_dirs(&self) -> Vec<PathBuf> {
        vec![
            self.libcarla_source_dir(),
            self.source_dir.join("LibCarla/source/third-party"),
            self.source_dir
                .join("LibCarla/source/third-party/rpclib/include"),
            self.source_dir
                .join("build/boost-1.80.0-c10-install/include"),
            self.source_dir.join("build/rpclib-c10-install/include"),
            self.source_dir
                .join("build/recast-22dfcb-c10-install/include"),
            self.source_dir.join("build/zlib-install/include"),
            self.source_dir.join("build/libpng-1.6.37-install/include"),
        ]
    }

    /// Get the path to pre-built dependencies if they exist
    pub fn build_dir(&self) -> PathBuf {
        self.source_dir.join("build")
    }

    /// Get library search paths
    pub fn lib_dirs(&self) -> Vec<PathBuf> {
        vec![
            self.build_dir().join("boost-1.80.0-c10-install/lib"),
            self.build_dir().join("rpclib-c10-install/lib"),
            self.build_dir().join("recast-22dfcb-c10-install/lib"),
            self.build_dir().join("zlib-install/lib"),
            self.build_dir().join("libpng-1.6.37-install/lib"),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_source_validation() {
        // This test would require a valid CARLA source directory
        // For now, we just test the error case
        let result = CarlaSource::from_local("/tmp/nonexistent");
        assert!(result.is_err());
    }
}
