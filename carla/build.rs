use anyhow::{bail, Context, Result};
use std::{env, str::FromStr};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum CarlaVersion {
    #[default]
    V0_9_16,
    V0_9_15,
    V0_9_14,
}

impl FromStr for CarlaVersion {
    type Err = anyhow::Error;

    fn from_str(ver_text: &str) -> Result<Self, Self::Err> {
        let ver = match ver_text {
            "0.9.16" => Self::V0_9_16,
            "0.9.15" => Self::V0_9_15,
            "0.9.14" => Self::V0_9_14,
            _ => bail!(
                "unsupported CARLA version: '{}'. Supported versions: 0.9.14, 0.9.15, 0.9.16",
                ver_text
            ),
        };
        Ok(ver)
    }
}

impl CarlaVersion {
    fn as_str(&self) -> &str {
        match self {
            CarlaVersion::V0_9_16 => "0.9.16",
            CarlaVersion::V0_9_15 => "0.9.15",
            CarlaVersion::V0_9_14 => "0.9.14",
        }
    }
}

fn parse_carla_version() -> Result<CarlaVersion> {
    // Get CARLA version from carla-sys dependency or environment
    // Note: DEP_CARLA_* comes from carla-sys's `links = "carla"`
    let version_str = env::var("DEP_CARLA_CARLA_VERSION")
        .or_else(|_| env::var("CARLA_VERSION"))
        .unwrap_or_else(|_| {
            eprintln!("Warning: CARLA version not detected, defaulting to 0.9.16");
            CarlaVersion::default().as_str().to_string()
        });

    let version: CarlaVersion = version_str
        .parse()
        .with_context(|| format!("failed to parse CARLA version from '{}'", version_str))?;

    Ok(version)
}

fn main() -> Result<()> {
    let version = parse_carla_version()?;

    eprintln!(
        "Building carla crate with CARLA version {}",
        version.as_str()
    );

    // Declare the custom cfg
    println!("cargo:rustc-check-cfg=cfg(carla_0916)");

    // Set up cfg flags for version-specific code
    match version {
        CarlaVersion::V0_9_16 => {
            println!("cargo:rustc-cfg=carla_0916");
            eprintln!("Setting carla_0916 cfg flag");
        }
        CarlaVersion::V0_9_15 => {
            // No special cfg flag for 0.9.15
        }
        CarlaVersion::V0_9_14 => {
            // No special cfg flag for 0.9.14
        }
    }

    // Rerun if CARLA_VERSION changes
    println!("cargo:rerun-if-env-changed=CARLA_VERSION");
    println!("cargo:rerun-if-env-changed=DEP_CARLA_CARLA_VERSION");

    Ok(())
}
