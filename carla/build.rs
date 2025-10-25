use std::env;

fn main() {
    // Declare the custom cfg
    println!("cargo:rustc-check-cfg=cfg(carla_0916)");

    // Get CARLA version from carla-sys dependency or environment
    // Note: DEP_CARLA_* comes from carla-sys's `links = "carla"`
    let version = env::var("DEP_CARLA_CARLA_VERSION")
        .or_else(|_| env::var("CARLA_VERSION"))
        .unwrap_or_else(|_| {
            eprintln!("Warning: CARLA version not detected, defaulting to 0.9.14");
            String::from("0.9.14")
        });

    eprintln!("Building carla crate with CARLA version {}", version);

    // Set up cfg flags for version-specific code
    if version == "0.9.16" {
        println!("cargo:rustc-cfg=carla_0916");
        eprintln!("Setting carla_0916 cfg flag");
    }

    // Rerun if CARLA_VERSION changes
    println!("cargo:rerun-if-env-changed=CARLA_VERSION");
    println!("cargo:rerun-if-env-changed=DEP_CARLA_CARLA_VERSION");
}
