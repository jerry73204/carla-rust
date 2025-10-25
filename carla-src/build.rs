fn main() {
    // Pass CARLA_VERSION environment variable to the compiler
    if let Ok(version) = std::env::var("CARLA_VERSION") {
        println!("cargo:rustc-env=CARLA_VERSION={}", version);
    }

    // Rerun if CARLA_VERSION changes
    println!("cargo:rerun-if-env-changed=CARLA_VERSION");
}
