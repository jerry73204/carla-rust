//! Example of using carla-src to obtain CARLA source code

use carla_src::{CarlaSource, CarlaSourceError};

fn main() -> Result<(), CarlaSourceError> {
    // Initialize logging
    env_logger::init();

    // Create CarlaSource - requires CARLA_ROOT to be set
    println!("Obtaining CARLA source from CARLA_ROOT...");
    let source = CarlaSource::new()?;

    // Print source information
    println!("\nCARLA source information:");
    println!("  Source directory: {}", source.source_dir().display());

    // Print include directories
    println!("\nInclude directories:");
    for include_dir in source.include_dirs() {
        println!("  {}", include_dir.display());
    }

    // Print library directories
    println!("\nLibrary directories:");
    for lib_dir in source.lib_dirs() {
        println!("  {}", lib_dir.display());
    }

    // Print specific paths
    println!("\nSpecific paths:");
    println!(
        "  LibCarla source: {}",
        source.libcarla_source_dir().display()
    );
    println!("  Source root: {}", source.source_dir().display());
    println!("  Build directory: {}", source.build_dir().display());

    Ok(())
}
