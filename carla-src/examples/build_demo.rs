//! Example demonstrating how carla-sys would use carla-src to build LibCarla

use carla_src::CarlaSource;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    println!("=== CARLA Build System Demo ===\n");

    // CARLA_ROOT is required
    match CarlaSource::new() {
        Ok(source) => {
            println!("✓ Successfully loaded CARLA source from CARLA_ROOT");
            println!("  Source directory: {}", source.source_dir().display());
            print_build_info(&source);
        }
        Err(e) => {
            println!("✗ Failed to load CARLA source: {}", e);
            println!("\nTo use carla-src:");
            println!("  export CARLA_ROOT=/path/to/carla/source");
            println!("  cargo run --example build_demo");
            return Ok(());
        }
    }

    // Test 2: Direct path mode
    println!("\n\nTesting direct path mode...");
    let test_path = Path::new("/tmp/test_carla_source");
    match CarlaSource::from_local(test_path) {
        Ok(source) => {
            println!("✓ Successfully loaded from path: {}", test_path.display());
            print_build_info(&source);
        }
        Err(e) => {
            println!("✗ Expected error for non-existent path: {}", e);
        }
    }

    Ok(())
}

fn print_build_info(source: &CarlaSource) {
    println!("\nBuild configuration:");
    println!(
        "  LibCarla source: {}",
        source.libcarla_source_dir().display()
    );
    println!("  Source root: {}", source.source_dir().display());
    println!("  Build directory: {}", source.build_dir().display());

    println!("\nInclude directories:");
    for (i, include) in source.include_dirs().iter().enumerate() {
        println!("  [{}] {}", i + 1, include.display());
    }

    println!("\nLibrary directories:");
    for (i, lib) in source.lib_dirs().iter().enumerate() {
        println!("  [{}] {}", i + 1, lib.display());
    }
}
