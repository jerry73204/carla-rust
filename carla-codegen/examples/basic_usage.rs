//! Basic usage example for carla-codegen
//!
//! This example demonstrates how to use the carla-codegen library
//! to generate Rust code from CARLA Python API YAML files.

use carla_codegen::{config::Config, Generator};

fn main() -> carla_codegen::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    // Example 1: Basic usage with default configuration
    basic_generation()?;

    // Example 2: Advanced usage with custom configuration
    advanced_generation()?;

    Ok(())
}

/// Basic code generation with default settings
fn basic_generation() -> carla_codegen::Result<()> {
    println!("=== Basic Generation Example ===");

    // Create a generator with default configuration
    let config = Config::default();
    let _generator = Generator::new(config);

    // This would normally point to actual CARLA YAML files
    // For this example, we'll just show the API usage
    println!("Would generate from YAML files in: carla-simulator/PythonAPI/docs/");
    println!("Output would be written to: generated/");

    Ok(())
}

/// Advanced code generation with custom configuration
fn advanced_generation() -> carla_codegen::Result<()> {
    println!("\n=== Advanced Generation Example ===");

    // Build a custom configuration
    let config = Config::builder()
        .output_dir("custom_output")
        .builder_threshold(3)
        .generate_async(true)
        .add_type_mapping("actor_id".to_string(), "u32".to_string())
        .exclude_class("DeprecatedClass".to_string())
        .include_class("Actor".to_string())
        .include_class("Vehicle".to_string())
        .skip_module("osm2odr".to_string())
        .build();

    let _generator = Generator::new(config);

    // Configure type mappings and filters
    println!("Custom configuration:");
    println!("- Builder threshold: 3");
    println!("- Async generation: enabled");
    println!("- Custom type mapping: actor_id -> u32");
    println!("- Excluded classes: DeprecatedClass");
    println!("- Included classes: Actor, Vehicle");
    println!("- Skipped modules: osm2odr");

    Ok(())
}

/// Example of using the configuration system
#[allow(dead_code)]
fn configuration_examples() -> carla_codegen::Result<()> {
    // Load configuration from TOML file
    let _config = Config::from_file("carla-codegen.toml")?;

    // Create configuration with builder pattern
    let _config = Config::builder()
        .yaml_dir("../carla-simulator/PythonAPI/docs")
        .output_dir("src/generated")
        .builder_threshold(2)
        .add_type_mapping("int".to_string(), "i32".to_string())
        .exclude_class("ObsoleteClass".to_string())
        .build();

    // Modify existing configuration
    let mut config = Config::default();
    config.add_type_mapping("float".to_string(), "f64".to_string());
    config.exclude_class("TestClass".to_string());
    config.set_builder_threshold(4);

    Ok(())
}
