//! CARLA blueprint library example
//!
//! This example demonstrates how to:
//! - Access the blueprint library
//! - Filter blueprints by type
//! - Examine blueprint attributes
//! - Find specific blueprints

use anyhow::Result;
use carla::client::{ActorBlueprint, Client};
use clap::Parser;

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry};

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "CARLA example: Blueprint library exploration"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    /// Filter blueprints by ID pattern
    #[arg(short, long)]
    filter: Option<String>,

    /// Show detailed attributes for each blueprint
    #[arg(short, long)]
    detailed: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    args.common.init_logging();

    println!("=== CARLA Blueprint Library Example ===\n");

    // Connect to CARLA
    args.common.print_connection_info();
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    println!("✓ Connected successfully!\n");

    // Run the example
    run_example(&client, &args)?;

    println!("\n✓ Example completed successfully!");
    Ok(())
}

fn run_example(client: &Client, args: &Args) -> Result<()> {
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    println!("Blueprint Library:");
    println!("==================");
    println!("Total blueprints: {}", blueprint_library.len());

    // Apply filter if provided
    let blueprints: Vec<_> = if let Some(filter) = &args.filter {
        blueprint_library
            .iter()
            .filter(|bp| bp.id().contains(filter))
            .collect()
    } else {
        blueprint_library.iter().collect()
    };

    println!("Matching blueprints: {}\n", blueprints.len());

    // Group blueprints by category
    let mut categories: std::collections::HashMap<String, Vec<&ActorBlueprint>> =
        std::collections::HashMap::new();

    for blueprint in &blueprints {
        let id = blueprint.id();
        let category = id.split('.').next().unwrap_or("unknown");
        categories
            .entry(category.to_string())
            .or_default()
            .push(blueprint);
    }

    // Display blueprints by category
    let mut sorted_categories: Vec<_> = categories.into_iter().collect();
    sorted_categories.sort_by_key(|(cat, _)| cat.clone());

    for (category, bps) in sorted_categories {
        println!("{}/ ({} blueprints)", category, bps.len());
        println!("{}", "-".repeat(50));

        for blueprint in bps {
            println!("  {}", blueprint.id());

            if args.detailed {
                // Show tags
                // TODO: Blueprint tags are not yet exposed in the API
                let tags: Vec<&str> = vec![];
                if !tags.is_empty() {
                    println!("    Tags: {}", tags.join(", "));
                }

                // Show attributes
                // TODO: Blueprint attributes iteration is not yet exposed in the API
                // This would require implementing Iterator trait for ActorBlueprint
                println!();
            }
        }
        println!();
    }

    // Show some interesting statistics
    println!("Blueprint Statistics:");
    println!("====================");

    // Count vehicles
    let vehicles = blueprint_library.filter("vehicle.*")?;
    println!("Vehicles: {}", vehicles.len());

    // Count sensors
    let sensors = blueprint_library.filter("sensor.*")?;
    println!("Sensors: {}", sensors.len());

    // Count walkers
    let walkers = blueprint_library.filter("walker.*")?;
    println!("Walkers: {}", walkers.len());

    // Count props
    let props = blueprint_library.filter("static.prop.*")?;
    println!("Props: {}", props.len());

    // Find a specific blueprint as an example
    println!("\nExample: Finding Tesla Model 3");
    println!("==============================");
    match blueprint_library.find("vehicle.tesla.model3")? {
        Some(tesla) => {
            println!("Found: {}", tesla.id());
            println!("Attributes:");
            // TODO: Blueprint attributes iteration is not yet exposed in the API
            println!("  (attribute iteration not yet implemented)");
        }
        None => {
            println!("Tesla Model 3 blueprint not found in this CARLA version");
        }
    }

    Ok(())
}
