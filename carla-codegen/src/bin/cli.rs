//! CLI tool for CARLA code generation

use anyhow::Result;
use carla_codegen::{config::Config, Generator};
use clap::{Parser, Subcommand};
use std::path::PathBuf;
use tracing::info;
use tracing_subscriber::EnvFilter;

/// CARLA Python API to Rust code generator
#[derive(Parser)]
#[command(name = "carla-codegen")]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Generate Rust code from YAML files
    Generate {
        /// YAML input directory or file
        #[arg(short, long)]
        input: PathBuf,

        /// Output directory for generated code
        #[arg(short, long, default_value = "generated")]
        output: PathBuf,

        /// Configuration file
        #[arg(short, long)]
        config: Option<PathBuf>,

        /// Add custom type mapping (can be used multiple times)
        #[arg(long, value_parser = parse_type_mapping)]
        type_mapping: Vec<(String, String)>,

        /// Exclude class from generation (can be used multiple times)
        #[arg(long)]
        exclude_class: Vec<String>,

        /// Include only specific classes (can be used multiple times)
        #[arg(long)]
        include_class: Vec<String>,

        /// Skip module (can be used multiple times)
        #[arg(long)]
        skip_module: Vec<String>,

        /// Builder pattern threshold
        #[arg(long)]
        builder_threshold: Option<usize>,
    },

    /// Validate YAML files without generating code
    Validate {
        /// YAML input directory or file
        #[arg(short, long)]
        input: PathBuf,

        /// Configuration file
        #[arg(short, long)]
        config: Option<PathBuf>,
    },

    /// List all classes and modules found in YAML files
    List {
        /// YAML input directory or file
        #[arg(short, long)]
        input: PathBuf,

        /// List modules only
        #[arg(long)]
        modules: bool,

        /// List classes only
        #[arg(long)]
        classes: bool,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Initialize logging
    let filter = if cli.verbose {
        EnvFilter::new("debug")
    } else {
        EnvFilter::new("info")
    };

    tracing_subscriber::fmt().with_env_filter(filter).init();

    match cli.command {
        Commands::Generate {
            input,
            output,
            config,
            type_mapping,
            exclude_class,
            include_class,
            skip_module,
            builder_threshold,
        } => generate(
            input,
            output,
            config,
            type_mapping,
            exclude_class,
            include_class,
            skip_module,
            builder_threshold,
        ),
        Commands::Validate { input, config } => validate(input, config),
        Commands::List {
            input,
            modules,
            classes,
        } => list(input, modules, classes),
    }
}

#[allow(clippy::too_many_arguments)]
fn generate(
    input: PathBuf,
    output: PathBuf,
    config_file: Option<PathBuf>,
    type_mappings: Vec<(String, String)>,
    exclude_classes: Vec<String>,
    include_classes: Vec<String>,
    skip_modules: Vec<String>,
    builder_threshold: Option<usize>,
) -> Result<()> {
    info!("Starting code generation");

    // Load or create configuration
    let mut config = if let Some(config_path) = config_file {
        Config::from_file(config_path)?
    } else {
        Config::default()
    };

    // Apply CLI overrides
    config.output_dir = output;

    for (python_type, rust_type) in type_mappings {
        config.add_type_mapping(python_type, rust_type);
    }

    for class in exclude_classes {
        config.exclude_class(class);
    }

    for class in include_classes {
        config.include_class(class);
    }

    for module in skip_modules {
        if !config.filters.skip_modules.contains(&module) {
            config.filters.skip_modules.push(module);
        }
    }

    if let Some(threshold) = builder_threshold {
        config.set_builder_threshold(threshold);
    }

    // Create generator
    let mut generator = Generator::new(config);

    // Add input files
    if input.is_file() {
        generator.add_yaml_file(&input)?;
    } else {
        generator.add_yaml_dir(&input)?;
    }

    // Generate code
    generator.generate()?;
    info!("Code generation completed successfully");
    info!(
        "Output written to: {}",
        generator.config.output_dir.display()
    );

    Ok(())
}

fn validate(input: PathBuf, config_file: Option<PathBuf>) -> Result<()> {
    info!("Validating YAML files");

    let config = if let Some(config_path) = config_file {
        Config::from_file(config_path)?
    } else {
        Config::default()
    };

    let mut generator = Generator::new(config);

    // Parse input files
    if input.is_file() {
        generator.add_yaml_file(&input)?;
    } else {
        generator.add_yaml_dir(&input)?;
    }

    info!("Validation successful - all YAML files are valid");
    Ok(())
}

fn list(input: PathBuf, modules_only: bool, classes_only: bool) -> Result<()> {
    use carla_codegen::parser::YamlParser;

    let mut parser = YamlParser::new();

    // Parse input files
    if input.is_file() {
        parser.parse_file(&input)?;
    } else {
        parser.parse_directory(&input)?;
    }

    let modules = parser.modules();

    if modules_only || !classes_only {
        println!("Modules:");
        for module in modules {
            println!("  - {}", module.module_name);
        }
    }

    if classes_only || !modules_only {
        println!("\nClasses:");
        for module in modules {
            for class in &module.classes {
                println!("  - {} (in {})", class.class_name, module.module_name);
            }
        }
    }

    Ok(())
}

fn parse_type_mapping(s: &str) -> Result<(String, String), String> {
    let parts: Vec<&str> = s.split('=').collect();
    if parts.len() != 2 {
        return Err(format!(
            "Invalid type mapping format: '{s}'. Expected format: 'python_type=rust_type'"
        ));
    }
    Ok((parts[0].to_string(), parts[1].to_string()))
}
