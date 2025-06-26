#![allow(dead_code)]

use clap::Parser;

/// Common CLI arguments for CARLA examples
#[derive(Parser, Debug, Clone)]
pub struct CommonArgs {
    /// CARLA server hostname
    #[arg(short = 'H', long, default_value = "localhost")]
    pub host: String,

    /// CARLA server port
    #[arg(short = 'p', long, default_value_t = 2000)]
    pub port: u16,

    /// Connection timeout in seconds
    #[arg(short = 't', long, default_value_t = 10)]
    pub timeout: u32,

    /// Enable verbose output
    #[arg(short, long)]
    pub verbose: bool,

    /// Clean up all actors before starting
    #[arg(short, long)]
    pub clean: bool,
}

impl CommonArgs {
    /// Initialize logging based on verbosity setting
    pub fn init_logging(&self) {
        let filter = if self.verbose { "debug" } else { "info" };
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(filter))
            .format_timestamp(None)
            .init();
    }

    /// Print connection info
    pub fn print_connection_info(&self) {
        println!(
            "Connecting to CARLA server at {}:{}...",
            self.host, self.port
        );
    }
}
