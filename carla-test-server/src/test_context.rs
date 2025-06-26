//! Test context utilities for enhanced test organization.

use crate::config::CarlaTestConfig;
use std::sync::Arc;

/// Context information available during test execution.
///
/// This struct provides access to test metadata and configuration
/// that can be useful for debugging and test organization.
#[derive(Clone)]
pub struct TestContext {
    /// Name of the current test
    pub test_name: String,
    /// Test configuration
    pub config: Arc<CarlaTestConfig>,
    /// Start time of the test
    pub start_time: std::time::Instant,
}

impl TestContext {
    /// Creates a new test context.
    pub fn new(test_name: impl Into<String>, config: CarlaTestConfig) -> Self {
        Self {
            test_name: test_name.into(),
            config: Arc::new(config),
            start_time: std::time::Instant::now(),
        }
    }

    /// Gets the elapsed time since test start.
    pub fn elapsed(&self) -> std::time::Duration {
        self.start_time.elapsed()
    }

    /// Logs a test milestone with timing information.
    pub fn log_milestone(&self, message: &str) {
        log::info!(
            "[{}] {} (elapsed: {:?})",
            self.test_name,
            message,
            self.elapsed()
        );
    }

    /// Creates a sub-context for a test phase.
    pub fn phase(&self, phase_name: &str) -> TestPhase {
        TestPhase::new(self.clone(), phase_name)
    }
}

/// Represents a phase within a test for better organization.
pub struct TestPhase {
    context: TestContext,
    phase_name: String,
    start_time: std::time::Instant,
}

impl TestPhase {
    fn new(context: TestContext, phase_name: impl Into<String>) -> Self {
        let phase_name = phase_name.into();
        log::info!("[{}] Starting phase: {}", context.test_name, phase_name);

        Self {
            context,
            phase_name,
            start_time: std::time::Instant::now(),
        }
    }

    /// Logs completion of the phase.
    pub fn complete(self) {
        log::info!(
            "[{}] Completed phase '{}' in {:?}",
            self.context.test_name,
            self.phase_name,
            self.start_time.elapsed()
        );
    }
}

impl Drop for TestPhase {
    fn drop(&mut self) {
        // Auto-log completion if not explicitly called
        if !std::thread::panicking() {
            log::info!(
                "[{}] Phase '{}' ended after {:?}",
                self.context.test_name,
                self.phase_name,
                self.start_time.elapsed()
            );
        }
    }
}

/// Type alias for configuration override functions.
type ConfigOverride = Box<dyn FnOnce(&mut CarlaTestConfig)>;

/// Builder for constructing test scenarios with common setup.
pub struct TestScenarioBuilder {
    name: String,
    config_overrides: Vec<ConfigOverride>,
}

impl TestScenarioBuilder {
    /// Creates a new test scenario builder.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            config_overrides: Vec::new(),
        }
    }

    /// Overrides the server port.
    pub fn with_port(mut self, port: u16) -> Self {
        self.config_overrides.push(Box::new(move |config| {
            config.server.port = port;
        }));
        self
    }

    /// Overrides the quality level.
    pub fn with_quality(mut self, quality: impl Into<String>) -> Self {
        let quality = quality.into();
        self.config_overrides.push(Box::new(move |config| {
            config.server.quality_level = quality;
        }));
        self
    }

    /// Sets windowed mode.
    pub fn windowed(mut self, windowed: bool) -> Self {
        self.config_overrides.push(Box::new(move |config| {
            config.server.windowed = windowed;
        }));
        self
    }

    /// Adds additional server arguments.
    pub fn with_args(mut self, args: Vec<String>) -> Self {
        self.config_overrides.push(Box::new(move |config| {
            config.server.additional_args.extend(args);
        }));
        self
    }

    /// Builds the test configuration.
    pub fn build(self) -> Result<(TestContext, CarlaTestConfig), Box<dyn std::error::Error>> {
        let mut config = crate::config::load_config()?;

        // Apply all overrides
        for override_fn in self.config_overrides {
            override_fn(&mut config);
        }

        let context = TestContext::new(&self.name, config.clone());
        Ok((context, config))
    }
}

/// Macro to simplify test scenario creation.
///
/// Example:
/// ```rust
/// test_scenario!("test_high_quality", |builder| {
///     builder.with_quality("Epic").windowed(true)
/// });
/// ```
#[macro_export]
macro_rules! test_scenario {
    ($name:expr, |$builder:ident| $config:expr) => {{
        let $builder = $crate::test_context::TestScenarioBuilder::new($name);
        let builder = $config;
        builder.build()
    }};
}
