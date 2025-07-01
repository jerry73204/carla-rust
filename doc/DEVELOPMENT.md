# Development Guide

## Overview

This guide covers development workflows, build processes, testing strategies, and quality assurance for the CARLA Rust client library. The project uses Make for build automation and follows Rust best practices.

## Quick Start

### Prerequisites

**Required**:
- Rust toolchain (stable + nightly for formatting)
- GNU Make
- Git
- cargo-nextest (for enhanced test execution)

**For Full Development** (optional):
- CARLA simulator server (0.10.0)
- CMake (for building CARLA C++ library)
- Clang 12+ (for C++ FFI bindings)

### Environment Setup

```bash
# Clone the repository
git clone https://github.com/jerry73204/carla-rust.git
cd carla-rust

# Install Rust nightly for formatting
rustup install nightly
rustup component add rustfmt --toolchain nightly

# Install cargo-nextest for enhanced testing
cargo install cargo-nextest

# Verify setup
make help
```

### Ubuntu 22.04+ Setup

```bash
# Install required system dependencies
sudo apt install clang-12 libclang-12-dev cmake

# Set environment variables for build
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
```

## Build System

### Core Make Targets

The project uses a Makefile for consistent development workflows:

#### **Build Commands**

```bash
# Build entire workspace with all targets
make build

# Build specific components (use make build unless manual modification needed)
make build                           # Preferred: full workspace build via make
cargo build                         # Manual: basic build
cargo build --all-targets           # Manual: all targets (bins, tests, examples)
cargo build --all-features          # Manual: all features enabled
cargo build --release               # Manual: release build

# Check compilation without building
cargo check --all-targets
```

#### **Testing Commands**

```bash
# Run unit tests (no CARLA server required)
make test

# Run tests with nextest for complete execution
cargo nextest run --no-fail-fast

# Run specific test with nextest
cargo nextest run --no-fail-fast --package carla --lib client::tests

# Check test compilation
cargo test --no-run
```

#### **Code Quality Commands**

```bash
# Run linting (clippy + format check)
make lint

# Format code
make format

# Manual quality checks
cargo clippy --all-targets --all-features -- -D warnings
cargo +nightly fmt --all -- --check
```

#### **Example Commands**

```bash
# List all available examples
make list-examples

# Build all examples
make examples

# Run specific example with default server
make run-example EXAMPLE=01_connection

# Run with custom server and arguments
make run-example EXAMPLE=01_spawn_vehicle CARLA_HOST=192.168.1.100 CARLA_PORT=3000
make run-example EXAMPLE=02_world_info ARGS="--verbose --clean"

# Test example compilation
make test-examples
```

#### **Cleanup Commands**

```bash
# Clean build artifacts
make clean

# Full cleanup including logs
rm -rf target/ /tmp/carla_test_logs/
```

### Build Features

The project supports several feature flags for different build scenarios:

#### **carla-sys Features**

```bash
# Use documentation stubs (for docs.rs)
cargo build --features docs-only

# Regenerate code from CARLA source (maintainer mode)
export CARLA_ROOT=/path/to/carla-source
cargo build --features save-bindgen

# Save built library as tarball
cargo build --features save-lib
```

#### **carla Features**

```bash
# Enable code generation support
cargo build --features codegen

# Build with all features
cargo build --all-features
```

## Testing Strategy

### Nextest Integration

The project uses `cargo-nextest` for enhanced test execution with the following benefits:

- **Complete Execution**: `--no-fail-fast` ensures every test runs to completion
- **Better Output**: Enhanced test reporting and result summaries
- **Accurate Counting**: Precise test result statistics
- **Parallel Execution**: Improved performance over standard `cargo test`

#### **Key Nextest Commands**

```bash
# Run all tests with complete execution (preferred)
cargo nextest run --no-fail-fast

# Run tests for specific package
cargo nextest run --no-fail-fast --package carla

# Run with verbose output
cargo nextest run --no-fail-fast --verbose

# Run specific test pattern
cargo nextest run --no-fail-fast integration_test
```

### Test Categories

#### **1. Unit Tests**
- No external dependencies
- Fast execution
- Test individual components

```bash
# Run all unit tests with complete execution
make test

# Run with nextest for enhanced output and complete execution
cargo nextest run --no-fail-fast

# Run specific test module with nextest
cargo nextest run --no-fail-fast --lib client::tests

# Run with detailed output
cargo nextest run --no-fail-fast --lib -- --nocapture
```

#### **2. Integration Testing via Examples**
- Tests requiring CARLA server have been moved to examples
- Examples serve as integration tests and documentation
- Use examples instead of traditional integration tests

```bash
# Check all examples compile
make test-examples

# Run specific example for integration testing
make run-example EXAMPLE=01_connection

# Run example with server validation
CARLA_HOST=localhost CARLA_PORT=2000 make run-example EXAMPLE=01_spawn_vehicle

# Run examples in sequence for comprehensive testing
make run-example EXAMPLE=01_connection
make run-example EXAMPLE=02_world_info
make run-example EXAMPLE=01_spawn_vehicle
```

#### **3. Documentation Tests**
- Verify code examples in documentation
- Run with nextest when possible

```bash
# Test documentation examples
cargo test --doc

# Test specific crate docs with nextest (when supported)
cargo nextest run --no-fail-fast --doc --package carla

# Fallback to standard cargo test for doc tests
cargo test --doc --package carla
```

### Test Configuration

#### **Server Configuration**

```bash
# Default server
make run-example EXAMPLE=01_connection

# Custom server
make run-example EXAMPLE=01_connection CARLA_HOST=192.168.1.100 CARLA_PORT=3000

# With additional arguments
make run-example EXAMPLE=01_connection ARGS="--timeout 10 --verbose"
```

#### **Test Environment Variables**

```bash
# Enable debug logging for tests
RUST_LOG=debug cargo nextest run --no-fail-fast

# Run tests with trace-level logging
RUST_LOG=trace cargo nextest run --no-fail-fast

# Custom server configuration for examples
export CARLA_HOST=127.0.0.1
export CARLA_PORT=2000
make run-example EXAMPLE=01_connection
```

## Code Quality Assurance

### Linting Configuration

The project enforces strict code quality standards:

#### **Clippy Configuration**

```toml
# In Cargo.toml
[lints.clippy]
all = { level = "warn", priority = -1 }
pedantic = { level = "warn", priority = -1 }
nursery = { level = "warn", priority = -1 }
cargo = { level = "warn", priority = -1 }

# Specific overrides
missing_docs_in_private_items = "allow"
module_name_repetitions = "allow"
```

#### **Running Lints**

```bash
# Full lint check (used in CI)
make lint

# Fix auto-fixable issues
cargo clippy --fix --all-targets --all-features

# Check specific warnings
cargo clippy --all-targets -- -W clippy::todo
cargo clippy --all-targets -- -W clippy::unwrap_used
```

### Code Formatting

```bash
# Format all code
make format

# Check formatting without changes
cargo +nightly fmt --all -- --check

# Format specific files
cargo +nightly fmt -- src/lib.rs src/client.rs
```

### Documentation Standards

#### **Documentation Requirements**

- All public APIs must have documentation
- Include examples in documentation
- Document error conditions
- Explain safety requirements for unsafe code

#### **Documentation Commands**

```bash
# Build documentation
cargo doc --open

# Build with all features
cargo doc --all-features --open

# Check documentation links
cargo doc --all-features 2>&1 | grep warning

# Test documentation examples
cargo test --doc
```

#### **Documentation Style**

```rust
/// Brief description of the function.
///
/// Longer description explaining the purpose, behavior, and any important
/// implementation details.
///
/// # Arguments
/// * `param1` - Description of parameter 1
/// * `param2` - Description of parameter 2  
///
/// # Returns
/// Description of return value and semantics
///
/// # Errors
/// Description of error conditions:
/// - `CarlaError::ConnectionFailed` - When connection fails
/// - `CarlaError::InvalidArgument` - When arguments are invalid
///
/// # Example
/// ```rust,no_run
/// use carla::{Client, ClientBuilder};
/// 
/// let client = ClientBuilder::new()
///     .host("localhost")
///     .port(2000)
///     .connect()?;
/// # Ok::<(), carla::CarlaError>(())
/// ```
///
/// # Safety
/// (Only for unsafe functions)
/// Explain safety requirements and invariants.
pub fn example_function(param1: &str, param2: u32) -> Result<String> {
    // Implementation...
}
```

## Development Workflows

### Feature Development

#### **1. Create Feature Branch**

```bash
git checkout -b feature/new-sensor-api
```

#### **2. Development Cycle**

```bash
# Build and test frequently
make build
make test

# Run comprehensive tests with nextest
cargo nextest run --no-fail-fast

# Check code quality
make lint

# Test with CARLA server examples
make run-example EXAMPLE=sensor_demo

# Document changes
cargo doc --open
```

#### **3. Pre-Commit Checklist**

```bash
# Format code
make format

# Full test suite with complete execution
make test
cargo nextest run --no-fail-fast
make test-examples

# Quality checks
make lint

# Documentation
cargo test --doc
cargo doc --all-features
```

### Bug Fix Workflow

#### **1. Reproduce Issue**

```bash
# Create minimal reproduction case
make run-example EXAMPLE=reproduce_bug

# Add test case for bug
cargo nextest run --no-fail-fast bug_reproduction_test
```

#### **2. Fix and Validate**

```bash
# Implement fix
make build
make test

# Run comprehensive test suite
cargo nextest run --no-fail-fast

# Verify fix with integration test
make run-example EXAMPLE=test_fix
```

#### **3. Regression Prevention**

```bash
# Add regression test
# Update documentation if needed
# Ensure all tests pass with complete execution
make test
cargo nextest run --no-fail-fast
make test-examples
```

### Release Workflow

#### **1. Pre-Release Checklist**

```bash
# Full test suite with complete execution
make test
cargo nextest run --no-fail-fast
make test-examples

# Code quality
make lint
cargo audit

# Documentation
cargo doc --all-features
cargo test --doc

# Examples work
make examples
```

#### **2. Version Management**

```bash
# Update version in Cargo.toml files
# Update CHANGELOG.md
# Create git tag
git tag -a v0.2.0 -m "Release v0.2.0"
```

#### **3. Publication**

```bash
# Dry run
cargo publish --dry-run --package carla-sys
cargo publish --dry-run --package carla

# Actual publication
cargo publish --package carla-sys
cargo publish --package carla
```

## IDE Integration

### VS Code Configuration

```json
// .vscode/settings.json
{
    "rust-analyzer.cargo.allTargets": true,
    "rust-analyzer.cargo.allFeatures": true,
    "rust-analyzer.check.command": "clippy",
    "rust-analyzer.check.allTargets": true,
    "rust-analyzer.procMacro.enable": true,
    "files.exclude": {
        "target/": true,
        "**/*.rs.bk": true
    }
}
```

### Recommended Extensions

- rust-analyzer
- CodeLLDB (for debugging)
- Better TOML
- Error Lens

## Debugging

### Debug Builds

```bash
# Debug build with symbols
cargo build
RUST_BACKTRACE=1 cargo run --example debug_test

# Release build with debug info
cargo build --release --config profile.release.debug=true
```

### Logging Configuration

```bash
# Enable logging
RUST_LOG=debug cargo run --example 01_connection
RUST_LOG=carla=trace cargo run --example detailed_debug

# Custom log levels with nextest
RUST_LOG=carla::client=debug,carla::sensor=trace cargo nextest run --no-fail-fast
```

### Profiling

```bash
# Install profiling tools
cargo install cargo-flamegraph

# Profile example
cargo flamegraph --example performance_test

# Memory profiling
valgrind --tool=memcheck target/debug/examples/memory_test
```

## Continuous Integration

### GitHub Actions

The project uses GitHub Actions for CI/CD:

```yaml
# .github/workflows/ci.yml (example)
name: CI
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
      - run: make build
      - run: make test
      - run: make lint
```

### Local CI Simulation

```bash
# Run the same checks as CI
make build
make test
cargo nextest run --no-fail-fast
make lint
make test-examples

# Check for common issues
cargo audit
cargo outdated
```

## Performance Considerations

### Build Performance

```bash
# Parallel builds
cargo build -j $(nproc)

# Incremental builds
export CARGO_INCREMENTAL=1

# Faster linker (Linux)
sudo apt install lld
export RUSTFLAGS="-C link-arg=-fuse-ld=lld"
```

### Runtime Performance

```bash
# Release builds for performance testing
cargo build --release --example performance_test

# Profile-guided optimization
export RUSTFLAGS="-C profile-generate=/tmp/pgo-data"
cargo build --release
# Run workload
export RUSTFLAGS="-C profile-use=/tmp/pgo-data"
cargo build --release
```

## Troubleshooting

### Common Build Issues

#### **Missing CARLA Dependencies**

```bash
# Error: CARLA_ROOT not found
export CARLA_ROOT=/path/to/carla-source

# Error: CMake not found
sudo apt install cmake

# Error: Clang version mismatch
export CLANG_PATH=/usr/bin/clang-12
```

#### **Linking Issues**

```bash
# Update library paths
export LD_LIBRARY_PATH=$CARLA_ROOT/lib:$LD_LIBRARY_PATH

# Check library dependencies
ldd target/debug/examples/01_connection
```

#### **Runtime Issues**

```bash
# Connection refused
# Ensure CARLA server is running
cd /path/to/carla && ./CarlaUnreal.sh

# Permission denied
# Check firewall and port availability
netstat -an | grep 2000
```

### Getting Help

#### **Documentation**

```bash
# Local documentation
cargo doc --open

# Command help
make help
cargo build --help
```

#### **Community**

- GitHub Issues: Report bugs and feature requests
- GitHub Discussions: Ask questions and share ideas
- Discord: Real-time community support

## Contributing

### Development Setup

```bash
# Fork and clone repository
git clone https://github.com/your-username/carla-rust.git
cd carla-rust

# Set up development environment
make build
make test
cargo nextest run --no-fail-fast

# Create feature branch
git checkout -b feature/your-feature
```

### Contribution Guidelines

1. **Code Quality**: All code must pass `make lint`
2. **Tests**: Add tests for new functionality
3. **Documentation**: Document all public APIs
4. **Examples**: Provide usage examples
5. **Compatibility**: Maintain backward compatibility

### Pull Request Process

1. Create feature branch from main
2. Implement changes with tests
3. Run full test suite: `make test && cargo nextest run --no-fail-fast && make test-examples`
4. Update documentation as needed
5. Submit pull request with clear description

This development guide provides comprehensive coverage of the build system, testing strategies, and development workflows needed to effectively contribute to the CARLA Rust client library.