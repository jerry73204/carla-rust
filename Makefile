# CARLA Rust Client Library Makefile
# Simple build and test automation

.PHONY: build test test-server clean help lint

# Default target
all: build

# Build the entire Rust workspace with all targets
build:
	cargo build --all-targets --all-features

# Run tests without CARLA server connection
test:
	cargo nextest run --all-targets --no-fail-fast

# Run tests with CARLA server connection (requires running server)
test-server:
	cargo nextest run --all-targets --features test-carla-server --no-fail-fast

# Clean build artifacts
clean:
	cargo clean

lint:
	cargo clippy --all-targets --all-features

# Show available targets
help:
	@echo "Available targets:"
	@echo "  build       - Build the Rust project with --all-targets"
	@echo "  test        - Run tests without CARLA server connection"
	@echo "  test-server - Run tests with CARLA server connection"
	@echo "  lint        - Run clippy for code quality checks"
	@echo "  clean       - Clean up build artifacts"
	@echo "  help        - Show this help message"
