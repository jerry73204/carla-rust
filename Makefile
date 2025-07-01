# CARLA Rust Client Library Makefile
# Build and test automation

.PHONY: build test clean help lint format examples list-examples run-example test-examples test-codegen clean-codegen

# Variables
CARLA_HOST ?= localhost
CARLA_PORT ?= 2000

# Default target
all: build

# Build the entire Rust workspace with all targets
build:
	cargo build --all-targets --all-features

# Run unit tests only (no CARLA server connection)
test:
	@echo "Running unit tests only..."
	cargo test --lib --no-fail-fast

# Example-related targets
examples:
	@echo "Building all examples..."
	cargo build --examples --all-features

list-examples:
	@echo "Available examples:"
	@echo "=================="
	@echo "Basics:"
	@echo "  01_connection     - Basic connection and server info"
	@echo "  02_world_info     - World information and actors"
	@echo "  03_blueprints     - Blueprint library exploration"
	@echo ""
	@echo "Actors:"
	@echo "  01_spawn_vehicle  - Spawn and control a vehicle"
	@echo ""
	@echo "Legacy examples:"
	@echo "  actor_list_usage  - ActorList usage patterns"
	@echo "  actor_list_chaining - ActorList method chaining"

run-example:
ifndef EXAMPLE
	@echo "Error: EXAMPLE not specified"
	@echo "Usage: make run-example EXAMPLE=01_connection"
	@echo "       make run-example EXAMPLE=01_spawn_vehicle"
	@echo ""
	@$(MAKE) list-examples
else
	@echo "Running example: $(EXAMPLE)"
	@echo "Server: $(CARLA_HOST):$(CARLA_PORT)"
	cargo run --example $(EXAMPLE) -- --host $(CARLA_HOST) --port $(CARLA_PORT) $(ARGS)
endif

test-examples:
	@echo "Testing all examples can build..."
	cargo check --examples

# Clean build artifacts
clean:
	cargo clean
	rm -rf target/
	rm -rf /tmp/carla_test_logs/

lint:
	cargo +nightly fmt --all -- --check
	cargo clippy --all-targets --all-features -- -D warnings
	cargo clippy --examples -- -D warnings

format:
	cargo +nightly fmt

# Test code generation on CARLA YAML files
test-codegen:
	@echo "Testing CARLA code generation..."
	@echo "================================"
	@cd carla-codegen && \
	if [ -d ../carla-simulator/PythonAPI/docs ]; then \
		cargo run --release -- generate \
			--input ../carla-simulator/PythonAPI/docs \
			--config configs/carla_full_generation.toml \
			--output test_temp \
			--verbose || true; \
		if [ -d test_temp ]; then \
			echo "📁 Generated files in: carla-codegen/test_temp"; \
			find test_temp -name "*.rs" | wc -l | xargs echo "📊 Total Rust files generated:"; \
			echo "⚠️  Note: Some files may have syntax errors due to complex type mappings"; \
		else \
			echo "❌ No output directory created"; \
		fi; \
	else \
		echo "❌ CARLA YAML files not found at carla-simulator/PythonAPI/docs/"; \
		echo "Please ensure the CARLA submodule is initialized:"; \
		echo "  git submodule update --init --recursive"; \
		exit 1; \
	fi

# Clean generated test files
clean-codegen:
	@echo "Cleaning code generation test output..."
	@rm -rf carla-codegen/test_temp carla-codegen/test_generated_carla
	@echo "✅ Cleaned code generation test directories"

# Show available targets
help:
	@echo "Available targets:"
	@echo "  build         - Build the Rust project with all targets"
	@echo "  test          - Run unit tests only (no CARLA server)"
	@echo "  examples      - Build all examples"
	@echo "  list-examples - List all available examples"
	@echo "  run-example   - Run a specific example (requires EXAMPLE=name)"
	@echo "  test-examples - Check that all examples compile"
	@echo "  lint          - Run clippy and format check"
	@echo "  format        - Format code with cargo +nightly fmt"
	@echo "  test-codegen  - Test code generation on CARLA YAML files"
	@echo "  clean-codegen - Clean code generation test output"
	@echo "  clean         - Clean up build artifacts"
	@echo "  help          - Show this help message"
	@echo ""
	@echo "Example usage:"
	@echo "  make run-example EXAMPLE=basics/01_connection"
	@echo "  make run-example EXAMPLE=actors/01_spawn_vehicle CARLA_HOST=192.168.1.100"
	@echo "  make run-example EXAMPLE=basics/02_world_info ARGS='--verbose --clean'"
