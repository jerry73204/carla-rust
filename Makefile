# CARLA Rust Client Library Makefile
# Build and test automation

.PHONY: build test clean help lint format examples list-examples run-example test-examples test-codegen clean-codegen clean-logs

# Variables
CARLA_HOST ?= localhost
CARLA_PORT ?= 2000

# Log directory with date
LOG_DATE := $(shell date +%Y-%m-%d)
LOG_TIME := $(shell date +%H-%M-%S)
LOG_DIR := logs/$(LOG_DATE)
LOG_SUFFIX := $(LOG_DATE)_$(LOG_TIME)

# Default target
all: build

# Build the entire Rust workspace with all targets
build:
	@mkdir -p $(LOG_DIR)
	@echo "Building all targets... (output in $(LOG_DIR)/build_$(LOG_SUFFIX).log)"
	@cargo build --all-targets --all-features 2>&1 | tee $(LOG_DIR)/build_$(LOG_SUFFIX).log

# Run unit tests only (no CARLA server connection)
test:
	@mkdir -p $(LOG_DIR)
	@echo "Running unit tests only... (output in $(LOG_DIR)/test_$(LOG_SUFFIX).log)"
	@cargo test --lib --no-fail-fast 2>&1 | tee $(LOG_DIR)/test_$(LOG_SUFFIX).log

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

# Clean old logs (keep last 7 days)
clean-logs:
	@echo "Cleaning logs older than 7 days..."
	@find logs -type f -name "*.log" -mtime +7 -delete 2>/dev/null || true
	@find logs -type d -empty -delete 2>/dev/null || true
	@echo "✅ Old logs cleaned"

lint:
	@mkdir -p $(LOG_DIR)
	@echo "Running lint checks... (output in $(LOG_DIR)/lint_$(LOG_SUFFIX).log)"
	@echo "=== Running cargo fmt check ===" > $(LOG_DIR)/lint_$(LOG_SUFFIX).log
	@cargo +nightly fmt --all -- --check 2>&1 | tee -a $(LOG_DIR)/lint_$(LOG_SUFFIX).log
	@echo -e "\n=== Running cargo clippy on all targets ===" >> $(LOG_DIR)/lint_$(LOG_SUFFIX).log
	@cargo clippy --all-targets --all-features -- -D warnings 2>&1 | tee -a $(LOG_DIR)/lint_$(LOG_SUFFIX).log
	@echo -e "\n=== Running cargo clippy on examples ===" >> $(LOG_DIR)/lint_$(LOG_SUFFIX).log
	@cargo clippy --examples -- -D warnings 2>&1 | tee -a $(LOG_DIR)/lint_$(LOG_SUFFIX).log
	@echo "Lint complete. Check $(LOG_DIR)/lint_$(LOG_SUFFIX).log for details."

format:
	cargo +nightly fmt

# Test code generation on CARLA YAML files
test-codegen:
	@mkdir -p $(LOG_DIR)
	@echo "Testing CARLA code generation... (output in $(LOG_DIR)/test-codegen_$(LOG_SUFFIX).log)"
	@echo "================================"
	@(cd carla-codegen && ../scripts/test-codegen.sh) 2>&1 | tee $(LOG_DIR)/test-codegen_$(LOG_SUFFIX).log

# Clean generated test files
clean-codegen:
	@echo "Cleaning code generation test output..."
	@rm -rf carla-test-codegen/src
	@rm -rf carla-test-codegen/target
	@rm -f carla-test-codegen/Cargo.lock
	@rm -rf carla-codegen/test_temp carla-codegen/test_crate carla-codegen/test_generated_carla
	@rm -rf carla-codegen/debug_generated carla-codegen/debug_temp
	@echo "✅ Cleaned code generation test directories"

# Show recent logs
show-logs:
	@echo "Recent log files:"
	@echo "================="
	@if [ -d logs ]; then \
		find logs -type f -name "*.log" -printf "%T@ %p\n" | sort -rn | head -20 | cut -d' ' -f2- | while read -r file; do \
			size=$$(du -h "$$file" | cut -f1); \
			echo "$$size	$$file"; \
		done; \
	else \
		echo "No logs directory found. Run a command to generate logs."; \
	fi

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
	@echo "  test-codegen  - Test code generation on CARLA YAML files (logs saved)"
	@echo "  clean-codegen - Clean code generation test output"
	@echo "  clean         - Clean up build artifacts"
	@echo "  clean-logs    - Clean logs older than 7 days"
	@echo "  show-logs     - Show recent log files"
	@echo "  help          - Show this help message"
	@echo ""
	@echo "Logs are saved in: logs/YYYY-MM-DD/"
	@echo ""
	@echo "Example usage:"
	@echo "  make run-example EXAMPLE=basics/01_connection"
	@echo "  make run-example EXAMPLE=actors/01_spawn_vehicle CARLA_HOST=192.168.1.100"
	@echo "  make run-example EXAMPLE=basics/02_world_info ARGS='--verbose --clean'"
