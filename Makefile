# Makefile for carla-rust

.PHONY: help
help:
	@echo "Usage: make <target>"
	@echo ""
	@echo "Targets:"
	@echo "  build            Build with dev-release profile"
	@echo "  test             Run unit tests"
	@echo "  format           Format Rust and C++ code"
	@echo "  format-rust      Format Rust code only"
	@echo "  format-cpp       Format C++ code only"
	@echo "  lint             Run Rust and C++ lints"
	@echo "  lint-rust        Run Rust lints only"
	@echo "  lint-cpp         Run C++ lints (format + tidy)"
	@echo "  lint-cpp-format  Run C++ format check only"
	@echo "  lint-cpp-tidy    Run clang-tidy only"
	@echo "  clean            Remove build artifacts"
	@echo "  doc              Build documentation"

.PHONY: build
build:
	cargo build --all-targets --profile dev-release

.PHONY: test
test:
	cargo nextest run --no-tests pass --no-fail-fast --cargo-profile dev-release

.PHONY: format
format: format-rust format-cpp

.PHONY: format-rust
format-rust:
	cargo +nightly fmt --all

.PHONY: format-cpp
format-cpp:
	find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i --style=file

.PHONY: lint
lint: lint-rust lint-cpp

.PHONY: lint-rust
lint-rust:
	cargo +nightly fmt --check
	cargo clippy --all-targets -- -D warnings

.PHONY: lint-cpp
lint-cpp: lint-cpp-format lint-cpp-tidy

.PHONY: lint-cpp-format
lint-cpp-format:
	find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format --dry-run --Werror --style=file

.PHONY: lint-cpp-tidy
lint-cpp-tidy:
	@CARLA_INCLUDE=$$(find target/dev-release/build/carla-sys-*/out/libcarla_client/*/include -type d 2>/dev/null | head -1); \
	if [ -z "$$CARLA_INCLUDE" ]; then \
		CARLA_INCLUDE=$$(find target/debug/build/carla-sys-*/out/libcarla_client/*/include -type d 2>/dev/null | head -1); \
	fi; \
	if [ -z "$$CARLA_INCLUDE" ]; then \
		echo "Error: CARLA headers not found. Run 'make build' first."; \
		exit 1; \
	else \
		echo "Using CARLA headers from: $$CARLA_INCLUDE"; \
		echo "Running clang-tidy on all files..."; \
		clang-tidy $$(find carla-sys/csrc -name "*.hpp" -o -name "*.cpp") -- -std=c++14 -Icarla-sys/csrc -I$$CARLA_INCLUDE; \
	fi

.PHONY: clean
clean:
	cargo clean

.PHONY: doc
doc:
	cargo doc --no-deps --open

.DEFAULT_GOAL := help
