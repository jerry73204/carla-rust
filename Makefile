# Makefile for carla-rust

.PHONY: help
help:
	@echo "Usage: make <target>"
	@echo ""
	@echo "Targets:"
	@echo "  build        Build with dev-release profile"
	@echo "  test         Run unit tests"
	@echo "  format       Format Rust and C++ code"
	@echo "  format-rust  Format Rust code only"
	@echo "  format-cpp   Format C++ code only"
	@echo "  lint         Run Rust and C++ lints"
	@echo "  lint-rust    Run Rust lints only"
	@echo "  lint-cpp     Run C++ lints only"
	@echo "  clean        Remove build artifacts"
	@echo "  doc          Build documentation"

.PHONY: build
build:
	cargo build --all-targets --profile dev-release

.PHONY: test
test:
	cargo nextest run --no-fail-fast --cargo-profile dev-release

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
lint-cpp:
	find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format --dry-run --Werror --style=file

.PHONY: clean
clean:
	cargo clean

.PHONY: doc
doc:
	cargo doc --no-deps --open

.DEFAULT_GOAL := help
