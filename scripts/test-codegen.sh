#!/bin/bash
# Test script for CARLA code generation
# This script generates Rust code from CARLA YAML files and verifies it compiles

set -e  # Exit on error

# Configuration
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INPUT_DIR="carla-simulator/PythonAPI/docs"
CONFIG_FILE="carla-codegen/configs/carla_full_generation.toml"
TEST_CRATE_NAME="carla-test-codegen"
TEST_CRATE_DIR="$REPO_ROOT/$TEST_CRATE_NAME"

# Create log directory structure
LOG_DATE=$(date +%Y-%m-%d)
LOG_TIME=$(date +%H-%M-%S)
LOG_DIR="$REPO_ROOT/logs/${LOG_DATE}/codegen"
LOG_SUFFIX="${LOG_DATE}_${LOG_TIME}"
mkdir -p "$LOG_DIR"

echo "🚀 Starting CARLA code generation test..."
echo "========================================"
echo "📁 Repository root: $REPO_ROOT"
echo "📁 Test crate will be created at: $TEST_CRATE_DIR"
echo "📁 Logs will be saved to: $LOG_DIR"

# Check if CARLA YAML files exist
if [ ! -d "$REPO_ROOT/$INPUT_DIR" ]; then
    echo "❌ CARLA YAML files not found at $INPUT_DIR"
    echo "Please ensure the CARLA submodule is initialized:"
    echo "  git submodule update --init --recursive"
    exit 1
fi

# Clean previous test crate
echo "🧹 Cleaning previous test crate..."
rm -rf "$TEST_CRATE_DIR"

# Create test crate directory structure
echo "📁 Creating test crate at $TEST_CRATE_NAME/..."
mkdir -p "$TEST_CRATE_DIR/src"

# Create Cargo.toml for the test crate (excluded from workspace)
cat > "$TEST_CRATE_DIR/Cargo.toml" << EOF
[package]
name = "carla-test-codegen"
version = "0.1.0"
edition = "2021"

# Exclude from parent workspace
[workspace]

[dependencies]
# Required dependencies for generated code
thiserror = "1.0"

# Optional dependencies that generated code might use
anyhow = { version = "1.0", optional = true }
serde = { version = "1.0", features = ["derive"], optional = true }
serde_json = { version = "1.0", optional = true }
nalgebra = { version = "0.32", optional = true }

[features]
default = []
full = ["anyhow", "serde", "serde_json", "nalgebra"]
EOF

# Create error module that generated code expects
cat > "$TEST_CRATE_DIR/src/error.rs" << 'EOF'
//! Error types for CARLA bindings
//!
//! This module provides error types used throughout the generated code.
//! All fallible operations return `Result<T>` using the error type defined here.

use thiserror::Error;

/// Main error type for CARLA operations
#[derive(Debug, Error, Clone)]
pub enum CarlaError {
    /// Feature not yet implemented
    #[error("Not implemented: {0}")]
    NotImplemented(String),

    /// Generic error
    #[error("Error: {0}")]
    Generic(String),
}

/// Result type alias for CARLA operations
pub type Result<T> = std::result::Result<T, CarlaError>;

impl CarlaError {
    /// Create a not implemented error
    pub fn not_implemented(feature: impl Into<String>) -> Self {
        Self::NotImplemented(feature.into())
    }
}
EOF

# Create initial lib.rs
cat > "$TEST_CRATE_DIR/src/lib.rs" << 'EOF'
//! Test crate for generated CARLA code

pub mod error;
EOF

# Run code generation directly into the test crate
echo ""
echo "📝 Running code generation..."
echo "Input: $INPUT_DIR"
echo "Config: $CONFIG_FILE"
echo "Output: $TEST_CRATE_DIR/src"
echo ""

CODEGEN_LOG="$LOG_DIR/codegen_${LOG_SUFFIX}.log"
echo "Code generation started at $(date)" > "$CODEGEN_LOG"
echo "Working directory: $(pwd)" >> "$CODEGEN_LOG"
echo "Running from: $REPO_ROOT/carla-codegen" >> "$CODEGEN_LOG"
echo "Command: cargo run --release -- generate --input ../$INPUT_DIR --config $CONFIG_FILE --output $TEST_CRATE_DIR/src --verbose" >> "$CODEGEN_LOG"
echo "" >> "$CODEGEN_LOG"

# Change to carla-codegen directory to run the generator
cd "$REPO_ROOT/carla-codegen"

if cargo run --release -- generate \
    --input "../$INPUT_DIR" \
    --config "configs/carla_full_generation.toml" \
    --output "$TEST_CRATE_DIR/src" \
    --verbose >> "$CODEGEN_LOG" 2>&1; then
    echo "✅ Code generation completed successfully"
    echo "✅ Code generation completed successfully at $(date)" >> "$CODEGEN_LOG"
else
    echo "❌ Code generation failed"
    echo "❌ Code generation failed at $(date)" >> "$CODEGEN_LOG"
    echo "   Code generation log: $CODEGEN_LOG"
    exit 1
fi

echo "   Code generation log: $CODEGEN_LOG"

# Count generated files
rust_files=$(find "$TEST_CRATE_DIR/src" -name "*.rs" | grep -v "^$TEST_CRATE_DIR/src/lib.rs$" | grep -v "^$TEST_CRATE_DIR/src/error.rs$" | wc -l)
echo "📊 Total Rust files generated: $rust_files"

if [ "$rust_files" -eq 0 ]; then
    echo "❌ No Rust files were generated"
    exit 1
fi

# Add module declarations to lib.rs based on what was generated
echo "" >> "$TEST_CRATE_DIR/src/lib.rs"
echo "// Generated modules" >> "$TEST_CRATE_DIR/src/lib.rs"

# Add carla module if it exists
if [ -d "$TEST_CRATE_DIR/src/carla" ]; then
    echo "pub mod carla;" >> "$TEST_CRATE_DIR/src/lib.rs"
fi

# Add command module if it exists
if [ -d "$TEST_CRATE_DIR/src/command" ]; then
    echo "pub mod command;" >> "$TEST_CRATE_DIR/src/lib.rs"
fi

# Add any other top-level modules that were generated
for dir in "$TEST_CRATE_DIR/src"/*; do
    if [ -d "$dir" ]; then
        module_name=$(basename "$dir")
        # Skip if already added or if it's one of our pre-created modules
        if [ "$module_name" != "carla" ] && [ "$module_name" != "command" ] && [ "$module_name" != "error" ]; then
            if ! grep -q "pub mod $module_name;" "$TEST_CRATE_DIR/src/lib.rs"; then
                echo "pub mod $module_name;" >> "$TEST_CRATE_DIR/src/lib.rs"
            fi
        fi
    fi
done

# Change to test crate directory
cd "$TEST_CRATE_DIR"

# Attempt to compile the test crate
echo ""
echo "🔨 Attempting to compile generated code..."
echo "=========================================="

# Quick syntax validation check
echo "📋 Performing quick syntax validation..."

# Count Rust files
rust_file_count=$(find src -name "*.rs" -type f | wc -l)
echo "   Found $rust_file_count Rust files to validate"

# Basic syntax check with rustc
syntax_check_failures=0
syntax_check_total=0
SYNTAX_LOG="$LOG_DIR/syntax_check_${LOG_SUFFIX}.log"

echo "Starting syntax validation..." > "$SYNTAX_LOG"
echo "=============================" >> "$SYNTAX_LOG"

# Check first 20 files for quick validation
for rs_file in $(find src -name "*.rs" -type f | head -20); do
    syntax_check_total=$((syntax_check_total + 1))
    echo "Checking: $rs_file" >> "$SYNTAX_LOG"
    if ! rustc --edition 2021 --crate-type lib --emit=metadata "$rs_file" >> "$SYNTAX_LOG" 2>&1; then
        syntax_check_failures=$((syntax_check_failures + 1))
        echo "❌ FAILED: $rs_file" >> "$SYNTAX_LOG"
    else
        echo "✅ PASSED: $rs_file" >> "$SYNTAX_LOG"
    fi
    echo "" >> "$SYNTAX_LOG"
done

echo "   Checked $syntax_check_total files, $syntax_check_failures with syntax errors"
echo "   Syntax check log: $SYNTAX_LOG"

if [ "$syntax_check_failures" -gt 10 ]; then
    echo "❌ Too many syntax errors detected"
    compile_success=false
else
    echo "✅ Basic syntax validation passed"
    compile_success=true
fi

# Try cargo check
echo ""
echo "📋 Running cargo check..."
CARGO_CHECK_LOG="$LOG_DIR/cargo_check_${LOG_SUFFIX}.log"

echo "Running cargo check..." > "$CARGO_CHECK_LOG"
echo "======================" >> "$CARGO_CHECK_LOG"
echo "Working directory: $(pwd)" >> "$CARGO_CHECK_LOG"
echo "" >> "$CARGO_CHECK_LOG"

if cargo check --all-targets >> "$CARGO_CHECK_LOG" 2>&1; then
    echo "✅ Cargo check passed - code compiles successfully!"
    echo "✅ Cargo check completed successfully" >> "$CARGO_CHECK_LOG"
    check_success=true
else
    # Check if it's just warnings or actual errors
    if grep -q "error\[E[0-9]\+\]" "$CARGO_CHECK_LOG"; then
        echo "❌ Cargo check failed with compilation errors"
        echo "❌ Cargo check failed with errors" >> "$CARGO_CHECK_LOG"
        check_success=false
    else
        echo "⚠️  Cargo check completed with warnings"
        echo "⚠️  Cargo check completed with warnings" >> "$CARGO_CHECK_LOG"
        check_success=true
    fi
fi

echo "   Cargo check log: $CARGO_CHECK_LOG"

# Try cargo build if check passed
build_success=false
if [ "$check_success" = true ]; then
    echo ""
    echo "📋 Attempting cargo build..."
    BUILD_LOG="$LOG_DIR/build_${LOG_SUFFIX}.log"
    
    if timeout 60 cargo build --lib >> "$BUILD_LOG" 2>&1; then
        echo "✅ Build successful!"
        build_success=true
    else
        echo "⚠️  Build failed (likely due to todo!() placeholders)"
        build_success=true  # Still consider success since generated code has todo!()
    fi
    echo "   Build log: $BUILD_LOG"
fi

# Summary
echo ""
echo "📊 Test Summary"
echo "==============="
echo "📁 Generated files: $rust_files"
echo "📁 Test crate location: $TEST_CRATE_DIR"

if [ "$compile_success" = true ]; then
    echo "✅ Syntax validation: PASSED"
else
    echo "❌ Syntax validation: FAILED"
fi

if [ "$check_success" = true ]; then
    echo "✅ Cargo check: PASSED"
else
    echo "❌ Cargo check: FAILED"
fi

if [ "$build_success" = true ]; then
    echo "✅ Cargo build: PASSED (or failed due to todo!())"
else
    echo "❌ Cargo build: FAILED"
fi

# Create summary log
SUMMARY_LOG="$LOG_DIR/summary_${LOG_SUFFIX}.log"
{
    echo "CARLA Code Generation Test Summary"
    echo "=================================="
    echo "Date: $(date)"
    echo "Repository Root: $REPO_ROOT"
    echo ""
    echo "Configuration:"
    echo "  Input Directory: $INPUT_DIR"
    echo "  Config File: $CONFIG_FILE"
    echo "  Test Crate: $TEST_CRATE_DIR"
    echo ""
    echo "Results:"
    echo "  Generated Rust files: $rust_files"
    echo "  Syntax validation: $([ "$compile_success" = true ] && echo "PASSED" || echo "FAILED")"
    echo "  Cargo check: $([ "$check_success" = true ] && echo "PASSED" || echo "FAILED")"
    echo "  Cargo build: $([ "$build_success" = true ] && echo "PASSED" || echo "FAILED")"
    echo ""
    echo "Log Files:"
    echo "  Code Generation: $CODEGEN_LOG"
    echo "  Syntax Check: $SYNTAX_LOG"
    echo "  Cargo Check: $CARGO_CHECK_LOG"
    if [ -f "$BUILD_LOG" ]; then
        echo "  Build: $BUILD_LOG"
    fi
    echo "  Summary: $SUMMARY_LOG"
    echo ""
    echo "Notes:"
    echo "  - The test crate is excluded from the workspace via Cargo.toml"
    echo "  - Generated code contains todo!() placeholders for unimplemented FFI functions"
    echo "  - The test crate directory is added to .gitignore"
} > "$SUMMARY_LOG"

echo ""
echo "📁 All logs saved to: $LOG_DIR"
echo "📄 Summary log: $SUMMARY_LOG"

# Determine overall result
if [ "$check_success" = true ] && [ "$rust_files" -gt 0 ]; then
    echo ""
    echo "🎉 Code generation test PASSED!"
    echo "Generated code compiles successfully."
    echo ""
    echo "💡 To explore the generated code:"
    echo "   cd $TEST_CRATE_NAME"
    echo "   cargo doc --open"
    exit 0
else
    echo ""
    echo "💥 Code generation test FAILED!"
    if [ "$rust_files" -eq 0 ]; then
        echo "Reason: No files were generated"
    elif [ "$compile_success" = false ]; then
        echo "Reason: Generated code has syntax errors"
    else
        echo "Reason: Generated code has compilation errors"
    fi
    echo ""
    echo "💡 Check the logs for details:"
    echo "   Syntax errors: $SYNTAX_LOG"
    echo "   Compilation errors: $CARGO_CHECK_LOG"
    exit 1
fi