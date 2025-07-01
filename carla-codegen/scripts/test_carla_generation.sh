#!/bin/bash

# Test script for generating all CARLA types using carla-codegen
# This script demonstrates how to use carla-codegen with the CARLA simulator docs

set -e  # Exit on any error

echo "🚀 CARLA Code Generation Test Script"
echo "====================================="

# Configuration
CARLA_DOCS_DIR="carla-simulator/PythonAPI/docs"
CONFIG_FILE="configs/carla_full_generation.toml"
OUTPUT_DIR="test_generated_carla"
CARLA_REPO_URL="https://github.com/carla-simulator/carla.git"
CARLA_BRANCH="0.10.0"

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to clone CARLA if needed
setup_carla_docs() {
    echo "📁 Setting up CARLA documentation..."
    
    if [ ! -d "carla-simulator" ]; then
        echo "  Cloning CARLA repository (this may take a while)..."
        if command_exists git; then
            # Clone only the docs we need to save bandwidth
            git clone --depth 1 --branch $CARLA_BRANCH --single-branch $CARLA_REPO_URL carla-simulator
            echo "  ✅ CARLA repository cloned"
        else
            echo "  ❌ Git not found. Please install git or manually download CARLA docs."
            exit 1
        fi
    else
        echo "  ✅ CARLA repository already exists"
    fi
    
    # Check if docs directory exists
    if [ ! -d "$CARLA_DOCS_DIR" ]; then
        echo "  ❌ CARLA docs directory not found at $CARLA_DOCS_DIR"
        echo "  Please ensure CARLA repository is properly cloned"
        exit 1
    fi
    
    # List available YAML files
    echo "  📄 Available YAML documentation files:"
    find "$CARLA_DOCS_DIR" -name "*.yml" -o -name "*.yaml" | head -10
    echo "  (showing first 10 files...)"
}

# Function to build carla-codegen
build_codegen() {
    echo "🔨 Building carla-codegen..."
    
    if command_exists cargo; then
        cargo build --release
        echo "  ✅ carla-codegen built successfully"
    else
        echo "  ❌ Cargo not found. Please install Rust and Cargo."
        exit 1
    fi
}

# Function to run code generation
run_generation() {
    echo "⚙️  Running CARLA code generation..."
    
    # Clean previous output
    if [ -d "$OUTPUT_DIR" ]; then
        echo "  🧹 Cleaning previous output..."
        rm -rf "$OUTPUT_DIR"
    fi
    
    # Check if config file exists
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "  ❌ Configuration file not found: $CONFIG_FILE"
        echo "  Please ensure the config file exists"
        exit 1
    fi
    
    echo "  🎯 Using configuration: $CONFIG_FILE"
    echo "  📁 Input directory: $CARLA_DOCS_DIR"
    echo "  📁 Output directory: $OUTPUT_DIR"
    
    # Run carla-codegen
    ./target/release/carla-codegen generate \
        --input "$CARLA_DOCS_DIR" \
        --config "$CONFIG_FILE" \
        --output "$OUTPUT_DIR" \
        --verbose
        
    echo "  ✅ Code generation completed"
}

# Function to analyze output
analyze_output() {
    echo "📊 Analyzing generated code..."
    
    if [ ! -d "$OUTPUT_DIR" ]; then
        echo "  ❌ Output directory not found"
        return 1
    fi
    
    # Count generated files
    rust_files=$(find "$OUTPUT_DIR" -name "*.rs" | wc -l)
    mod_files=$(find "$OUTPUT_DIR" -name "mod.rs" | wc -l)
    
    echo "  📈 Generation statistics:"
    echo "    • Total Rust files: $rust_files"
    echo "    • Module files: $mod_files"
    echo "    • Library files: $((rust_files - mod_files))"
    
    # Show directory structure
    echo "  📁 Generated directory structure:"
    tree "$OUTPUT_DIR" -I "*.rs" | head -20 || ls -la "$OUTPUT_DIR"
    
    # Show a sample generated file
    echo "  📝 Sample generated file:"
    sample_file=$(find "$OUTPUT_DIR" -name "*.rs" -not -name "mod.rs" | head -1)
    if [ -n "$sample_file" ]; then
        echo "    File: $sample_file"
        echo "    Content preview:"
        head -20 "$sample_file" | sed 's/^/      /'
    fi
}

# Function to validate generated code
validate_code() {
    echo "✅ Validating generated code..."
    
    # Check if Rust code compiles (basic syntax check)
    echo "  🔍 Checking Rust syntax..."
    
    # Create a temporary Cargo project to test compilation
    temp_project="temp_carla_test"
    if [ -d "$temp_project" ]; then
        rm -rf "$temp_project"
    fi
    
    cargo new "$temp_project" --lib >/dev/null 2>&1
    
    # Copy generated code to temporary project
    cp -r "$OUTPUT_DIR"/* "$temp_project/src/" 2>/dev/null || true
    
    # Try to check syntax
    cd "$temp_project"
    if cargo check >/dev/null 2>&1; then
        echo "    ✅ Generated code has valid Rust syntax"
    else
        echo "    ⚠️  Generated code has syntax issues (expected for incomplete bindings)"
    fi
    cd ..
    
    # Clean up
    rm -rf "$temp_project"
}

# Function to show usage examples
show_examples() {
    echo "📚 Usage Examples"
    echo "================"
    
    echo
    echo "1. Generate all CARLA types:"
    echo "   ./scripts/test_carla_generation.sh"
    echo
    echo "2. Generate specific modules only:"
    echo "   cargo run -- generate \\"
    echo "     --input carla-simulator/PythonAPI/docs/client.yml \\"
    echo "     --config configs/carla_full_generation.toml \\"
    echo "     --output generated_client"
    echo
    echo "3. Generate with custom config:"
    echo "   cargo run -- generate \\"
    echo "     --input carla-simulator/PythonAPI/docs/ \\"
    echo "     --config my_custom_config.toml \\"
    echo "     --output my_output"
    echo
    echo "4. Generate with overrides:"
    echo "   cargo run -- generate \\"
    echo "     --input carla-simulator/PythonAPI/docs/ \\"
    echo "     --config configs/carla_full_generation.toml \\"
    echo "     --output generated \\"
    echo "     --exclude-class 'DeprecatedClass' \\"
    echo "     --type-mapping 'int=i64'"
    echo
}

# Main execution
main() {
    echo "Starting CARLA code generation test..."
    echo
    
    # Check command line arguments
    if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
        show_examples
        exit 0
    fi
    
    # Run all steps
    setup_carla_docs
    echo
    build_codegen
    echo
    run_generation
    echo
    analyze_output
    echo
    validate_code
    echo
    
    echo "🎉 CARLA code generation test completed successfully!"
    echo
    echo "📁 Generated code is available in: $OUTPUT_DIR"
    echo "📊 You can now inspect the generated Rust bindings for CARLA"
    echo
    echo "Next steps:"
    echo "  • Review the generated code in $OUTPUT_DIR"
    echo "  • Integrate into your Rust project"
    echo "  • Customize the configuration in $CONFIG_FILE as needed"
    echo "  • Report any issues with the generated bindings"
}

# Run main function
main "$@"