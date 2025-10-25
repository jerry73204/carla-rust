#!/bin/bash

echo "Generating code..."
cargo run -- generate --input ../carla-simulator/PythonAPI/docs --output test_temp_compile

echo "Creating test crate..."
mkdir -p test_crate/src
cd test_crate

# Create Cargo.toml
cat > Cargo.toml <<EOF
[package]
name = "test-generated"
version = "0.1.0"
edition = "2021"

[workspace]

[dependencies]
thiserror = "1.0"
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
nalgebra = "0.33"
tokio = { version = "1.0", features = ["full"] }
async-trait = "0.1"
EOF

# Copy generated files
cp -r ../test_temp_compile/* src/

# Create lib.rs
cat > src/lib.rs <<EOF
#![allow(unused_imports)]
#![allow(dead_code)]

pub mod error;
pub mod carla;
pub mod command;
EOF

echo "Compiling..."
cargo check 2>&1 | head -100