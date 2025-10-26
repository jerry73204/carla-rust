#!/bin/bash
set -e

usage() {
    cat << EOF
Usage: $0 <CARLA_PATH> [VERSION]

Build prebuilt libcarla_client libraries for distribution.

Arguments:
  CARLA_PATH    Path to CARLA source directory (required)
  VERSION       CARLA version: 0.9.14, 0.9.15, or 0.9.16 (default: 0.9.16)

Examples:
  $0 /path/to/carla
  $0 /path/to/carla 0.9.14
  $0 /path/to/carla 0.9.15
  $0 /path/to/carla 0.9.16

Output:
  carla-sys/generated/libcarla_client.{VERSION}-{TARGET}.tar.zstd
  carla-sys/generated/bindings.{VERSION}.rs
EOF
    exit 1
}

# Parse arguments
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    usage
fi

if [ -z "$1" ]; then
    echo "Error: No CARLA path provided"
    echo ""
    usage
fi

CARLA_DIR="$1"
CARLA_VERSION="${2:-0.9.16}"

# Validate version
if [ "$CARLA_VERSION" != "0.9.14" ] && [ "$CARLA_VERSION" != "0.9.15" ] && [ "$CARLA_VERSION" != "0.9.16" ]; then
    echo "Error: Unsupported CARLA version: $CARLA_VERSION"
    echo "Supported versions: 0.9.14, 0.9.15, 0.9.16"
    exit 1
fi

# Expand and normalize path
CARLA_DIR="${CARLA_DIR/#\~/$HOME}"
CARLA_DIR="${CARLA_DIR%/}"
export CARLA_DIR
export CARLA_VERSION

# Validate CARLA directory
if [ ! -d "$CARLA_DIR" ]; then
    echo "Error: Directory does not exist: $CARLA_DIR"
    exit 1
fi

if [ ! -f "$CARLA_DIR/CMakeLists.txt" ] && [ ! -d "$CARLA_DIR/LibCarla" ]; then
    echo "Error: Not a valid CARLA repository: $CARLA_DIR"
    exit 1
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CARLA_SYS_DIR="$PROJECT_ROOT/carla-sys"

echo "Building with:"
echo "  CARLA_DIR:     $CARLA_DIR"
echo "  CARLA_VERSION: $CARLA_VERSION"
echo ""

# Get target triple
TARGET=$(rustc -vV | grep host | awk '{print $2}')
echo "Target: $TARGET"
echo ""

# Build
cd "$CARLA_SYS_DIR"
cargo clean
cargo build --release --features build-prebuilt

# Determine output paths
PREBUILD_NAME="libcarla_client.${CARLA_VERSION}-${TARGET}"
OUTPUT_DIR="$CARLA_SYS_DIR/generated"
INSTALL_DIR="$OUTPUT_DIR/$PREBUILD_NAME"
TARBALL="$OUTPUT_DIR/${PREBUILD_NAME}.tar.zstd"
BINDINGS="$OUTPUT_DIR/bindings.${CARLA_VERSION}.rs"

# Verify outputs exist
echo ""
echo "Verifying build outputs..."

if [ ! -d "$INSTALL_DIR" ]; then
    echo "Error: Install directory not found: $INSTALL_DIR"
    exit 1
fi

if [ ! -f "$TARBALL" ]; then
    echo "Error: Tarball not found: $TARBALL"
    exit 1
fi

if [ ! -f "$BINDINGS" ]; then
    echo "Error: Bindings file not found: $BINDINGS"
    exit 1
fi

# Show library files
echo ""
echo "Built libraries:"
find "$INSTALL_DIR/lib" -name "*.a" -exec basename {} \; | sort

# Show artifact sizes
echo ""
echo "Generated artifacts:"
echo "  Install dir:  $(du -sh "$INSTALL_DIR" | awk '{print $1}')"
echo "  Tarball:      $(du -h "$TARBALL" | awk '{print $1}')"
echo "  Bindings:     $(du -h "$BINDINGS" | awk '{print $1}')"

echo ""
echo "Output locations:"
echo "  $INSTALL_DIR/"
echo "  $TARBALL"
echo "  $BINDINGS"
echo ""
echo "Build complete!"
