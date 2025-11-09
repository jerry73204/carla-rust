#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "List all cargo examples in the carla package."
    echo ""
    echo "OPTIONS:"
    echo "  -p, --paths    Show full file paths"
    echo "  -h, --help     Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  $0                    # List example names"
    echo "  $0 --paths            # List with full paths"
    echo "  $0 | grep test_       # Find examples starting with 'test_'"
    echo "  $0 | wc -l            # Count examples"
    exit 0
}

SHOW_PATHS=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            ;;
        -p|--paths)
            SHOW_PATHS=true
            shift
            ;;
        *)
            echo "Error: Unknown option: $1" >&2
            usage
            ;;
    esac
done

cd "$REPO_ROOT"

if [[ "$SHOW_PATHS" == true ]]; then
    # Show name and path
    cargo metadata --format-version 1 --no-deps 2>/dev/null | \
        jq -r '.packages[] | select(.name == "carla") | .targets[] |
        select(.kind[] == "example") |
        "\(.name)\t\(.src_path)"'
else
    # Show just names
    cargo metadata --format-version 1 --no-deps 2>/dev/null | \
        jq -r '.packages[] | select(.name == "carla") | .targets[] |
        select(.kind[] == "example") | .name'
fi
