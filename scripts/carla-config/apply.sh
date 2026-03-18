#!/usr/bin/env bash
# Apply patched CARLA config files to a CARLA installation.
# Usage: apply.sh [CARLA_DIR]
# Default CARLA_DIR: $HOME/Downloads/CARLA_0.9.16
set -euo pipefail

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
CARLA_DIR="${1:-${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16}}"

if [ ! -d "$CARLA_DIR" ]; then
    echo "ERROR: CARLA directory not found: $CARLA_DIR" >&2
    exit 1
fi

apply_file() {
    local rel_path="$1"
    local src="$SCRIPT_DIR/$rel_path"
    local dst="$CARLA_DIR/$rel_path"

    if [ ! -f "$src" ]; then
        echo "WARNING: patched file not found, skipping: $src" >&2
        return
    fi

    # Backup original if not already backed up
    if [ ! -f "${dst}.orig-backup" ]; then
        cp "$dst" "${dst}.orig-backup"
        echo "Backed up: $dst -> ${dst}.orig-backup"
    fi

    cp "$src" "$dst"
    echo "Applied: $rel_path"
}

echo "Applying carla-rust config patches to: $CARLA_DIR"
apply_file "Engine/Config/Linux/LinuxEngine.ini"
apply_file "CarlaUE4/Config/DefaultEngine.ini"
echo "Done."
