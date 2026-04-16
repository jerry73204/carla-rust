#!/usr/bin/env bash
# Restore original CARLA config files (undo apply.sh).
# Usage: restore.sh [CARLA_DIR]
set -euo pipefail

CARLA_DIR="${1:-${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16}}"

if [ ! -d "$CARLA_DIR" ]; then
    echo "ERROR: CARLA directory not found: $CARLA_DIR" >&2
    exit 1
fi

restore_file() {
    local rel_path="$1"
    local dst="$CARLA_DIR/$rel_path"
    local backup="${dst}.orig-backup"

    if [ -f "$backup" ]; then
        cp "$backup" "$dst"
        rm "$backup"
        echo "Restored: $rel_path"
    else
        echo "No backup found, skipping: $rel_path"
    fi
}

echo "Restoring original CARLA config in: $CARLA_DIR"
restore_file "Engine/Config/Linux/LinuxEngine.ini"
if [ -d "$CARLA_DIR/CarlaUnreal" ]; then
    restore_file "CarlaUnreal/Config/DefaultEngine.ini"
else
    restore_file "CarlaUE4/Config/DefaultEngine.ini"
fi
echo "Done."
