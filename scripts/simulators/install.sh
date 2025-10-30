#!/usr/bin/env bash
set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    echo "Usage: $0 <install|uninstall> <VERSION>"
    echo ""
    echo "Commands:"
    echo "  install VERSION    Install CARLA service for specified version"
    echo "  uninstall VERSION  Uninstall CARLA service for specified version"
    echo ""
    echo "Supported versions: 0.9.14, 0.9.15, 0.9.16"
    echo ""
    echo "Examples:"
    echo "  $0 install 0.9.16"
    echo "  $0 uninstall 0.9.16"
    exit 1
}

validate_version() {
    local version=$1
    case "$version" in
        0.9.14|0.9.15|0.9.16)
            return 0
            ;;
        *)
            echo -e "${RED}Error: Invalid version '$version'${NC}"
            echo "Supported versions: 0.9.14, 0.9.15, 0.9.16"
            exit 1
            ;;
    esac
}

install_service() {
    local version=$1
    local service_name="carla-${version}@.service"
    local service_file="${SCRIPT_DIR}/${service_name}"

    echo -e "${GREEN}Installing CARLA ${version} service...${NC}"

    # Check if service template exists
    if [ ! -f "$service_file" ]; then
        echo -e "${RED}Error: Service template not found: ${service_file}${NC}"
        exit 1
    fi

    # Create systemd user directory if it doesn't exist
    mkdir -p "$SYSTEMD_USER_DIR"

    # Copy service file
    echo "Copying service file to ${SYSTEMD_USER_DIR}/${service_name}"
    cp "$service_file" "$SYSTEMD_USER_DIR/"

    # Reload systemd
    echo "Reloading systemd daemon..."
    systemctl --user daemon-reload

    echo -e "${GREEN}✓ CARLA ${version} service installed successfully!${NC}"
    echo ""
    echo "To start the service on port 3000:"
    echo "  systemctl --user start carla-${version}@3000"
    echo ""
    echo "To enable auto-start on login:"
    echo "  systemctl --user enable carla-${version}@3000"
    echo ""
    echo "To check status:"
    echo "  systemctl --user status carla-${version}@3000"
    echo ""
    echo "To view logs:"
    echo "  journalctl --user -u carla-${version}@3000 -f"
}

uninstall_service() {
    local version=$1
    local service_name="carla-${version}@.service"
    local service_path="${SYSTEMD_USER_DIR}/${service_name}"

    echo -e "${YELLOW}Uninstalling CARLA ${version} service...${NC}"

    # Check if service file exists
    if [ ! -f "$service_path" ]; then
        echo -e "${YELLOW}Warning: Service file not found: ${service_path}${NC}"
        echo "Service may already be uninstalled."
        exit 0
    fi

    # Stop all instances of this service
    echo "Stopping all instances of carla-${version}@..."
    systemctl --user stop "carla-${version}@*.service" 2>/dev/null || true

    # Disable all instances
    echo "Disabling service..."
    systemctl --user disable "carla-${version}@*.service" 2>/dev/null || true

    # Remove service file
    echo "Removing service file..."
    rm -f "$service_path"

    # Reload systemd
    echo "Reloading systemd daemon..."
    systemctl --user daemon-reload

    echo -e "${GREEN}✓ CARLA ${version} service uninstalled successfully!${NC}"
}

# Main script
if [ $# -ne 2 ]; then
    usage
fi

COMMAND=$1
VERSION=$2

# Validate version
validate_version "$VERSION"

# Execute command
case "$COMMAND" in
    install)
        install_service "$VERSION"
        ;;
    uninstall)
        uninstall_service "$VERSION"
        ;;
    *)
        echo -e "${RED}Error: Invalid command '$COMMAND'${NC}"
        usage
        ;;
esac
