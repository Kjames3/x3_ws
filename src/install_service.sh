#!/bin/bash
# Install and enable the x3_server and orbbec_depth systemd services.
# Run with: sudo bash install_service.sh

set -e

SCRIPT_DIR="$(dirname "$(realpath "$0")")"

if [ "$EUID" -ne 0 ]; then
    echo "Run as root: sudo bash install_service.sh"
    exit 1
fi

install_service() {
    local name="$1"
    local src="$SCRIPT_DIR/${name}.service"
    local dst="/etc/systemd/system/${name}.service"

    if [ ! -f "$src" ]; then
        echo "SKIP: $src not found"
        return
    fi
    echo "Installing $name..."
    cp "$src" "$dst"
    systemctl enable "$name"
    systemctl restart "$name"
    echo "  ✓ $name enabled and started"
}

echo "Reloading systemd daemon..."
systemctl daemon-reload

# Discovery server must start first — it has Before= x3_server and orbbec_depth
install_service fastdds_discovery
install_service x3_server
install_service orbbec_depth

echo ""
echo "Done. All three services are enabled (auto-start on boot)."
echo ""
echo "Useful commands:"
echo "  sudo systemctl status fastdds_discovery  # FastDDS discovery server"
echo "  sudo systemctl status x3_server          # web server + ROS2 bringup"
echo "  sudo systemctl status orbbec_depth       # depth camera publisher"
echo "  journalctl -u fastdds_discovery -f       # discovery server logs"
echo "  journalctl -u x3_server -f               # x3 live logs"
echo "  journalctl -u orbbec_depth -f            # depth camera live logs"
