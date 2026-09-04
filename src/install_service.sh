#!/bin/bash
# Install and enable the x3_server, orbbec_depth and lidar-home systemd services.
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

# Install stable serial-device names before starting services that consume them.
for rule in 63-rosmaster 64-openrb150; do
    if [ -f "$SCRIPT_DIR/${rule}.rules" ] && \
       ! cmp -s "$SCRIPT_DIR/${rule}.rules" "/etc/udev/rules.d/${rule}.rules"; then
        echo "Installing udev rule ${rule}..."
        cp "$SCRIPT_DIR/${rule}.rules" "/etc/udev/rules.d/${rule}.rules"
        udevadm control --reload && udevadm trigger --subsystem-match=tty
    fi
done

# Discovery server must start first — it has Before= x3_server and orbbec_depth
install_service fastdds_discovery
# Level the tilting lidar mount before starting the server; it needs
# /dev/openrb150 (the sweep loop and this service share the Dynamixel bus).
# The legacy x3_lidar_home unit must remain disabled: it targets the LX-16A.
systemctl disable --now x3_lidar_home.service 2>/dev/null || true
install_service x3_dynamixel_tilt_home
install_service x3_server
install_service orbbec_depth

echo ""
echo "Done. All services are enabled (auto-start on boot)."
echo ""
echo "Useful commands:"
echo "  sudo systemctl status fastdds_discovery  # FastDDS discovery server"
echo "  sudo systemctl status x3_server          # web server + ROS2 bringup"
echo "  sudo systemctl status orbbec_depth       # depth camera publisher"
echo "  sudo systemctl status x3_dynamixel_tilt_home # lidar tilt mount homed to level"
echo "  journalctl -u fastdds_discovery -f       # discovery server logs"
echo "  journalctl -u x3_server -f               # x3 live logs"
echo "  journalctl -u orbbec_depth -f            # depth camera live logs"
