#!/bin/bash
# Install and enable the x3_server systemd service.
# Run with: sudo bash install_service.sh

set -e

SERVICE_SRC="$(dirname "$(realpath "$0")")/x3_server.service"
SERVICE_DST="/etc/systemd/system/x3_server.service"

if [ "$EUID" -ne 0 ]; then
    echo "Run as root: sudo bash install_service.sh"
    exit 1
fi

echo "Copying service file to $SERVICE_DST..."
cp "$SERVICE_SRC" "$SERVICE_DST"

echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling x3_server (auto-start on boot)..."
systemctl enable x3_server

echo "Starting x3_server now..."
systemctl start x3_server

echo ""
echo "Done. Useful commands:"
echo "  sudo systemctl status x3_server   # check status"
echo "  journalctl -u x3_server -f        # live logs"
echo "  sudo systemctl stop x3_server     # stop manually"
echo "  sudo systemctl disable x3_server  # remove from autostart"
