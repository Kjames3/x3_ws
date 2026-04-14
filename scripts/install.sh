#!/usr/bin/env bash
# =============================================================================
# install.sh
# Yahboom X3 robot — project setup script
#
# Assumes ROS2 Humble is already installed on Ubuntu 22.04.
# Installs all remaining dependencies, builds the workspace, and configures
# hardware access (udev rules, user groups).
#
# Usage (run as a normal user, NOT root):
#   cd ~/x3_ws
#   bash scripts/install.sh
#
# Optional flags:
#   --sim    Also install ros-humble-ros-gz (Gazebo/Ignition simulation)
# =============================================================================

set -e

BOLD='\033[1m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

ok()   { echo -e "  ${GREEN}✓${NC}  $1"; }
warn() { echo -e "  ${YELLOW}!${NC}  $1"; }
info() { echo -e "  ${CYAN}→${NC}  $1"; }
die()  { echo -e "  ${RED}✗${NC}  $1"; exit 1; }

# ── Parse flags ───────────────────────────────────────────────────────────────
INSTALL_SIM=false
for arg in "$@"; do
    case "$arg" in
        --sim) INSTALL_SIM=true ;;
        *) warn "Unknown flag: $arg" ;;
    esac
done

# ── Workspace root (auto-detected from script location) ───────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
ROS_SETUP="/opt/ros/humble/setup.bash"

echo "============================================================"
echo -e "  ${BOLD}Yahboom X3 — Project Setup${NC}"
echo "============================================================"
echo "  Workspace : $WS_DIR"
echo "  User      : $USER"
echo "  Sim mode  : $INSTALL_SIM"
echo ""

# ── Safety checks ─────────────────────────────────────────────────────────────
if [ "$EUID" -eq 0 ]; then
    die "Do not run as root. Run as a normal user: bash scripts/install.sh"
fi

if [ ! -f "$ROS_SETUP" ]; then
    die "ROS2 Humble not found at $ROS_SETUP — install it first, then re-run this script."
fi

UBUNTU_VERSION=$(. /etc/os-release && echo "$VERSION_ID")
if [ "$UBUNTU_VERSION" != "22.04" ]; then
    warn "Ubuntu 22.04 expected for ROS2 Humble — found $UBUNTU_VERSION. Proceeding anyway."
fi

echo ""

# =============================================================================
# [1/7] APT — system libraries
# =============================================================================
echo -e "${BOLD}[1/7] Installing system libraries...${NC}"
sudo apt-get update -q
sudo apt-get install -y \
    python3-serial \
    python3-opencv \
    i2c-tools \
    libi2c-dev \
    libusb-1.0-0-dev \
    libopenni2-dev \
    openni2-utils \
    libopenblas-dev \
    python3-colcon-common-extensions \
    python3-rosdep
ok "System libraries installed"
echo ""

# =============================================================================
# [2/7] APT — ROS2 Humble packages
# =============================================================================
echo -e "${BOLD}[2/7] Installing ROS2 Humble packages...${NC}"
sudo apt-get install -y \
    ros-humble-imu-filter-madgwick \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-msgs \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard

if [ "$INSTALL_SIM" = true ]; then
    info "Installing Gazebo/Ignition simulation packages..."
    sudo apt-get install -y ros-humble-ros-gz
    ok "ros-humble-ros-gz installed"
fi

ok "ROS2 packages installed"
echo ""

# Initialise rosdep (idempotent)
if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "Initialising rosdep..."
    sudo rosdep init
fi
rosdep update --rosdistro humble
echo ""

# =============================================================================
# [3/7] pip — Python dependencies
# =============================================================================
echo -e "${BOLD}[3/7] Installing Python dependencies...${NC}"

# ROS2 build tools (required for colcon to process Python packages)
pip3 install --quiet catkin_pkg "empy==3.3.4" lark
ok "ROS2 build tools: catkin_pkg, empy, lark"

# Runtime dependencies
pip3 install --quiet \
    numpy \
    websockets \
    pyyaml \
    pyserial \
    smbus2 \
    scipy \
    pillow \
    "luma.oled" \
    "luma.core"
ok "Runtime libs: numpy, websockets, pyyaml, pyserial, smbus2, scipy, pillow, luma.oled"

# Ultralytics YOLO (pulls torch/torchvision from PyPI if not present)
echo ""
warn "Installing ultralytics — this pulls PyTorch (~500 MB on first install)."
warn "On Jetson, pre-install NVIDIA's CUDA-enabled torch wheel first for GPU acceleration."
warn "See: https://docs.ultralytics.com/guides/nvidia-jetson/"
pip3 install --quiet ultralytics
ok "ultralytics (YOLO) installed"

echo ""

# =============================================================================
# [4/7] YDLidar C++ SDK  (prerequisite for ydlidar_ros2_driver)
# =============================================================================
echo -e "${BOLD}[4/7] Building YDLidar C++ SDK...${NC}"

if ldconfig -p | grep -q libydlidar_sdk 2>/dev/null; then
    ok "YDLidar SDK already installed — skipping build"
else
    SDK_DIR="$HOME/YDLidar-SDK"
    if [ ! -d "$SDK_DIR" ]; then
        info "Cloning YDLidar-SDK into $SDK_DIR..."
        git clone --depth=1 https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
    else
        info "YDLidar-SDK already cloned — pulling latest..."
        git -C "$SDK_DIR" pull
    fi
    mkdir -p "$SDK_DIR/build"
    cmake -S "$SDK_DIR" -B "$SDK_DIR/build" -DCMAKE_BUILD_TYPE=Release -Wno-dev
    make -C "$SDK_DIR/build" -j"$(nproc)"
    sudo make -C "$SDK_DIR/build" install
    sudo ldconfig
    ok "YDLidar SDK built and installed"
fi
echo ""

# =============================================================================
# [5/7] udev rules
# =============================================================================
echo -e "${BOLD}[5/7] Writing udev rules...${NC}"

write_udev_rule() {
    local file="$1"
    local content="$2"
    if [ ! -f "$file" ]; then
        echo "$content" | sudo tee "$file" > /dev/null
        ok "Written: $file"
    else
        ok "Already exists: $file"
    fi
}

write_udev_rule /etc/udev/rules.d/99-orbbec.rules \
'# Orbbec Astra / Astra Pro SC
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1d27", MODE="0666", GROUP="plugdev"'

write_udev_rule /etc/udev/rules.d/60-ydlidar.rules \
'# YDLidar — stable /dev/ydlidar symlink (CP2102 / PL2303 / CH340)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar",  MODE="0666", GROUP="dialout"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="ydlidar",  MODE="0666", GROUP="dialout"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ydlidar",  MODE="0666", GROUP="dialout"'

write_udev_rule /etc/udev/rules.d/61-rosmaster.rules \
'# Yahboom Rosmaster controller board (CH340 / ttyACM)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="rosmaster", MODE="0666", GROUP="dialout"
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"'

write_udev_rule /etc/udev/rules.d/99-gpio.rules \
'# Jetson GPIO
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", \
    PROGRAM="/bin/sh -c '"'"'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"'"'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", \
    PROGRAM="/bin/sh -c '"'"'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"'"'"'

sudo udevadm control --reload-rules && sudo udevadm trigger 2>/dev/null || true

warn "If YDLidar and Rosmaster share the same USB chip (VID 1a86:7523), both udev"
warn "symlinks will race. Plug each in separately, run 'udevadm info /dev/ttyUSBx',"
warn "and differentiate by KERNELS (physical port path) in the udev rules."
echo ""

# =============================================================================
# [6/7] User groups
# =============================================================================
echo -e "${BOLD}[6/7] Adding $USER to hardware groups...${NC}"
sudo usermod -aG dialout,i2c,video,plugdev "$USER" 2>/dev/null || true
# gpio group may not exist on non-Jetson systems
if getent group gpio > /dev/null 2>&1; then
    sudo usermod -aG gpio "$USER" 2>/dev/null || true
fi
ok "Added to: dialout, i2c, video, plugdev (+ gpio if present)"
echo ""

# =============================================================================
# [7/7] rosdep + colcon build --symlink-install
# =============================================================================
echo -e "${BOLD}[7/7] Building ROS2 workspace...${NC}"

# Source ROS2 for rosdep and colcon
source "$ROS_SETUP"

cd "$WS_DIR"

# Initialise git submodules (e.g. ydlidar_ros2_driver) if not already cloned
if [ -f "$WS_DIR/.gitmodules" ]; then
    info "Initialising git submodules..."
    git -C "$WS_DIR" submodule update --init --recursive
    ok "Submodules up to date"
fi

info "Running rosdep install..."
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
    && ok "rosdep dependencies satisfied" \
    || warn "rosdep had some failures — check output above"

echo ""
info "Running colcon build --symlink-install (this may take 5-10 min)..."
colcon build \
    --symlink-install \
    --packages-select \
        yahboomcar_msgs \
        yahboomcar_description \
        yahboomcar_base_node \
        yahboomcar_bringup \
        yahboomcar_nav \
        ydlidar_ros2_driver \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && ok "Workspace built successfully" \
    || die "colcon build failed — see output above"

echo ""

# =============================================================================
# .bashrc — source workspace + convenience aliases
# =============================================================================
BASHRC="$HOME/.bashrc"
if ! grep -q "x3_ws/install" "$BASHRC" 2>/dev/null; then
    info "Adding ROS2 workspace sourcing to ~/.bashrc..."
    cat >> "$BASHRC" << BASHLINES

# === ROS2 x3_ws (added by install.sh) ===
source /opt/ros/humble/setup.bash
source ${WS_DIR}/install/local_setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=42
alias cb='cd ${WS_DIR} && colcon build --symlink-install'
alias cs='source ${WS_DIR}/install/local_setup.bash'
BASHLINES
    ok "~/.bashrc updated"
else
    ok "~/.bashrc already has workspace sourcing"
fi
echo ""

# =============================================================================
# Verification
# =============================================================================
echo "============================================================"
echo -e "  ${BOLD}Verification${NC}"
echo "============================================================"

[ -f "$ROS_SETUP" ]           && ok "ROS2 Humble at /opt/ros/humble"   || warn "ROS2 Humble not found"
[ -d "$WS_DIR/install" ]      && ok "Workspace built: $WS_DIR/install" || warn "Build artifacts not found"
ldconfig -p | grep -q libydlidar_sdk 2>/dev/null \
                              && ok "YDLidar SDK in ldconfig"           || warn "YDLidar SDK not found — driver build may fail"
[ -e /dev/ydlidar ]           && ok "/dev/ydlidar symlink active"       || info "/dev/ydlidar — plug in YDLidar to activate"
[ -e /dev/rosmaster ]         && ok "/dev/rosmaster symlink active"     || info "/dev/rosmaster — plug in Rosmaster to activate"
lsusb 2>/dev/null | grep -qiE "2bc5|1d27|orbbec|astra" \
                              && ok "Orbbec camera detected"            || info "Orbbec camera not connected"

echo ""
echo "============================================================"
echo -e "  ${BOLD}Setup complete!${NC}"
echo "============================================================"
echo ""
echo -e "${BOLD}Next steps:${NC}"
echo ""
echo "  1. Reload your shell (or open a new terminal):"
echo "       source ~/.bashrc"
echo ""
echo "  2. Reboot to activate udev rules and group memberships:"
echo "       sudo reboot"
echo ""
echo "  3. After reboot — start the robot:"
echo "       bash $WS_DIR/scripts/jetson_bringup.sh"
echo "       python3 $WS_DIR/src/server_x3.py --domain-id 42"
echo ""
echo "  4. Open the GUI in a browser:"
echo "       http://<jetson-ip>:8080/GUI.html"
echo ""
echo "  Workspace: $WS_DIR"
echo "  Build:     cb   (alias: colcon build --symlink-install)"
echo "  Source:    cs   (alias: source install/local_setup.bash)"
echo ""
