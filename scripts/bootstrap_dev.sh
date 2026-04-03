#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run as a regular user, not root. The script will call sudo when needed."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"

echo "[1/7] Installing base system packages"
sudo apt-get update
sudo apt-get install -y \
  software-properties-common \
  curl \
  gnupg2 \
  lsb-release \
  ca-certificates \
  python3-pip \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool \
  clang-format

if ! dpkg -s ros-jazzy-desktop >/dev/null 2>&1; then
  echo "[2/7] Adding ROS 2 apt repository"
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  echo "[3/7] Installing ROS 2 Jazzy"
  sudo apt-get update
  sudo apt-get install -y ros-jazzy-desktop ros-dev-tools
else
  echo "[2/7] ROS 2 Jazzy already installed"
fi

echo "[4/7] Ensuring rosdep is initialized"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "[5/7] Installing pre-commit"
python3 -m pip install --user --break-system-packages pre-commit

if ! grep -q 'source /opt/ros/jazzy/setup.bash' "${HOME}/.bashrc"; then
  echo "source /opt/ros/jazzy/setup.bash" >> "${HOME}/.bashrc"
fi
if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' "${HOME}/.bashrc"; then
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "${HOME}/.bashrc"
fi

echo "[6/7] Installing repo hooks and ROS dependencies"
export PATH="${HOME}/.local/bin:${PATH}"
PRE_COMMIT_HOME="${REPO_ROOT}/.pre-commit-cache" pre-commit install -f --install-hooks

source /opt/ros/jazzy/setup.bash
cd "${WORKSPACE_ROOT}"
rosdep install --from-paths src --ignore-src -r -y

echo "[7/7] Building workspace"
colcon build --symlink-install --merge-install

echo
echo "Bootstrap complete."
echo "Open a new shell, then run:"
echo "  cd ${WORKSPACE_ROOT}"
echo "  source install/setup.bash"
echo "  ros2 pkg list | head"
