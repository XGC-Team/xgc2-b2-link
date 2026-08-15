#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${XGC2_B2_ROS_DISTRO:-jazzy}"
UBUNTU_CODENAME="${UBUNTU_CODENAME:-noble}"
DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-noble-ros-jazzy:1.0.0}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker-${ROS_DISTRO}}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
EXPECTED_ARCH="${EXPECTED_ARCH:-}"
PACKAGE_VERSION="${PACKAGE_VERSION:-}"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
readonly HOST_UID HOST_GID
[[ "$HOST_UID" =~ ^[0-9]+$ && "$HOST_GID" =~ ^[0-9]+$ ]] \
  || { echo "host uid/gid must be numeric" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "$WORK_DIR" "$OUTPUT_DIR"
# The following single-quoted value is an inner Bash program; its continuations
# are intentionally interpreted by bash -lc inside the container.
# shellcheck disable=SC1004
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e EXPECTED_ARCH="$EXPECTED_ARCH" \
  -e HOST_GID="$HOST_GID" \
  -e HOST_UID="$HOST_UID" \
  -e PACKAGE_VERSION="$PACKAGE_VERSION" \
  -e ROS_DISTRO="$ROS_DISTRO" \
  -v "$REPO_ROOT:/workspace/repo:ro" \
  -v "$WORK_DIR:/workspace/work" \
  -v "$OUTPUT_DIR:/workspace/out" \
  "$DOCKER_IMAGE" bash -lc '
    set -euo pipefail
    export DEBIAN_FRONTEND=noninteractive
    return_mount_ownership() {
      chown -R "${HOST_UID}:${HOST_GID}" /workspace/work /workspace/out
    }
    trap return_mount_ownership EXIT
    actual_arch="$(dpkg --print-architecture)"
    [[ -z "${EXPECTED_ARCH}" || "${actual_arch}" == "${EXPECTED_ARCH}" ]] \
      || { echo "container architecture ${actual_arch} != ${EXPECTED_ARCH}" >&2; exit 1; }
    for pkg in dpkg-dev fakeroot rsync python3-pytest python3-yaml \
      "ros-${ROS_DISTRO}-rclpy" "ros-${ROS_DISTRO}-nav-msgs" \
      "ros-${ROS_DISTRO}-sensor-msgs" "ros-${ROS_DISTRO}-diagnostic-msgs" \
      "ros-${ROS_DISTRO}-std-msgs"
    do
      if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
        echo "image is missing ${pkg}; use xgc2-build-noble-ros-jazzy" >&2
        exit 1
      fi
    done
    find /workspace/work -mindepth 1 -maxdepth 1 \
      \( -name build -o -name install -o -name log -o -name src -o -name install-root \) \
      -exec rm -rf {} +
    mkdir -p /workspace/work/src /workspace/work/install-root/opt/ros/${ROS_DISTRO}
    rsync -a --exclude .git --exclude .work --exclude debs \
      /workspace/repo/ /workspace/work/src/xgc2_b2_link/
    cd /workspace/work
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u
    colcon build --merge-install --packages-select xgc2_b2_link
    PYTHONPATH=/workspace/work/src/xgc2_b2_link \
      python3 -m pytest /workspace/work/src/xgc2_b2_link/test -q
    rsync -a /workspace/work/install/ /workspace/work/install-root/opt/ros/${ROS_DISTRO}/
    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root --output-dir /workspace/out \
      --ros-distro "${ROS_DISTRO}"
    deb="$(find /workspace/out -maxdepth 1 -type f \
      -name "ros-${ROS_DISTRO}-xgc2-b2-link_*.deb" | sort | tail -1)"
    test -n "$deb"
    rm -rf /workspace/work/extracted
    mkdir -p /workspace/work/extracted
    dpkg-deb -x "$deb" /workspace/work/extracted
    for executable in b2_forwarder_d0 b2_forwarder_d17 b2_sim_publisher; do
      test -x "/workspace/work/extracted/opt/ros/${ROS_DISTRO}/lib/xgc2_b2_link/${executable}"
    done
    test ! -e "/workspace/work/extracted/opt/ros/${ROS_DISTRO}/lib/xgc2_b2_link/b2_ground_peer"
    test ! -e "/workspace/work/extracted/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages/xgc2_b2_link/ground_peer.py"
    test ! -e "/workspace/work/extracted/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages/xgc2_b2_link/ground_ros1.py"
    test ! -e "/workspace/work/extracted/opt/ros/${ROS_DISTRO}/share/xgc2_b2_link/config/ground_peer.yaml"
    dpkg-deb -f "$deb" Package Version Architecture Depends Recommends

    package="ros-${ROS_DISTRO}-xgc2-b2-link"
    expected_version="$(dpkg-deb -f "$deb" Version)"
    # LAB uses only standard ROS messages. FIELD driver/arm message packages
    # are optional source capabilities and must never be faked by install gates.
    apt-get install -y --no-install-recommends "$deb"
    EXPECTED_VERSION="$expected_version" \
      /workspace/repo/.xgc2/scripts/check_installed_package.sh

    for workspace_file in \
      setup.bash setup.ps1 setup.sh setup.zsh \
      local_setup.bash local_setup.ps1 local_setup.sh local_setup.zsh \
      _local_setup_util_ps1.py _local_setup_util_sh.py \
      .colcon_install_layout COLCON_IGNORE; do
      if dpkg-query -L "$package" | grep -Fqx "/opt/ros/${ROS_DISTRO}/${workspace_file}"; then
        echo "$package must not own ROS workspace file ${workspace_file}" >&2
        exit 1
      fi
    done
    for workspace_file in setup.bash local_setup.bash; do
      dpkg-query -S "/opt/ros/${ROS_DISTRO}/${workspace_file}" \
        | grep -q "^ros-${ROS_DISTRO}-ros-workspace:"
    done
  '

find "$OUTPUT_DIR" -maxdepth 1 -type f -name "ros-${ROS_DISTRO}-xgc2-b2-link_*.deb" -print | sort
