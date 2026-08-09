#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${XGC2_B2_ROS_DISTRO:-jazzy}"
PACKAGE="ros-${ROS_DISTRO}-xgc2-b2-link"
VERSION="${PACKAGE_VERSION:-$(
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --ros-distro) ROS_DISTRO="$2"; PACKAGE="ros-${ROS_DISTRO}-xgc2-b2-link"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ -n "$INSTALL_ROOT" && "$INSTALL_ROOT" == /* && "$INSTALL_ROOT" != / ]] \
  || { echo "--install-root must be an absolute, non-root path" >&2; exit 2; }
[[ -n "$OUTPUT_DIR" && "$OUTPUT_DIR" == /* && "$OUTPUT_DIR" != / ]] \
  || { echo "--output-dir must be an absolute, non-root path" >&2; exit 2; }
[[ -n "$VERSION" ]] || { echo "product version is empty" >&2; exit 2; }

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
PKG_ROOT="$(mktemp -d -t xgc2-b2-link-deb-XXXXXX)"
trap 'rm -rf "${PKG_ROOT}"' EXIT

[[ -d "$PREFIX_ROOT" ]] || { echo "install root missing: $PREFIX_ROOT" >&2; exit 1; }
mkdir -p "$OUTPUT_DIR" "$PKG_ROOT/DEBIAN" "$PKG_ROOT/usr/share/doc/$PACKAGE" "$PKG_ROOT$PREFIX"
find "$OUTPUT_DIR" -maxdepth 1 -type f -name "${PACKAGE}_*.deb" -delete

copy_product_path() {
  local source="$1"
  local relative="${source#"$PREFIX_ROOT"/}"
  [[ "$relative" != "$source" ]] \
    || { echo "refusing to stage a path outside $PREFIX_ROOT: $source" >&2; exit 1; }
  if [[ -d "$source" ]]; then
    mkdir -p "$PKG_ROOT$PREFIX/$relative"
    rsync -a "$source/" "$PKG_ROOT$PREFIX/$relative/"
  else
    mkdir -p "$PKG_ROOT$PREFIX/$(dirname "$relative")"
    rsync -a "$source" "$PKG_ROOT$PREFIX/$relative"
  fi
}

# A merged colcon install also contains the ROS workspace setup files owned by
# ros-${ROS_DISTRO}-ros-workspace. Stage only files owned by this package.
for relative in lib/xgc2_b2_link share/xgc2_b2_link; do
  [[ -d "$PREFIX_ROOT/$relative" ]] \
    || { echo "installed product path is missing: $PREFIX_ROOT/$relative" >&2; exit 1; }
  copy_product_path "$PREFIX_ROOT/$relative"
done

mapfile -t PYTHON_PACKAGE_SOURCES < <(
  find "$PREFIX_ROOT/lib" -type d -path '*/site-packages/xgc2_b2_link' -print
)
(( ${#PYTHON_PACKAGE_SOURCES[@]} == 1 )) \
  || { echo "expected exactly one installed Python package" >&2; exit 1; }
PYTHON_PACKAGE_SOURCE="${PYTHON_PACKAGE_SOURCES[0]}"
SITE_PACKAGES_SOURCE="$(dirname "$PYTHON_PACKAGE_SOURCE")"
copy_product_path "$PYTHON_PACKAGE_SOURCE"

shopt -s nullglob
PYTHON_METADATA_SOURCES=(
  "$SITE_PACKAGES_SOURCE"/xgc2_b2_link-*.egg-info
  "$SITE_PACKAGES_SOURCE"/xgc2_b2_link-*.dist-info
)
shopt -u nullglob
(( ${#PYTHON_METADATA_SOURCES[@]} >= 1 )) \
  || { echo "installed Python package metadata is missing" >&2; exit 1; }
for source in "${PYTHON_METADATA_SOURCES[@]}"; do
  copy_product_path "$source"
done

mapfile -t AMENT_INDEX_SOURCES < <(
  find "$PREFIX_ROOT/share/ament_index/resource_index" -type f \
    -name xgc2_b2_link -print
)
(( ${#AMENT_INDEX_SOURCES[@]} >= 1 )) \
  || { echo "installed ament index entry is missing" >&2; exit 1; }
for source in "${AMENT_INDEX_SOURCES[@]}"; do
  copy_product_path "$source"
done

COLCON_INDEX_SOURCE="$PREFIX_ROOT/share/colcon-core/packages/xgc2_b2_link"
if [[ -f "$COLCON_INDEX_SOURCE" ]]; then
  copy_product_path "$COLCON_INDEX_SOURCE"
fi

# The source tree keeps a ground-side contract probe for codec/loopback tests,
# but the formal onboard Debian must not ship a second ground consumer or ROS
# recovery implementation.
PYTHON_PACKAGE_DIR="$PKG_ROOT$PREFIX/${PYTHON_PACKAGE_SOURCE#"$PREFIX_ROOT"/}"
[[ -n "$PYTHON_PACKAGE_DIR" ]] || { echo "installed Python package is missing" >&2; exit 1; }
rm -f \
  "$PYTHON_PACKAGE_DIR/ground_peer.py" \
  "$PYTHON_PACKAGE_DIR/ground_ros1.py" \
  "$PKG_ROOT$PREFIX/share/xgc2_b2_link/config/ground_peer.yaml"
if [[ -d "$PYTHON_PACKAGE_DIR/__pycache__" ]]; then
  find "$PYTHON_PACKAGE_DIR/__pycache__" -maxdepth 1 -type f \
    \( -name 'ground_peer.*.pyc' -o -name 'ground_ros1.*.pyc' \) -delete
fi

for executable in b2_forwarder_d0 b2_forwarder_d17 b2_sim_publisher; do
  test -x "$PKG_ROOT$PREFIX/lib/xgc2_b2_link/$executable" \
    || { echo "missing installed entrypoint: $executable" >&2; exit 1; }
done
test ! -e "$PKG_ROOT$PREFIX/lib/xgc2_b2_link/b2_ground_peer" \
  || { echo "contract peer must not be an installed console entrypoint" >&2; exit 1; }
test ! -e "$PYTHON_PACKAGE_DIR/ground_peer.py"
test ! -e "$PYTHON_PACKAGE_DIR/ground_ros1.py"
test ! -e "$PKG_ROOT$PREFIX/share/xgc2_b2_link/config/ground_peer.yaml"
test -f "$PKG_ROOT$PREFIX/share/xgc2_b2_link/contract/zenoh_v1.yaml"
test -f "$PKG_ROOT$PREFIX/share/xgc2_b2_link/config/forwarder_d0.yaml"
test -f "$PKG_ROOT$PREFIX/share/xgc2_b2_link/config/forwarder_d17.yaml"
for workspace_file in \
  setup.bash setup.ps1 setup.sh setup.zsh \
  local_setup.bash local_setup.ps1 local_setup.sh local_setup.zsh \
  _local_setup_util_ps1.py _local_setup_util_sh.py \
  .colcon_install_layout COLCON_IGNORE; do
  test ! -e "$PKG_ROOT$PREFIX/$workspace_file" \
    || { echo "Deb must not own merged workspace file: $workspace_file" >&2; exit 1; }
done

DEPENDS="python3-yaml, ros-${ROS_DISTRO}-ament-index-python, ros-${ROS_DISTRO}-rclpy, ros-${ROS_DISTRO}-nav-msgs, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-diagnostic-msgs, ros-${ROS_DISTRO}-std-msgs"
RECOMMENDS="ros-${ROS_DISTRO}-b2-ros2-driver, ros-${ROS_DISTRO}-arx5-arm-msg"
cat >"$PKG_ROOT/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@xgc2.local>
Depends: ${DEPENDS}
Recommends: ${RECOMMENDS}
Description: XGC2 Unitree B2 read-only onboard data forwarders
 Explicit d0 and optional d17 ROS 2 status forwarders for the frozen B2
 Zenoh/TCP wire. This package contains no motion command consumer.
EOF

printf '%s\n' "$PACKAGE" >"$PKG_ROOT/usr/share/doc/$PACKAGE/README"
find "$PKG_ROOT" -type d -exec chmod 0755 {} +
find "$PKG_ROOT" -type f -exec chmod 0644 {} +
find "$PKG_ROOT$PREFIX/lib/xgc2_b2_link" -type f -exec chmod 0755 {} +
chmod 0755 "$PKG_ROOT/DEBIAN"

DEB="$OUTPUT_DIR/${PACKAGE}_${VERSION}_${ARCH}.deb"
fakeroot dpkg-deb --build "$PKG_ROOT" "$DEB" >/dev/null
size="$(stat -c%s "$DEB")"
(( size >= 12000 )) || { echo "deb package is unexpectedly small: $size bytes" >&2; exit 1; }
echo "built $DEB ($size bytes)"
