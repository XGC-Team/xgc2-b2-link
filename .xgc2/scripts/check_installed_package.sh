#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${XGC2_B2_ROS_DISTRO:-jazzy}"
PACKAGE="ros-${ROS_DISTRO}-xgc2-b2-link"
EXPECTED_VERSION="${EXPECTED_VERSION:-0.1.3}"
PREFIX="/opt/ros/${ROS_DISTRO}"
ROS_PACKAGE="xgc2_b2_link"

installed_version="$(dpkg-query -W -f='${Version}' "$PACKAGE")"
[[ "$installed_version" == "$EXPECTED_VERSION" ]] \
  || { echo "$PACKAGE version $installed_version != $EXPECTED_VERSION" >&2; exit 1; }
for executable in b2_forwarder_d0 b2_forwarder_d17 b2_sim_publisher; do
  test -x "$PREFIX/lib/$ROS_PACKAGE/$executable"
done
test ! -e "$PREFIX/lib/$ROS_PACKAGE/b2_ground_peer"
test ! -e "$PREFIX/lib/python3.12/site-packages/$ROS_PACKAGE/ground_peer.py"
test ! -e "$PREFIX/lib/python3.12/site-packages/$ROS_PACKAGE/ground_ros1.py"
test ! -e "$PREFIX/share/$ROS_PACKAGE/config/ground_peer.yaml"
test -f "$PREFIX/share/$ROS_PACKAGE/contract/zenoh_v1.yaml"
test -f "$PREFIX/share/$ROS_PACKAGE/config/forwarder_d0.yaml"
test -f "$PREFIX/share/$ROS_PACKAGE/config/forwarder_d17.yaml"
grep -A5 '^[[:space:]]*cmd:' "$PREFIX/share/$ROS_PACKAGE/contract/zenoh_v1.yaml" \
  | grep -q 'authorized: false'
if rg -n 'choices=\["auto"|kind == "auto"|publish_ros|Ros1RecoveredPubs' \
  "$PREFIX/lib/python3.12/site-packages/$ROS_PACKAGE"; then
  echo "installed B2 link contains a fallback or ROS recovery peer" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "$PREFIX/setup.bash"
set -u
[[ "$(ros2 pkg prefix "$ROS_PACKAGE")" == "$PREFIX" ]]
python3 - <<'PY'
from pathlib import Path

from xgc2_b2_link.contract import contract_path, load_contract

expected = Path("/opt/ros/jazzy/share/xgc2_b2_link/contract/zenoh_v1.yaml")
assert contract_path() == expected, (contract_path(), expected)
assert load_contract()["version"] == 1
PY
depends="$(dpkg-query -W -f='${Depends}' "$PACKAGE")"
if grep -q 'b2-ros2-driver' <<<"$depends"; then
  echo "FIELD-only b2_ros2_driver must not be a hard package dependency" >&2
  exit 1
fi
echo "Installed B2 link gate passed: $PACKAGE=$installed_version"
