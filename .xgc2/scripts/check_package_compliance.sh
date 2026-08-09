#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

test -f package.xml
test -f setup.py
test -f .xgc2/product.yml
test -f .github/workflows/ci.yml
test -f .github/workflows/release.yml
test -f .xgc2/scripts/xgc2_artifact_manifest.py
grep -q 'ubuntu-24.04-arm' .github/workflows/ci.yml
grep -q -- '--product xgc2-b2-link' .github/workflows/release.yml
if rg -n 'b2arx-description|xgc2-ros-jazzy-b2arx' .github/workflows; then
  echo "B2 link workflow still contains copied description-product metadata" >&2
  exit 1
fi
test -x .xgc2/scripts/package_debs.sh
test -x .xgc2/scripts/build_debs_in_docker.sh
test -x .xgc2/scripts/check_installed_package.sh
test -x .xgc2/scripts/check_public_apt_install.sh
test -f contract/zenoh_v1.yaml
test -f xgc2_b2_link/forwarder_node.py
test -f xgc2_b2_link/arm_forwarder_node.py
test -f xgc2_b2_link/runtime_config.py
test -f xgc2_b2_link/ground_peer.py
test -f xgc2_b2_link/sim_publisher.py
test -f xgc2_b2_link/transport.py
test -f xgc2_b2_link/codec.py

# Shared contract must stay versioned
grep -q 'version: 1' contract/zenoh_v1.yaml
grep -q 'up/odom' contract/zenoh_v1.yaml
grep -q 'up/joint_states' contract/zenoh_v1.yaml
grep -q 'up/power_summary' contract/zenoh_v1.yaml
grep -q 'down/cmd' contract/zenoh_v1.yaml

# The command key is reserved, disabled, and never consumed by an executable.
grep -A5 '^[[:space:]]*cmd:' contract/zenoh_v1.yaml | grep -q 'authorized: false'
if rg -n 'cmd_vel|low_cmd|default_allow_types' contract config xgc2_b2_link; then
  echo "B2 link must not contain a motion command path" >&2
  exit 1
fi
if rg -n 'choices=\["auto"|kind == "auto"|fallback' xgc2_b2_link; then
  echo "B2 link must not select a transport fallback" >&2
  exit 1
fi
if rg -n 'publish_ros|Ros1RecoveredPubs' xgc2_b2_link/ground_peer.py \
  || grep -Fq 'b2_ground_peer =' setup.py; then
  echo "contract peer must not expose a packaged ROS recovery path" >&2
  exit 1
fi
test ! -e xgc2_b2_link/ground_ros1.py
test ! -e scripts/one_shot_viz.sh
test ! -e scripts/start_verified_viz.sh
test ! -e scripts/viz_closed_loop.sh
test ! -d layouts
if grep -q '<exec_depend>b2_ros2_driver</exec_depend>' package.xml \
  || grep -q '^DEPENDS=.*b2-ros2-driver' .xgc2/scripts/package_debs.sh; then
  echo "FIELD-only driver messages must not be a hard LAB package dependency" >&2
  exit 1
fi

python3 -m py_compile xgc2_b2_link/codec.py
python3 -m py_compile xgc2_b2_link/contract.py
python3 -m py_compile xgc2_b2_link/transport.py
python3 -m py_compile xgc2_b2_link/forwarder_node.py
python3 -m py_compile xgc2_b2_link/arm_forwarder_node.py
python3 -m py_compile xgc2_b2_link/runtime_config.py
python3 -m py_compile xgc2_b2_link/ground_peer.py
python3 -m py_compile xgc2_b2_link/sim_publisher.py
python3 -m py_compile xgc2_b2_link/sim_models.py
python3 -m py_compile .xgc2/scripts/xgc2_artifact_manifest.py
bash -n .xgc2/scripts/package_debs.sh \
  .xgc2/scripts/build_debs_in_docker.sh \
  .xgc2/scripts/check_installed_package.sh \
  .xgc2/scripts/check_public_apt_install.sh

PYTHONPATH=. python3 -m pytest test/ -q

echo "compliance OK"
