#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-jazzy-xgc2-b2-link"
VERSION="${PACKAGE_VERSION:-$(
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "$REPO_ROOT/.xgc2/product.yml"
)}"
APT_BASE_URL="${XGC2_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}"
DOCKER_IMAGE="${DOCKER_IMAGE:-ros:jazzy-ros-base-noble}"
[[ "$VERSION" == "0.1.2" ]] || { echo "public gate is frozen to 0.1.2" >&2; exit 1; }

container_name="xgc2-b2-link-public-apt-$(date +%s)-$$"
cleanup() { docker rm -f "$container_name" >/dev/null 2>&1 || true; }
trap cleanup EXIT
docker create --name "$container_name" "$DOCKER_IMAGE" sleep infinity >/dev/null
docker start "$container_name" >/dev/null
docker exec "$container_name" mkdir -p /tmp/xgc2-b2-link
docker cp "$REPO_ROOT/.xgc2/scripts/check_installed_package.sh" \
  "$container_name:/tmp/xgc2-b2-link/check_installed_package.sh"
# The following single-quoted value is an inner Bash program; its continuations
# are intentionally interpreted by bash -lc inside the container.
# shellcheck disable=SC1004
docker exec \
  -e APT_BASE_URL="$APT_BASE_URL" -e PACKAGE="$PACKAGE" -e VERSION="$VERSION" \
  "$container_name" bash -lc '
    set -euo pipefail
    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends ca-certificates curl gnupg ripgrep
    install -d -m 0755 /etc/apt/keyrings
    curl -fsSL "${APT_BASE_URL%/}/xgc2-archive-keyring.gpg" \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    printf "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s noble main\n" \
      "${APT_BASE_URL%/}" >/etc/apt/sources.list.d/xgc2.list
    apt-get update
    missing=0
    candidate="$(apt-cache policy "$PACKAGE" | awk "/Candidate:/ {print \$2; exit}")"
    if ! apt-cache madison "$PACKAGE" | awk "{print \$3}" | grep -Fxq "$VERSION"; then
      echo "public Noble APT lacks $PACKAGE=$VERSION; candidate=${candidate:-none}" >&2
      missing=1
    fi
    (( missing == 0 )) || exit 1
    apt-get install -y --no-install-recommends "$PACKAGE=$VERSION"
    EXPECTED_VERSION="$VERSION" /tmp/xgc2-b2-link/check_installed_package.sh
  '

echo "Public Noble APT install gate passed: $PACKAGE=$VERSION"
