#!/usr/bin/env bash
# Record B2 dog + Odin image (ROS_DOMAIN_ID=0) and ARX R5 arm (ROS_DOMAIN_ID=17)
# for a fixed duration. Two domains cannot share one ros2 bag process, so this
# starts two parallel recorders and stops them together.
#
# Topic list sources:
#   - external/JG/2026-08-06-thor-b2-ros-现场排查.md
#   - external/JG/2026-08-06-odin-plan-track-b2-链路.md
#   - xgc2_b2_link/contract/zenoh_v1.yaml (domain 0 dog + domain 17 arm)
#   - Thor R5 remote_slave.yaml (arm_*_status names)
#   - Live Thor 2026-08-08: domain 17 publishes /arm_master_l_status
#
# Usage (on Thor or any host that sees both domains):
#   ./record_b2_arm_image_30s.sh
#   DURATION_SEC=30 OUT_DIR=~/bags/b2_capture ./record_b2_arm_image_30s.sh
#   DOMAIN0_ONLY=1 ./record_b2_arm_image_30s.sh
#   DOMAIN17_ONLY=1 ./record_b2_arm_image_30s.sh
#
# Optional env:
#   ROS_SETUP          extra workspaces to source (space-separated setup.bash paths)
#   DOMAIN0_RMW / DOMAIN17_RMW   per-domain RMW (defaults below)
#   DOMAIN0 / DOMAIN17           domain ids (defaults 0 / 17)
#   Thor note: arm domain 17 is Fast-RTPS in field; cyclone sees the name but
#   records 0 messages. Dog/Odin domain 0 often uses cyclone.
#   INCLUDE_RAW_IMAGE  1 to also record /odin1/image (large); default compressed only
#   INCLUDE_DEPTH      1 to record depth image topics
#   INCLUDE_CLOUD      1 to record odin point clouds (large)
#   INCLUDE_PLAN       1 to record /iplanner/* path topics
set -euo pipefail

DURATION_SEC="${DURATION_SEC:-30}"
OUT_ROOT="${OUT_DIR:-${HOME}/bags/b2_dual_domain}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUT_ROOT}/${STAMP}"
DOMAIN0="${DOMAIN0:-0}"
DOMAIN17="${DOMAIN17:-17}"
# Per-domain RMW (do NOT inherit a single global RMW for both domains).
# Field truth: domain17 arm only records with Fast-RTPS; cyclone often lists
# the topic but bags 0 messages. Global RMW_IMPLEMENTATION is ignored unless
# the operator explicitly sets DOMAIN0_RMW / DOMAIN17_RMW.
DOMAIN0_RMW="${DOMAIN0_RMW:-rmw_cyclonedds_cpp}"
DOMAIN17_RMW="${DOMAIN17_RMW:-rmw_fastrtps_cpp}"

DOMAIN0_ONLY="${DOMAIN0_ONLY:-0}"
DOMAIN17_ONLY="${DOMAIN17_ONLY:-0}"
INCLUDE_RAW_IMAGE="${INCLUDE_RAW_IMAGE:-0}"
INCLUDE_DEPTH="${INCLUDE_DEPTH:-0}"
INCLUDE_CLOUD="${INCLUDE_CLOUD:-0}"
INCLUDE_PLAN="${INCLUDE_PLAN:-0}"

log() { printf '[record-b2] %s\n' "$*" >&2; }
die() { printf '[record-b2] ERROR: %s\n' "$*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "missing command: $1"; }

source_ros() {
  # ament setup scripts touch unset vars; temporarily allow that.
  set +u
  # Prefer Jazzy (Thor field), then Humble.
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    set -u
    die "no /opt/ros/{jazzy,humble}/setup.bash"
  fi

  local candidate
  for candidate in \
    "${HOME}/b2arx_thor_ros/install/setup.bash" \
    "${HOME}/odin_ros2_jazzy/install/setup.bash" \
    "${HOME}/iplanner/ip_ws/install/setup.bash" \
    "${HOME}/iplanner/vip_ws/install/setup.bash" \
    "${HOME}/ros_ws/install/setup.bash" \
    "${HOME}/vision_ws/install/setup.bash" \
    "${HOME}/R5/ROS2/R5_ws/install/setup.bash" \
    "${HOME}/workspaces/R5/ROS2/R5_ws/install/setup.bash" \
    "${HOME}/R5/ROS2/R5_ws/install/arx5_arm_msg/share/arx5_arm_msg/local_setup.bash" \
    "${HOME}/workspaces/R5/ROS2/R5_ws/install/arx5_arm_msg/share/arx5_arm_msg/local_setup.bash"
  do
    if [[ -f "${candidate}" ]]; then
      # shellcheck disable=SC1090
      source "${candidate}" || true
      log "sourced ${candidate}"
    fi
  done
  # Optional extra workspaces from ROS_SETUP (space-separated).
  if [[ -n "${ROS_SETUP:-}" ]]; then
    local extra
    for extra in ${ROS_SETUP}; do
      if [[ -f "${extra}" ]]; then
        # shellcheck disable=SC1090
        source "${extra}" || true
        log "sourced ${extra}"
      fi
    done
  fi
  set -u
}

# Domain 0: dog B2 + Odin vision (not R5 arm).
domain0_topics() {
  local topics=(
    # B2 productized state (b2_ros2_driver)
    /b2/odom
    /b2/joint_states
    /b2/imu
    /b2/low_state
    /b2/driver_status
    # Odin localization / images (prefer compressed for bag size)
    /odin1/odometry
    /odin1/odometry_highfreq
    /odin1/image/compressed
    /tf
    /tf_static
  )
  if [[ "${INCLUDE_RAW_IMAGE}" == "1" ]]; then
    topics+=(/odin1/image /odin1/image/undistorted /odin1/overlay_image)
  fi
  if [[ "${INCLUDE_DEPTH}" == "1" ]]; then
    topics+=(
      /odin1/depth_img_competetion
      /odin1/depth_img_competetion/compressed
    )
  fi
  if [[ "${INCLUDE_CLOUD}" == "1" ]]; then
    topics+=(/odin1/cloud_raw)
  fi
  if [[ "${INCLUDE_PLAN}" == "1" ]]; then
    topics+=(/iplanner/path /iplanner/status /iplanner/inference_time_ms /mp_waypoint)
  fi
  printf '%s\n' "${topics[@]}"
}

# Domain 17: ARX R5 arm (separate DDS domain from dog/odin).
domain17_topics() {
  local topics=(
    # Live on Thor 2026-08-08 (master status)
    /arm_master_l_status
    # Slave / dual-arm names from remote_slave.yaml + zenoh contract
    /arm_slave_l_status
    /arm_slave_r_status
    /arm_master_r_status
    # Alternate names without leading slash appear in some configs; ros2 bag
    # accepts absolute form — also try non-slash variants if remapped.
    arm_master_l_status
    arm_slave_l_status
    arm_slave_r_status
    arm_master_r_status
  )
  printf '%s\n' "${topics[@]}"
}

filter_existing_topics() {
  # stdin: candidate topics; stdout: topics present in current domain graph.
  # Missing topics are skipped so bag record does not fail when a node is down.
  local available
  available="$(ros2 topic list 2>/dev/null || true)"
  local t
  while IFS= read -r t; do
    [[ -z "${t}" ]] && continue
    if printf '%s\n' "${available}" | grep -qx -- "${t}"; then
      printf '%s\n' "${t}"
    else
      log "skip missing topic: ${t}"
    fi
  done
}

start_record() {
  # Prints only the background PID on stdout (empty if skipped).
  # Args: domain bag_dir rmw topic...
  local domain="$1"
  local bag_dir="$2"
  local rmw="$3"
  shift 3
  local -a topics=("$@")
  if [[ ${#topics[@]} -eq 0 ]]; then
    log "domain ${domain}: no live topics matched; skip recorder"
    return 0
  fi
  mkdir -p "${bag_dir}"
  log "domain ${domain}: rmw=${rmw} recording ${#topics[@]} topics → ${bag_dir}"
  log "  topics: ${topics[*]}"
  # Resolve QoS override once (BEST_EFFORT arm / sensor topics).
  local qos_path=""
  if [[ -n "${QOS_OVERRIDE_PATH:-}" && -f "${QOS_OVERRIDE_PATH}" ]]; then
    qos_path="${QOS_OVERRIDE_PATH}"
  elif [[ -f "${OUT_DIR}/qos_override.yaml" ]]; then
    qos_path="${OUT_DIR}/qos_override.yaml"
  elif [[ -f "${HOME}/bin/b2_record_qos_override.yaml" ]]; then
    qos_path="${HOME}/bin/b2_record_qos_override.yaml"
  fi

  # Must redirect recorder stdout/stderr: callers capture PID via $(start_record)
  # and an open pipe from ros2 bag would block command substitution forever.
  (
    export ROS_DOMAIN_ID="${domain}"
    export RMW_IMPLEMENTATION="${rmw}"
    unset ROS_LOCALHOST_ONLY || true
    if [[ -n "${qos_path}" ]]; then
      exec ros2 bag record -o "${bag_dir}/bag" \
        --qos-profile-overrides-path "${qos_path}" \
        --topics "${topics[@]}" \
        >"${bag_dir}/record.log" 2>&1
    else
      exec ros2 bag record -o "${bag_dir}/bag" --topics "${topics[@]}" \
        >"${bag_dir}/record.log" 2>&1
    fi
  ) &
  printf '%s\n' "$!"
}

main() {
  # Source ROS first — bare login shells on Thor often lack ros2 on PATH.
  source_ros
  need_cmd ros2
  mkdir -p "${OUT_DIR}"
  # Ship QoS override next to this capture if the operator has one in ~/bin.
  if [[ -f "${HOME}/bin/b2_record_qos_override.yaml" && ! -f "${OUT_DIR}/qos_override.yaml" ]]; then
    cp -f "${HOME}/bin/b2_record_qos_override.yaml" "${OUT_DIR}/qos_override.yaml" || true
  fi
  log "output directory: ${OUT_DIR}"
  log "duration: ${DURATION_SEC}s  domain0_rmw=${DOMAIN0_RMW} domain17_rmw=${DOMAIN17_RMW}"

  local -a pids=()
  local pid

  if [[ "${DOMAIN17_ONLY}" != "1" ]]; then
    log "--- discover domain ${DOMAIN0} (rmw=${DOMAIN0_RMW}) ---"
    export ROS_DOMAIN_ID="${DOMAIN0}"
    export RMW_IMPLEMENTATION="${DOMAIN0_RMW}"
    mapfile -t d0_topics < <(domain0_topics | filter_existing_topics | sed '/^$/d')
    if ((${#d0_topics[@]} > 0)); then
      pid="$(start_record "${DOMAIN0}" "${OUT_DIR}/domain${DOMAIN0}" "${DOMAIN0_RMW}" "${d0_topics[@]}")"
      [[ -n "${pid}" ]] && pids+=("${pid}")
    else
      log "domain ${DOMAIN0}: no matching live topics"
    fi
  fi

  if [[ "${DOMAIN0_ONLY}" != "1" ]]; then
    log "--- discover domain ${DOMAIN17} (rmw=${DOMAIN17_RMW}) ---"
    export ROS_DOMAIN_ID="${DOMAIN17}"
    export RMW_IMPLEMENTATION="${DOMAIN17_RMW}"
    mapfile -t d17_topics < <(domain17_topics | filter_existing_topics | sed '/^$/d')
    if ((${#d17_topics[@]} > 0)); then
      pid="$(start_record "${DOMAIN17}" "${OUT_DIR}/domain${DOMAIN17}" "${DOMAIN17_RMW}" "${d17_topics[@]}")"
      [[ -n "${pid}" ]] && pids+=("${pid}")
    else
      log "domain ${DOMAIN17}: no matching live topics"
    fi
  fi

  [[ ${#pids[@]} -gt 0 ]] || die "no recorders started (no matching live topics in either domain)"

  # Write a small manifest for later merge/replay notes.
  {
    echo "stamp=${STAMP}"
    echo "duration_sec=${DURATION_SEC}"
    echo "domain0_rmw=${DOMAIN0_RMW}"
    echo "domain17_rmw=${DOMAIN17_RMW}"
    echo "domain0=${DOMAIN0}"
    echo "domain17=${DOMAIN17}"
    echo "host=$(hostname 2>/dev/null || true)"
    echo "started_utc=$(date -u -Iseconds)"
  } >"${OUT_DIR}/manifest.txt"

  log "recording for ${DURATION_SEC}s (pids: ${pids[*]})..."
  sleep "${DURATION_SEC}"

  log "stopping recorders..."
  local p
  for p in "${pids[@]}"; do
    if kill -0 "${p}" 2>/dev/null; then
      # ros2 bag finalizes the bag on SIGINT.
      kill -INT "${p}" 2>/dev/null || true
    fi
  done
  # Grace period to flush bag index.
  local deadline=$((SECONDS + 15))
  for p in "${pids[@]}"; do
    while kill -0 "${p}" 2>/dev/null && ((SECONDS < deadline)); do
      sleep 0.5
    done
    if kill -0 "${p}" 2>/dev/null; then
      log "pid ${p} still alive; sending TERM"
      kill -TERM "${p}" 2>/dev/null || true
    fi
  done
  for p in "${pids[@]}"; do
    wait "${p}" 2>/dev/null || true
  done

  {
    echo "finished_utc=$(date -u -Iseconds)"
    echo "--- domain${DOMAIN0} topics ---"
    domain0_topics 2>/dev/null || true
    echo "--- domain${DOMAIN17} topics ---"
    domain17_topics 2>/dev/null || true
  } >>"${OUT_DIR}/manifest.txt"

  log "done. bags under: ${OUT_DIR}"
  find "${OUT_DIR}" -maxdepth 3 -type d -name 'bag*' -o -name 'metadata.yaml' 2>/dev/null | head -40 || true
  du -sh "${OUT_DIR}" 2>/dev/null || true
}

main "$@"
