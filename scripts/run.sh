#!/usr/bin/env bash
# Run the simulator and the multi-track stack in containers.
#
#     ./scripts/run.sh sim [--headless [BRIDGE_IP]]   the organizers' simulator
#     ./scripts/run.sh race [NAME] [name:=value ...]  the multi-track race config (M15)
#     ./scripts/run.sh racer [name:=value ...]        race.launch.py with those arguments
#     ./scripts/run.sh shell                          bash in a racer container, nothing started
#     ./scripts/run.sh exec                           second bash in the running racer container
#
# Example:
#     ./scripts/run.sh race m16
#     ./scripts/run.sh racer track:=iros2026 tcp_nodelay:=true loop_hz_cap:=45 log_csv:=run_iros_21.csv
#
# Output: relative log_csv:= paths are written to runs_docker/ in this repo, and
# so is `race`'s NAME.csv. logs/ is mounted too, for run_mt.sh run by hand in
# the container, which writes there.
# Racelines: the host's raceline/ is mounted over the baked copy, so a new CSV
# is raced without a rebuild (MOUNT_RACELINE=0 uses the copy in the image).
# RViz opens on the host display when DISPLAY is set (xhost local:root is run
# for you); pass rviz:=false to skip it.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-autodrive_racer}"
TAG="${TAG:-multi-track}"
SIM_TAG="${SIM_TAG:-2026-iros-compete}"
NAME=autodrive_roboracer_api
IN=/root/Documents/roboracer

racer_run() {
  mkdir -p "$REPO/runs_docker" "$REPO/logs"
  xhost local:root >/dev/null 2>&1 || true
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  # runs/ takes relative log_csv:=; logs/ is where run_mt.sh and `race` write.
  local mounts=(-v "$REPO/runs_docker:$IN/runs" -v "$REPO/logs:$IN/logs")
  if [ "${MOUNT_RACELINE:-1}" = "1" ]; then
    mounts+=(-v "$REPO/raceline:$IN/raceline")
  fi
  # RViz needs the host GPU: without it Mesa's GLX cannot create a render
  # window on an XWayland display ("Invalid parentWindowHandle", rviz2 aborts).
  # GPUS=0 skips it on a machine without the NVIDIA container toolkit.
  local gpu=()
  if [ "${GPUS:-1}" = "1" ] && command -v nvidia-smi >/dev/null 2>&1; then
    gpu=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
  fi
  exec docker run --name "$NAME" --rm -it \
    --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
    "${gpu[@]}" "${mounts[@]}" \
    "${IMAGE}:${TAG}" "$@"
}

case "${1:-}" in
  sim)
    SIM_NAME=autodrive_roboracer_sim
    SIM_IMAGE="autodriveecosystem/autodrive_roboracer_sim:${SIM_TAG}"
    xhost local:root >/dev/null 2>&1 || true
    docker rm -f "$SIM_NAME" >/dev/null 2>&1 || true
    if [ "${2:-}" = "--headless" ]; then
      BRIDGE_IP="${3:-${BRIDGE_IP:-127.0.0.1}}"
      echo "[run.sh] headless sim -> bridge at ${BRIDGE_IP}:4567" >&2
      exec docker run --name "$SIM_NAME" --rm -it --entrypoint /bin/bash \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
        "$SIM_IMAGE" \
        -lc "./AutoDRIVE\\ Simulator.x86_64 -batchmode -nographics -ip ${BRIDGE_IP} -port 4567"
    fi
    exec docker run --name "$SIM_NAME" --rm -it --entrypoint /bin/bash \
      --network=host --ipc=host \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
      "$SIM_IMAGE"
    ;;
  race)
    # The multi-track race config, verbatim from experiments/iros2026/TEAM_IROS2026.md
    # (M15: 56 clean timed laps, first-30 mean 8.963 s). Kept here rather than
    # left to the registry because the registry default is still the older
    # rl_mt_b0.15 line at v_max 8.5, which M09 measured at 9.10-9.35.
    #
    # Differences from run_mt.sh, which raced against a persistent bridge in the
    # dev container: the bridge runs in this container, so bridge:=true and the
    # 45 Hz cap comes from tcp_nodelay + loop_hz_cap instead of bridge.sh.
    shift
    RUN_NAME="${1:-race}"
    case "$RUN_NAME" in *:=*) RUN_NAME=race ;; *) shift || true ;; esac
    racer_run \
      track:=iros2026 rviz:="${RVIZ:-false}" \
      tcp_nodelay:=true loop_hz_cap:="${HZ_CAP:-45}" control_hz:=45 \
      path_csv:="$IN/raceline/iros2026/rl_mt_b05b15w05_ell_L65_B50_v9.0.csv" \
      log_csv:="$IN/runs/${RUN_NAME}.csv" \
      distance_source:=slip v_max:=9.0 enc_rate_window_s:="${ENC_WIN:-0.10}" \
      warmup_v_max:=2.0 warmup_dist_m:=21.0 \
      amcl_params_file:="$IN/experiments/iros2026/params/amcl_beams360.yaml" \
      exit_guard_from:=0.0 exit_guard_full:=0.0 \
      controller_mode:=hybrid_lqr lqr_k_lat:=0.03 lqr_k_head:=0.05 lqr_k_yaw:=0.0 \
      lqr_max_correction_rad:=0.02 \
      recover_settle_s:=2.0 recover_creep_s:=3.0 \
      "$@"
    ;;
  racer)
    shift
    [ $# -gt 0 ] || { echo "usage: $0 racer name:=value ..." >&2; exit 1; }
    racer_run "$@"
    ;;
  shell)
    racer_run bash
    ;;
  exec)
    exec docker exec -it "$NAME" bash
    ;;
  *)
    echo "usage: $0 {sim [--headless [BRIDGE_IP]] | race [NAME] [name:=value ...] | racer name:=value ... | shell | exec}" >&2
    exit 1
    ;;
esac
