#!/usr/bin/env bash
# Run the simulator and the multi-track stack in containers.
#
#     ./scripts/run.sh sim [--headless [BRIDGE_IP]]   the organizers' simulator
#     ./scripts/run.sh race [NAME] [name:=value ...]  the multi-track race config (M15)
#     ./scripts/run.sh truth [NAME] [name:=value ...] DIAGNOSTIC: steer on ground truth
#     ./scripts/run.sh shadow [NAME] [name:=value ...] `race` with localization_v2 running beside AMCL, no TF
#     ./scripts/run.sh racer [name:=value ...]        race.launch.py with those arguments
#     ./scripts/run.sh shell                          bash in a racer container, nothing started
#     ./scripts/run.sh exec                           second bash in the running racer container
#
# Example:
#     ./scripts/run.sh race m16
#     AMCL_PARAMS=amcl_beams720.yaml ./scripts/run.sh truth t_beams720
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

# The launch arguments `race` and `truth` share, in one place so a tuning change
# cannot drift between them. Everything here is from TEAM_IROS2026.md's race
# config; the two callers add only what differs.
#   AMCL_PARAMS  yaml in experiments/iros2026/params/, or an absolute path
#   LOCALIZER    amcl (default) | v2 | slam -- which node owns map->odom
#   SCAN_DUMP    with a NAME, also save every scan to runs/NAME.npz for
#                tools/replay_localization_v2.py (SCAN_DUMP=1)
#   V_MAX/ENC_WIN/HZ_CAP/RVIZ  override a single knob without editing this file
mt_args() {
  local line="$1" name="$2"
  local params="${AMCL_PARAMS:-amcl_beams360.yaml}"
  case "$params" in */*) ;; *) params="$IN/experiments/iros2026/params/$params" ;; esac
  MT_ARGS=(
    track:=iros2026 rviz:="${RVIZ:-false}" localizer:="${LOCALIZER:-amcl}"
    tcp_nodelay:=true loop_hz_cap:="${HZ_CAP:-45}" control_hz:=45
    path_csv:="$IN/raceline/iros2026/$line"
    log_csv:="$IN/runs/${name}.csv"
    scan_dump:="$( [ "${SCAN_DUMP:-0}" = "1" ] && echo "$IN/runs/${name}.npz" )"
    distance_source:=slip v_max:="${V_MAX:-9.0}" enc_rate_window_s:="${ENC_WIN:-0.10}"
    warmup_v_max:=2.0 warmup_dist_m:=21.0
    amcl_params_file:="$params"
    exit_guard_from:=0.0 exit_guard_full:=0.0
    controller_mode:=hybrid_lqr lqr_k_lat:=0.03 lqr_k_head:=0.05 lqr_k_yaw:=0.0
    lqr_max_correction_rad:=0.02
    recover_settle_s:=2.0 recover_creep_s:=3.0
  )
}

# First argument is an optional run NAME; anything containing := is already a
# launch override and must not be eaten. Sets RUN_NAME, and returns 0 only when
# a name was consumed, so the caller knows whether to shift.
#   run_name "${1:-}" <default> && shift || true
run_name() {
  RUN_NAME="$2"
  case "${1:-}" in
    ""|*:=*) return 1 ;;
    *) RUN_NAME="$1"; return 0 ;;
  esac
}

racer_run() {
  mkdir -p "$REPO/runs_docker" "$REPO/logs"
  xhost local:root >/dev/null 2>&1 || true
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  # runs/ takes relative log_csv:=; logs/ is where run_mt.sh and `race` write.
  local mounts=(-v "$REPO/runs_docker:$IN/runs" -v "$REPO/logs:$IN/logs")
  if [ "${MOUNT_RACELINE:-1}" = "1" ]; then
    mounts+=(-v "$REPO/raceline:$IN/raceline")
  fi
  # Same reason as raceline/: AMCL tuning is a loop of "edit the yaml, run
  # again", and the image bakes experiments/, so without this every parameter
  # change would need a 5 GB rebuild. MOUNT_PARAMS=0 uses the baked copy.
  if [ "${MOUNT_PARAMS:-1}" = "1" ] && [ -d "$REPO/experiments/iros2026/params" ]; then
    mounts+=(-v "$REPO/experiments/iros2026/params:$IN/experiments/iros2026/params")
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
    run_name "${1:-}" race && shift || true
    mt_args rl_mt_b05b15w05_ell_L65_B50_v9.0.csv "$RUN_NAME"
    racer_run "${MT_ARGS[@]}" "$@"
    ;;
  truth)
    # DIAGNOSTIC, NOT RACE-LEGAL. Steers on the simulator's ground-truth pose
    # (drive_on_truth:=true), which race.launch.py refuses in mode:=race and the
    # follower prints in red. raceline/FINDINGS.md section 11: 8.45 s mean over
    # 22 clean laps, against 8.92 for the best AMCL line -- that half second is
    # what localization costs, not a lap time this car can enter a race with.
    # The line is solved for the 0.09 m worst-case error the true-pose car has
    # (AMCL has 0.13 m), so driving it ON AMCL puts it into the right wall.
    #
    # It is also the AMCL tuning rig. race.launch.py line 174: "the localizer
    # still runs, so its error is still logged beside a car that is not using
    # it" -- and instruments.launch.py gets the unforced use_tf, so map->odom is
    # published and measured either way. The car therefore drives an identical
    # line whatever AMCL does, which makes the AMCL parameters the only
    # variable. Sweep them with AMCL_PARAMS= and compare runs/NAME.csv.
    shift
    run_name "${1:-}" truth && shift || true
    mt_args rl_mt_tb10_lat875_b55_L70.csv "$RUN_NAME"
    echo "[run.sh] DIAGNOSTIC: drive_on_truth -- ground-truth pose, NOT race-legal" >&2
    echo "[run.sh] amcl params: ${AMCL_PARAMS:-amcl_beams360.yaml}" >&2
    racer_run "${MT_ARGS[@]}" drive_on_truth:=true steer_a_lat_max:=8.0 "$@"
    ;;
  shadow)
    # The A/B nobody has to trust: the `race` config with AMCL owning map->odom
    # and localization_v2 running beside it without TF. log_localization writes
    # both estimates against truth (err_* is AMCL's, v2_* is v2's) and, with
    # SCAN_DUMP=1, the scan dump that tools/replay_localization_v2.py replays.
    shift
    run_name "${1:-}" shadow && shift || true
    LOCALIZER=amcl mt_args rl_mt_b05b15w05_ell_L65_B50_v9.0.csv "$RUN_NAME"
    echo "[run.sh] shadow: localizer=amcl + localization_v2 (no TF); v2_* columns in runs/${RUN_NAME}.csv" >&2
    racer_run "${MT_ARGS[@]}" v2_shadow:=true "$@"
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
    echo "usage: $0 {sim [--headless [BRIDGE_IP]] | race [NAME] | truth [NAME] | shadow [NAME] | racer name:=value ... | shell | exec}" >&2
    exit 1
    ;;
esac
