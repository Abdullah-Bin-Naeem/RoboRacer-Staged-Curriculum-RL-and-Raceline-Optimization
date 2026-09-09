#!/usr/bin/env bash
# Benchmark the localizer against the simulator's ground truth.
#
#     ./scripts/run.sh sim --headless      # terminal 1: the simulator
#
#     ./scripts/bench.sh up                # start the bench container
#     ./scripts/bench.sh record baseline   # drive and log      -> runs/baseline.csv
#     ./scripts/bench.sh score baseline    # analyse            -> runs/baseline.html
#     ./scripts/bench.sh sweep             # every variant, ranked
#     ./scripts/bench.sh shell             # a ROS shell in the bench container
#     ./scripts/bench.sh down
#
# WHY A SEPARATE CONTAINER FROM run.sh
# ------------------------------------
# `run.sh racer` is the submission, run exactly as the organizers run it: no
# mounts, no extra environment, default entrypoint. Benchmarking needs three
# things that would each change what is being submitted --
#
#   a host mount        the CSV is written inside the container and would
#                       otherwise die with --rm
#   RACER_MODE=dev      instruments.launch.py is where both ground-truth
#                       readers live, and mode:=race omits that file wholesale
#   scipy               log_localization builds a distance-to-wall field at
#                       startup for the fit_* columns, and the submission image
#                       does not ship scipy
#
# -- so it gets its own container name and leaves the image untouched. The
# tools are BIND-MOUNTED rather than baked in, so nothing here can end up in a
# submission by accident.
#
# WHAT IS AND IS NOT MEASURED
# ---------------------------
# mode:=dev adds the instrument nodes and leaves the control path alone:
# use_tf_pose stays true and bootstrap_seconds stays 0, so the car steers on
# the localizer's estimate exactly as it does in a timed run. The numbers are
# about the pose the car really races on.
#
# /ips and /odom are read continuously here. That is restricted during timed
# laps and entirely fine in development -- see roboracer_stack/common/restricted.py.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-autodrive_racer}"
TAG="${TAG:-qualification-1}"
NAME="${BENCH_NAME:-autodrive_roboracer_bench}"
RUNS="$REPO/runs"

# Inside the container.
WS=/home/autodrive_devkit
SHARE="$WS/install/roboracer_stack/share/roboracer_stack"
C_RUNS="$WS/runs"
C_TOOLS="$WS/tools"

SECONDS_DEFAULT="${BENCH_SECONDS:-90}"
LOG_RATE="${BENCH_LOG_RATE:-20.0}"
CONTROL_HZ="${BENCH_CONTROL_HZ:-40}"

die() { echo "bench: $*" >&2; exit 1; }

# Run a command inside the bench container with ROS sourced. The entrypoint
# already exported the DDS settings as image ENV, but a fresh `docker exec`
# shell still has to source the workspace itself -- nothing is automated from
# ~/.bashrc, by design.
# NOTE the absence of `set -u`. ROS's setup.bash reads AMENT_TRACE_SETUP_FILES
# and friends with no default, so under `set -u` the shell dies on the first
# source -- with stderr redirected, silently, and every later command reports
# "not found". autodrive_devkit.sh carries the same warning and the same fix;
# this is the second place that lesson had to be learned.
dex() {
  # No -i, and stdin closed. `docker exec -i` inherits the caller's stdin, and
  # cmd_sweep drives a `while read ... < variants.tsv` loop -- so the first exec
  # inside the loop SWALLOWS THE REST OF THE FILE and the sweep runs one variant
  # and stops. Nothing errors; it just quietly does a fraction of the work.
  docker exec "$NAME" bash -c "set -o pipefail
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source $WS/install/setup.bash >/dev/null 2>&1
    $*" < /dev/null
}

running() { [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = true ]; }

need_up() {
  running || die "container '$NAME' is not running -- ./scripts/bench.sh up"
}

# ---------------------------------------------------------------- up --------
cmd_up() {
  if running; then echo "bench: '$NAME' already up"; else
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    mkdir -p "$RUNS"
    # RACER_AUTOSTART=0 is already in autodrive_devkit.sh for exactly this: the
    # entrypoint sets the environment and sources the workspace but starts no
    # nodes, then execs the command -- so `sleep infinity` holds the container
    # open and every run below is started deliberately, one at a time. Without
    # it a second stack would come up behind us on the first exec.
    docker run -d --name "$NAME" --network=host --ipc=host \
      -e RACER_AUTOSTART=0 \
      -v "$RUNS:$C_RUNS" \
      -v "$REPO/roboracer_stack/tools:$C_TOOLS:ro" \
      -v "$REPO/roboracer_stack/raceline:$WS/raceline:ro" \
      "${IMAGE}:${TAG}" sleep infinity >/dev/null
    echo "bench: started '$NAME'  (runs -> $RUNS)"
  fi

  # scipy, for log_localization's fit_* columns. Those are the only thing that
  # separates "AMCL picked the wrong pose" from "the map is wrong here", and
  # without them the node fails at startup rather than degrading, because it
  # builds the distance field in __init__.
  #
  # --no-deps IS LOAD-BEARING, and a plain `pip install scipy` is actively
  # destructive here. Current scipy declares numpy>=1.23, the image ships
  # 1.22.2, so pip helpfully upgrades numpy to 2.x -- and every compiled ROS
  # Humble extension (rclpy, tf2_py, cv_bridge, laser_geometry) is built
  # against the numpy 1.x ABI and stops importing. The whole stack dies, in a
  # container whose only purpose was to measure it.
  #
  # scipy 1.10.x is the last line whose declared floor (numpy>=1.19.5) the
  # image's 1.22.2 actually satisfies -- 1.13 imports but warns it wants
  # >=1.22.4, and a numerical library running outside its supported numpy is
  # not something to build a benchmark on. --no-deps then guarantees the
  # installed numpy is not touched at all.
  if dex "python3 -c 'import scipy.ndimage' 2>/dev/null"; then
    echo "bench: scipy present"
  else
    local np_before; np_before=$(dex "python3 -c 'import numpy;print(numpy.__version__)'" | tr -d '\r\n')
    echo "bench: installing scipy (once per container; the fit_* columns need it)"
    dex "pip install --quiet --no-input --no-deps 'scipy==1.10.1'" \
      || echo "bench: WARNING scipy install failed"
    local np_after; np_after=$(dex "python3 -c 'import numpy;print(numpy.__version__)'" | tr -d '\r\n')
    if [ "$np_before" != "$np_after" ]; then
      echo "bench: numpy changed $np_before -> $np_after; rolling back (ROS needs 1.x)" >&2
      dex "pip install --quiet --no-input --force-reinstall 'numpy==$np_before'" || true
    fi
    # Prove the stack still imports before anything is recorded against it.
    if dex "python3 -c 'import scipy.ndimage, rclpy, tf2_ros' 2>/dev/null"; then
      echo "bench: scipy installed; rclpy and tf2_ros still import"
    else
      echo "bench: WARNING scipy is unusable or broke the ROS python modules." >&2
      echo "bench:   fit_* columns will be unavailable; everything else still works." >&2
      dex "pip uninstall -y --quiet scipy" >/dev/null 2>&1 || true
    fi
  fi

  echo "bench: simulator must be running separately: ./scripts/run.sh sim --headless"
}

cmd_down() { docker rm -f "$NAME" >/dev/null 2>&1 && echo "bench: removed '$NAME'" || true; }

cmd_shell() { need_up; docker exec -it "$NAME" bash; }

# ------------------------------------------------------------ record --------
# record <name> [--seconds N] [--params FILE] [extra launch args...]
cmd_record() {
  need_up
  local run="${1:-}"; shift || true
  [ -n "$run" ] || die "usage: bench.sh record <name> [--seconds N] [launch args...]"
  local secs="$SECONDS_DEFAULT" params='' extra=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --seconds) secs="$2"; shift 2 ;;
      --params)  params="$2"; shift 2 ;;
      *)         extra+=("$1"); shift ;;
    esac
  done

  local csv="$C_RUNS/$run.csv"
  local args=(
    "mode:=dev"
    "localizer:=amcl"
    "bootstrap_mode:=truth"
    "control_hz:=$CONTROL_HZ"
    "rviz:=false"
    "measure_error:=true"
    "log_csv:=$csv"
    "log_rate:=$LOG_RATE"
    # The grid the fit_* columns score the scan against MUST be the one the
    # localizer is using, or fit_ratio is meaningless. log_localization now
    # defaults to exactly that -- common.frames.DEFAULT_FIT_MAP, AMCL's grid for
    # the current track -- so this no longer names a map. It used to hardcode
    # Porto's maps/track_clean, which scored every icra run against the wrong
    # circuit. Pass log_map:= yourself to score against a different grid.
    # Body-to-wall clearance at the tightest point of raceline_a7.0.
    "wall_margin:=0.134"
  )
  [ -n "$params" ] && args+=("amcl_params:=$params")
  # NOT "${extra[@]:-}": on an empty array that expands to one EMPTY argument,
  # which ros2 launch takes as a malformed parameter and answers with
  # "Parameter file path is not a file: ." -- a warning that has nothing to do
  # with the actual problem and sends you looking in the wrong place.
  [ ${#extra[@]} -gt 0 ] && args+=("${extra[@]}")

  rm -f "$RUNS/$run.csv"
  echo "bench: recording '$run' for ${secs}s"
  echo "bench:   ros2 launch roboracer_stack race.launch.py ${args[*]}"

  # timeout -s INT, not a kill: log_localization flushes its CSV and
  # localization_error prints its session summary on SIGINT. SIGKILL loses the
  # last buffered rows and the summary with them.
  dex "cd $C_RUNS && timeout -s INT ${secs}s \
        ros2 launch roboracer_stack race.launch.py ${args[*]} \
        > $C_RUNS/$run.launch.log 2>&1; true"

  # The launch log carries the localization_error summary, which is an
  # INDEPENDENT check on the analyzer: a different code path reading a
  # different truth source (/odom rather than /ips + /imu).
  echo
  sed -n '/localization error over/,/^=\{40,\}$/p' "$RUNS/$run.launch.log" 2>/dev/null | head -20 || true

  [ -s "$RUNS/$run.csv" ] || die "no CSV produced -- see $RUNS/$run.launch.log
  The usual cause is that the simulator was never connected, so the bridge
  blocked and the car never moved."
  local rows; rows=$(( $(wc -l < "$RUNS/$run.csv") - 1 ))
  echo "bench: $rows samples -> runs/$run.csv"
  [ "$rows" -gt 100 ] || echo "bench: WARNING only $rows samples; was the car driving?"
}

# ------------------------------------------------------------- score --------
cmd_score() {
  need_up
  [ $# -gt 0 ] || die "usage: bench.sh score <name> [<name>...]"
  local files=()
  for r in "$@"; do
    [ -s "$RUNS/${r%.csv}.csv" ] || die "runs/${r%.csv}.csv not found"
    files+=("$C_RUNS/${r%.csv}.csv")
  done
  local cmp=()
  [ ${#files[@]} -gt 1 ] && cmp=(--compare "$C_RUNS/compare.html")
  dex "python3 $C_TOOLS/analyze_localization.py ${files[*]} ${cmp[*]:-}"
}

# ------------------------------------------------------------- sweep --------
cmd_sweep() {
  need_up
  local secs="${1:-$SECONDS_DEFAULT}"
  local tsv="$REPO/roboracer_stack/tools/bench/variants.tsv"
  [ -f "$tsv" ] || die "missing $tsv"
  mkdir -p "$RUNS/params"
  local names=()

  # Read the whole list up front. Even with dex's stdin closed, a loop whose
  # body starts containers has no business holding a file open on stdin.
  local lines=() ln
  while IFS= read -r ln || [ -n "$ln" ]; do lines+=("$ln"); done < "$tsv"

  for ln in "${lines[@]}"; do
    IFS=$'\t' read -r name overrides extra <<< "$ln"
    case "${name:-}" in ''|\#*) continue ;; esac
    name="$(echo "$name" | xargs)"
    overrides="$(echo "${overrides:--}" | xargs)"
    extra="$(echo "${extra:--}" | xargs)"
    [ "$extra" = "-" ] && extra=''

    echo; echo "=== $name ================================================"
    local pfile=''
    if [ "$overrides" != "-" ]; then
      pfile="$C_RUNS/params/$name.yaml"
      dex "python3 $C_TOOLS/bench/make_variant.py \
             $SHARE/config/amcl.yaml $pfile '$overrides'"
    fi

    # Put the car back on the spawn between variants, so every one of them is
    # scored on the same start rather than on wherever the last run finished.
    dex "ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool \
           '{data: true}' >/dev/null 2>&1; true"

    # shellcheck disable=SC2086
    if cmd_record "$name" --seconds "$secs" \
         ${pfile:+--params "$pfile"} $extra; then
      names+=("$name")
    else
      echo "bench: $name failed to record; skipping" >&2
    fi
  done

  [ ${#names[@]} -gt 0 ] || die "no variant recorded"
  echo; echo "=== scoring ${#names[@]} runs ============================"
  cmd_score "${names[@]}"
  echo "bench: ranked comparison -> runs/compare.html"
}

case "${1:-}" in
  up)     shift; cmd_up "$@" ;;
  down)   shift; cmd_down "$@" ;;
  shell)  shift; cmd_shell "$@" ;;
  record) shift; cmd_record "$@" ;;
  score)  shift; cmd_score "$@" ;;
  sweep)  shift; cmd_sweep "$@" ;;
  *) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
