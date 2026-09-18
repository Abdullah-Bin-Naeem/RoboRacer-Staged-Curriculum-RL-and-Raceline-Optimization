#!/usr/bin/env bash
# Run one tuning experiment end to end: bridge check, reset, run, report, index.
#
#   tools/tuning/run_experiment.sh ID SLUG --line CSV [--laps N] [--max-contacts N] [--hz stock|N]
#       [--hypothesis TEXT] [--prediction TEXT] [--rviz] [-- launch_arg:=value ...]
#
# --rviz opens the AMCL RViz view (map, particles, scan, raceline, AMCL pose, ground truth,
# lookahead) on this desktop. Needs X access for the container once per login:
#   xhost +SI:localuser:root
#
# Writes experiments/<track>/<ID>_<SLUG>/ (config.json, run.csv, laps.csv, launch.log,
# summary.json, report.md, figures/) and its row in experiments/<track>/EXPERIMENTS.md.
# The bridge runs in its own container (rr_bridge) for the whole session; if it is missing
# or on a different loop setting it is (re)started and this script waits for Connect.
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE=roboracer-dev:iros-compete
TRACK=iros2026
ID=$1; SLUG=$2; shift 2
LINE=""; LAPS=25; MAXC=1; HZ=stock; HYP=""; PRED=""; RVIZ=false; OVR=()
while [ $# -gt 0 ]; do
    case $1 in
        --line) LINE=$2; shift 2 ;;
        --laps) LAPS=$2; shift 2 ;;
        --max-contacts) MAXC=$2; shift 2 ;;
        --hz) HZ=$2; shift 2 ;;
        --hypothesis) HYP=$2; shift 2 ;;
        --prediction) PRED=$2; shift 2 ;;
        --rviz) RVIZ=true; shift ;;
        --) shift; OVR=("$@"); break ;;
        *) echo "unknown argument $1"; exit 2 ;;
    esac
done
[ -n "$LINE" ] && [ -f "$REPO/raceline/$TRACK/$LINE" ] || { echo "line raceline/$TRACK/$LINE not found"; exit 2; }
REL="experiments/$TRACK/${ID}_${SLUG}"
EXP="$REPO/$REL"
[ -e "$EXP/run.csv" ] && { echo "$REL already has a run; pick a new ID"; exit 2; }
mkdir -p "$EXP"

# ---- bridge: one per session, keyed by the loop setting
CUR=$(docker inspect -f '{{index .Config.Labels "hz"}}' rr_bridge 2>/dev/null || echo none)
if [ "$(docker inspect -f '{{.State.Running}}' rr_bridge 2>/dev/null)" != true ] || [ "$CUR" != "$HZ" ]; then
    docker rm -f rr_bridge >/dev/null 2>&1 || true
    docker run -d --name rr_bridge --label hz="$HZ" --net=host --ipc=host \
        -e HZ_CAP="$HZ" -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
        -v "$REPO:/root/Documents/roboracer" $IMAGE bash docker/dev/bridge.sh >/dev/null
    echo ">>> bridge (re)started with loop setting '$HZ'. In the simulator: press Reset, then Connect."
    until docker logs rr_bridge 2>&1 | grep -q "Connected!"; do
        docker inspect -f '{{.State.Running}}' rr_bridge | grep -q true || { docker logs rr_bridge | tail -20; exit 1; }
        sleep 2
    done
    echo ">>> bridge connected"
fi

# ---- config record
python3 - "$EXP/config.json" <<PY
import json, subprocess, sys, datetime
git = lambda *a: subprocess.run(['git', '-C', '$REPO', *a], capture_output=True, text=True).stdout.strip()
json.dump(dict(id='$ID', name='$SLUG', date=datetime.date.today().isoformat(),
               started=datetime.datetime.now().isoformat(timespec='seconds'), track='$TRACK',
               line='$LINE', laps=$LAPS, max_contacts=$MAXC, hz_cap='$HZ',
               overrides=[a for a in """${OVR[*]}""".split() if a],
               hypothesis="""$HYP""", prediction="""$PRED""",
               git_branch=git('rev-parse', '--abbrev-ref', 'HEAD'), git_commit=git('rev-parse', '--short', 'HEAD'),
               git_dirty=bool(git('status', '--porcelain', '--', 'devkit_ws/src', 'raceline'))),
          open(sys.argv[1], 'w'), indent=2)
PY

# ---- run
echo ">>> $REL: $LINE, $LAPS laps, stop at $MAXC contact(s), loop $HZ, overrides: ${OVR[*]:-(none)}"
docker rm -f rr_exp >/dev/null 2>&1 || true
XARGS=()
if [ "$RVIZ" = true ]; then
    XARGS=(-e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw
           --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all
           -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)
fi
docker run --rm --name rr_exp --net=host --ipc=host "${XARGS[@]}" -e RVIZ="$RVIZ" \
    -e EXP_DIR="/root/Documents/roboracer/$REL" -e LINE="$LINE" -e LAPS="$LAPS" -e MAX_CONTACTS="$MAXC" \
    -e TRACK="$TRACK" -e OVERRIDES="${OVR[*]}" -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -v "$REPO:/root/Documents/roboracer" $IMAGE bash docker/dev/experiment.sh

# ---- report
docker run --rm --net=host -e HOST_UID="$(id -u)" -v "$REPO:/root/Documents/roboracer" -w /root/Documents/roboracer \
    $IMAGE bash -c "python3 tools/tuning/report.py $REL; chown -R $(id -u):$(id -g) $REL experiments/$TRACK/EXPERIMENTS.md"
echo ">>> report: $REL/report.md"
