#!/usr/bin/env bash
# Verifies the sim + bridge are up, then starts SAC training.
# NOTE: no `-u`. ROS's setup.bash references unbound variables
# (AMENT_TRACE_SETUP_FILES et al), so `set -u` makes sourcing it fail.
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
# ROS first, then the venv: ROS puts rclpy on PYTHONPATH, the venv must
# win on PATH. Training needs the venv (torch+CUDA live only there).
VENV="$HOME/Documents/roboracer/.venv-rl/bin/activate"
if [ -f "$VENV" ]; then source "$VENV"; else
  echo "ERROR: venv missing at $VENV"; exit 1
fi

if ! pgrep -f "AutoDRIVE Simulator.x86_64" >/dev/null; then
  echo "ERROR: simulator not running. Start it with:"
  echo "  cd ../simulator_practice/autodrive_simulator && ./AutoDRIVE\\ Simulator.x86_64"
  exit 1
fi
if ! ss -ltn 2>/dev/null | grep -q 4567; then
  echo "ERROR: bridge not listening on 4567. Start it with:"
  echo "  source ../devkit_ws/install/setup.bash"
  echo "  ros2 launch autodrive_roboracer bringup_headless.launch.py"
  exit 1
fi
echo "sim + bridge OK"
python -c "import torch; assert torch.cuda.is_available(), 'CUDA not available'" \
  || { echo "ERROR: CUDA unavailable inside venv"; exit 1; }
echo "venv + CUDA OK"
exec python train_sac.py "$@"
