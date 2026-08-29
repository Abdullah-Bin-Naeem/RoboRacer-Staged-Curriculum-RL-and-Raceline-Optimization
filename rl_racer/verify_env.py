#!/usr/bin/env python3
"""Verify the training venv is complete and self-contained.

Run INSIDE the venv:
    source /opt/ros/humble/setup.bash
    source ~/Documents/roboracer/.venv-rl/bin/activate
    python verify_env.py
"""
import importlib, os, sys

VENV = "/home/theflash/Documents/roboracer/.venv-rl"

def where(mod):
    p = getattr(mod, "__file__", "") or getattr(mod, "__path__", [""])[0] or ""
    if VENV in p:            return "VENV"
    if "/.local/" in p:      return "USER(~/.local)"
    if "/opt/ros/" in p:     return "ROS"
    if "/usr/lib" in p or "/usr/local" in p: return "SYSTEM"
    return p[:40]

def check(name, attr="__version__", need_venv=False):
    try:
        m = importlib.import_module(name)
    except Exception as e:
        print(f"  {name:20s} MISSING  ({type(e).__name__})")
        return False
    ver = getattr(m, attr, "?")
    loc = where(m)
    flag = ""
    if need_venv and loc != "VENV":
        flag = "  <-- NOT in venv"
    print(f"  {name:20s} {str(ver):12s} {loc}{flag}")
    return not (need_venv and loc != "VENV")

ok = True
print("=== interpreter ===")
print(f"  python  : {sys.executable}")
in_venv = VENV in sys.executable
print(f"  in venv : {in_venv}")
if not in_venv:
    print("  !! Not running inside the venv -- activate it first.")
    sys.exit(1)

print("\n=== RL stack (should all say VENV) ===")
for m in ["torch", "stable_baselines3", "gymnasium", "tensorboard"]:
    ok &= check(m, need_venv=True)
ok &= check("numpy", need_venv=True)

print("\n=== GPU ===")
import torch
print(f"  cuda available      : {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"  device              : {torch.cuda.get_device_name(0)}")
    print(f"  VRAM                : {torch.cuda.get_device_properties(0).total_memory/1e9:.1f} GB")
    x = torch.randn(512, 512, device="cuda")
    print(f"  matmul smoke test   : {(x @ x).sum().item() != 0}")
else:
    print("  !! CUDA NOT AVAILABLE"); ok = False

print("\n=== ROS (must remain importable inside the venv) ===")
ok &= check("rclpy", "__name__")
ok &= check("cv_bridge", "__name__")
ok &= check("sensor_msgs", "__name__")

print("\n=== numpy ABI guard ===")
import numpy
major = int(numpy.__version__.split(".")[0])
print(f"  numpy {numpy.__version__} -> {'OK (<2)' if major < 2 else '!! >=2 BREAKS cv_bridge'}")
ok &= major < 2

print("\n=== project modules ===")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for m in ["rl_racer.config", "rl_racer.obs", "rl_racer.env"]:
    ok &= check(m, "__name__")

print("\n" + ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED (see <-- markers above)"))
sys.exit(0 if ok else 1)
