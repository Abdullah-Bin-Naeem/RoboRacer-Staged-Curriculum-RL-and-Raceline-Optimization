"""Loading SAC checkpoints, including ones saved with a frozen entropy coefficient.

ROS-free, so `enjoy.py` and `train_sac.py` can both use it.

Freezing the entropy coefficient (`--ent-coef`) clears `ent_coef_optimizer`, so
SB3 writes no `ent_coef_optimizer.pth`. Unless `model.ent_coef` is also set to
the float, the checkpoint's metadata still reads `"auto"`, so the model SB3
rebuilds on load expects that optimizer, `set_parameters` fails on the
parameter-name mismatch, and SB3's own fallback then dies on
`KeyError: 'policy.optimizer'` -- which is the error that actually escapes.
The frozen value is in the zip as `pytorch_variables`' `ent_coef_tensor`, so
such a checkpoint is complete and loads fine once `ent_coef` is handed back as
that float. Checkpoints written after the `model.ent_coef` fix need none of this.
"""
import io
import json
import zipfile


def frozen_ent_coef(path):
    """The frozen entropy coefficient of a checkpoint whose metadata still says
    "auto", or None if the checkpoint is not in that state (so a load failure
    is a real one and must not be retried)."""
    import torch
    try:
        with zipfile.ZipFile(path) as z:
            names = z.namelist()
            if "ent_coef_optimizer.pth" in names or "pytorch_variables.pth" not in names:
                return None
            if json.loads(z.read("data")).get("ent_coef") != "auto":
                return None
            pv = torch.load(io.BytesIO(z.read("pytorch_variables.pth")),
                            map_location="cpu", weights_only=False)
        value = pv.get("ent_coef_tensor") if isinstance(pv, dict) else None
        return None if value is None else float(value)
    except Exception:
        return None


def load_sac(path, ent_coef=None, **kw):
    """`SAC.load`, repairing a checkpoint saved with a frozen entropy coefficient.

    `ent_coef` overrides the value handed to the rebuilt model; leave it None to
    use the one stored in the checkpoint. It only matters for a resume that
    keeps training -- inference never reads it.
    """
    from stable_baselines3 import SAC
    try:
        return SAC.load(path, **kw)
    except (ValueError, KeyError):
        frozen = frozen_ent_coef(path)
        if frozen is None:
            raise
        use = frozen if ent_coef is None else float(ent_coef)
        print(f"[rl_racer] {path} was saved with a FROZEN entropy coefficient "
              f"({frozen:.4f}) while its metadata still says 'auto'; loading it "
              f"frozen at {use:.4f}.", flush=True)
        return SAC.load(path, custom_objects={"ent_coef": use}, **kw)
