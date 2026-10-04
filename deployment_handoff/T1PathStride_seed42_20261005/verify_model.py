"""Offline check for the exported T1 PathStride actor. Does not command hardware."""

from pathlib import Path
import hashlib
import importlib.util

import torch


ROOT = Path(__file__).resolve().parent
PTH = ROOT / "model" / "model_800.pth"
PT = ROOT / "model" / "model_800.pt"
MODEL_SOURCE = ROOT / "reference" / "model.py"
EXPECTED_SHA256 = "3cfa7d4773981481b8ef73485712e8b83de8fda1dabbcc94a8fac228001cd93b"


def main():
    digest = hashlib.sha256(PTH.read_bytes()).hexdigest()
    assert digest == EXPECTED_SHA256, f"Unexpected checkpoint SHA256: {digest}"

    spec = importlib.util.spec_from_file_location("candidate_model", MODEL_SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    checkpoint = torch.load(PTH, map_location="cpu", weights_only=True)
    model = module.ActorCritic(21, 80, 14)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    # A file object also works around Windows TorchScript builds that cannot
    # open a Unicode path through their C++ filename API.
    with PT.open("rb") as stream:
        exported = torch.jit.load(stream, map_location="cpu")
    exported.eval()

    # Check two distinct observations so a constant or mismatched export is rejected.
    observations = torch.stack((torch.zeros(80), torch.linspace(-0.2, 0.2, 80)))
    with torch.no_grad():
        expected = model.actor(observations)
        actual = exported(observations)
    assert tuple(actual.shape) == (2, 21), f"Wrong output shape: {tuple(actual.shape)}"
    assert bool(torch.isfinite(actual).all()), "Non-finite actor output"
    max_error = float((expected - actual).abs().max())
    assert max_error < 1e-6, f"Export differs from checkpoint: {max_error}"
    print(f"OK: checkpoint SHA256 {digest}; actor 80 -> 21; max export error {max_error:.3g}")


if __name__ == "__main__":
    main()
