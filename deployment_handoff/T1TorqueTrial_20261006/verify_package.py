"""Offline integrity and policy contract check. Does not connect to the robot."""

from pathlib import Path
import hashlib
import sys


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools"))
from footroom_debug import FootRoomShadow, Pose2D  # noqa: E402


MODELS = (
    ("mild200", "model_200.pt", "f3fe8ddd3d66e4441b4840448bd0eec6954cd7749054a204966f6895aa9bba57"),
    ("firm800", "model_800.pt", "5e8786e5fff1506345193570908d340df5c8cb864b7018cf268f033ea18bd8a8"),
)


def main():
    for name, filename, expected in MODELS:
        folder = ROOT / "models" / name
        model = folder / filename
        actual = hashlib.sha256(model.read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError("%s SHA256 mismatch: %s" % (name, actual))
        shadow = FootRoomShadow(config_path=folder / "train.yaml", model_path=model,
                                expected_model_sha256=expected)
        result = shadow.step(0.0, shadow.default_q.tolist(), [0.0] * 23,
                             [0.0] * 3, [0.0] * 3, [0.5, 0.0, 0.0],
                             Pose2D(0.0, 0.0, 0.0))
        sizes = tuple(len(result[key]) for key in
                      ("observation", "action", "targets_urdf_rad"))
        if sizes != (80, 21, 23):
            raise ValueError("%s wrong policy dimensions: %s" % (name, sizes))
        print("%s OK: SHA256 %s; observation/action/targets %s" %
              (name, actual, sizes))


if __name__ == "__main__":
    main()
