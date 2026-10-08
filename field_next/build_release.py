"""Build the reviewable, offline S46 source bundle from the local project."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import zipfile


EXCLUDED_PARTS = {".git", ".venv", "__pycache__", "logs"}
EXCLUDED_SUFFIXES = {".pyc", ".log"}
MODEL_SHA256 = "80af933e2034601a51e3ff33e151df973d0b3540f531c2110984957632f22639"


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def needed_for_t1_s46(path: Path, source: Path) -> bool:
    parts = path.relative_to(source).parts
    if EXCLUDED_PARTS.intersection(parts) or path.suffix in EXCLUDED_SUFFIXES:
        return False
    if parts[0] == "docs":
        return False
    if parts[0] == "tasks" and len(parts) > 1:
        if parts[1] not in {"hrt_s46", "locomotion"}:
            return False
        if parts[1] == "locomotion" and len(parts) > 3 and parts[2] == "robots":
            return parts[3] in {"t1", "__init__.py"}
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("S46_OFFLINE_CANDIDATE_20261008.zip"))
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    model = source / "tasks" / "hrt_s46" / "models" / "model_2200.pt"
    if digest(model.read_bytes()) != MODEL_SHA256:
        raise SystemExit("S46 model hash mismatch")
    if args.output.resolve() == source or source in args.output.resolve().parents:
        raise SystemExit("Output must be outside source directory")
    files = [
        p for p in source.rglob("*")
        if p.is_file() and needed_for_t1_s46(p, source)
    ]
    files.sort(key=lambda p: p.relative_to(source).as_posix())
    manifest = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            content = path.read_bytes()
            name = "booster_deploy_s46/" + path.relative_to(source).as_posix()
            archive.writestr(name, content)
            manifest.append(f"{digest(content)}  {name}")
        for filename in ("README.md", "REVIEW_20261008.md"):
            content = Path(__file__).with_name(filename).read_bytes()
            name = "FIELD_NEXT/" + filename
            archive.writestr(name, content)
            manifest.append(f"{digest(content)}  {name}")
        archive.writestr("SHA256SUMS.txt", "\n".join(manifest) + "\n")
    output_digest = digest(args.output.read_bytes())
    args.output.with_suffix(".sha256").write_text(
        f"{output_digest}  {args.output.name}\n", encoding="utf-8"
    )
    print(f"{args.output.name}: {len(files)} files, sha256 {output_digest}")


if __name__ == "__main__":
    main()
