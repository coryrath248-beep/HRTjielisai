"""Build the one-click S62 competition controller bundle."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import zipfile


EXCLUDED_PARTS = {".git", ".venv", "__pycache__", "logs"}
EXCLUDED_SUFFIXES = {".pyc", ".log"}
MODEL_SHA256 = "6cf343cd70a59156ee1e25655754acda0a067b62907b7c1b211e3763fd8f066f"
ARCHIVE_ROOT = "T1_S62_COMPETITION_DEPLOY"


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def needed_for_t1_s62(path: Path, source: Path) -> bool:
    parts = path.relative_to(source).parts
    if parts == ("README.md",):
        return False
    if EXCLUDED_PARTS.intersection(parts) or path.suffix in EXCLUDED_SUFFIXES:
        return False
    if parts[0] == "docs":
        return False
    if parts[0] == "tasks" and len(parts) > 1:
        if parts[1] not in {"hrt_s62", "locomotion"}:
            return False
        if parts[1] == "locomotion" and len(parts) > 3 and parts[2] == "robots":
            return parts[3] in {"t1", "__init__.py"}
    if parts[0] == "deployment" and len(parts) > 1:
        if parts[1] == "t1-s46.service":
            return False
        if parts[1] == "tests" and path.name == "validate_s46_stop_offline.py":
            return False
    return True


def write_file(archive: zipfile.ZipFile, name: str, content: bytes, executable: bool = False) -> None:
    info = zipfile.ZipInfo(name)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = ((0o755 if executable else 0o644) & 0xFFFF) << 16
    archive.writestr(info, content, compresslevel=6)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("T1_S62_COMPETITION_DEPLOY_20261010.zip"))
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    model = source / "tasks" / "hrt_s62" / "models" / "model.pt"
    if digest(model.read_bytes()) != MODEL_SHA256:
        raise SystemExit("field model hash mismatch")
    if args.output.resolve() == source or source in args.output.resolve().parents:
        raise SystemExit("Output must be outside source directory")
    files = [
        p for p in source.rglob("*")
        if p.is_file() and needed_for_t1_s62(p, source)
    ]
    files.sort(key=lambda p: p.relative_to(source).as_posix())
    manifest = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            content = path.read_bytes()
            relative = path.relative_to(source).as_posix()
            name = ARCHIVE_ROOT + "/" + relative
            write_file(archive, name, content, executable=path.suffix == ".sh")
            manifest.append(f"{digest(content)}  {name}")
        for filename in ("README.md", "S62_MODEL_CARD.json"):
            content = Path(__file__).with_name(filename).read_bytes()
            name = ARCHIVE_ROOT + "/" + filename
            write_file(archive, name, content)
            manifest.append(f"{digest(content)}  {name}")
        write_file(
            archive,
            ARCHIVE_ROOT + "/SHA256SUMS.txt",
            ("\n".join(manifest) + "\n").encode("utf-8"),
        )
    output_digest = digest(args.output.read_bytes())
    args.output.with_suffix(".sha256").write_text(
        f"{output_digest}  {args.output.name}\n", encoding="utf-8"
    )
    print(f"{args.output.name}: {len(files)} files, sha256 {output_digest}")


if __name__ == "__main__":
    main()
