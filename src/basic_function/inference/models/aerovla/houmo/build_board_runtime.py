"""Assemble the AeroVLA Houmo Python service for upload to RoboPi.

Mirrors yopo/houmo/build_board_runtime.py. The 13 GiB HMM files and the
token-embedding assets are NOT copied here: they already live on the board at
/data/local/workspace/aerovla_infer (pinned by artifacts.json), and the packed
runtime expects them under AEROVLA_INFER_DIR (default: <runtime>/aerovla_infer,
which the deploy step symlinks to the existing directory).
"""

import argparse
import json
import shutil
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
HERE = Path(__file__).resolve().parent
MODEL = HERE.parent


def copy_tree(source, target):
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=HERE / "artifacts/board-runtime")
    args = parser.parse_args()

    output = args.output.resolve()
    # Fail closed on an existing destination; never recursively delete user paths.
    if output.exists():
        raise FileExistsError("Choose a new --output directory: {}".format(output))
    output.mkdir(parents=True)

    for name in ("adapter.py", "serve.py", "start_board.sh", "backend_aerovla.py"):
        shutil.copy2(HERE / name, output / name)
    for name in ("schemas.py", "processing.py", "predictor.py"):
        shutil.copy2(MODEL / name, output / name)
    shutil.copy2(HERE / "artifacts.json", output / "artifacts.json")
    copy_tree(ROOT / "src/basic_function/inference/runtime/aerohub_inference",
              output / "aerohub_inference")

    wheelhouse = HERE / "artifacts/wheels/board"
    wheels = sorted(wheelhouse.glob("*.whl"))
    names = {path.name.split("-", 1)[0] for path in wheels}
    required = {"fastapi", "starlette", "uvicorn"}
    if not required <= names:
        raise RuntimeError("board wheelhouse must contain at least %s, found %s"
                           % (sorted(required), sorted(names)))
    for wheel in wheels:
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(output)

    manifest = {
        "runtime": "aerovla-houmo",
        "python": "3.12 (board /sys_prod/robot/out/bin/python3)",
        "board_prerequisites": {
            "tcim_python": "/data/local/houmo/python (tcim_lite)",
            "tcim_libs": "/data/local/houmo/lib",
            "infer_dir": "/data/local/workspace/aerovla_infer (models/, assets/, artifacts.json)",
        },
        "packed_files": sorted(p.relative_to(output).as_posix()
                               for p in output.rglob("*") if p.is_file()
                               and "aerohub_inference" not in p.parts
                               and p.suffix in (".py", ".sh", ".json")),
    }
    (output / "runtime-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
