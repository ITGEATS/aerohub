"""Fetch and verify the HF openai/clip-vit-large-patch14 port into ignored artifacts/.

Every file is pinned by SHA256 in artifacts.json. Downloads go to a temporary
file first and are checked before being placed; an existing correct file is
reused and a mismatching one fails loudly (never silently overwritten).
Runs with any Python 3 (standard library only) so it can execute before `uv sync`.
"""
import hashlib
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

MODEL = Path(__file__).resolve().parent
ARTIFACTS = MODEL / "artifacts"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(url, target, timeout=1800):
    temporary = target.parent / (target.name + ".download")
    if temporary.exists():
        temporary.unlink()
    request = urllib.request.Request(url, headers={"User-Agent": "aerohub-clip"})
    with urllib.request.urlopen(request, timeout=timeout) as response, temporary.open("xb") as output:
        while True:
            chunk = response.read(1 << 20)
            if not chunk:
                break
            output.write(chunk)
    temporary.replace(target)


def fetch_file(name, record, endpoints):
    target = record["directory_path"] / name
    want = record["files"][name]["sha256"]
    if target.is_file():
        got = sha256(target)
        if got == want:
            print(f"Verified existing {name} (reused, not rewritten)")
            return
        raise ValueError(f"Existing {name} checksum mismatch: {got} != {want}; "
                         f"inspect {target} before retrying")
    last_error = None
    for endpoint in endpoints:
        url = f"{endpoint}/{record['repo']}/resolve/{record['revision']}/{name}"
        for attempt in range(3):
            try:
                print(f"Downloading {url}")
                target.parent.mkdir(parents=True, exist_ok=True)
                download(url, target)
                got = sha256(target)
                if got != want:
                    target.unlink()
                    raise ValueError(f"Downloaded {name} checksum mismatch: {got} != {want}")
                print(f"Verified {name} sha256={got}")
                return
            except (ValueError, OSError) as error:  # mirror hiccup: try next endpoint/retry
                last_error = error
                print(f"  attempt failed: {error}", file=sys.stderr)
                time.sleep(5 * (attempt + 1))
    raise RuntimeError(f"Could not prepare {name} from {endpoints}: {last_error}")


def main():
    record = json.loads((MODEL / "artifacts.json").read_text(encoding="utf-8"))["clip"]
    endpoints = [os.environ.get("HF_ENDPOINT")] + record["endpoints"]
    endpoints = [e for e in dict.fromkeys(endpoints) if e]
    record["directory_path"] = MODEL / record["directory"]
    if not record["directory_path"].is_dir():
        record["directory_path"].mkdir(parents=True)
    for name in sorted(record["files"]):
        fetch_file(name, record, endpoints)
    manifest = {
        "model": "clip",
        "repo": record["repo"],
        "revision": record["revision"],
        "endpoints": endpoints,
        "files": [{"path": name, "sha256": record["files"][name]["sha256"],
                   "bytes": record["files"][name]["bytes"]} for name in sorted(record["files"])],
    }
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    (ARTIFACTS / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared CLIP files in {record['directory_path']}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError) as error:
        print(f"Artifact preparation failed: {error}", file=sys.stderr)
        sys.exit(1)
