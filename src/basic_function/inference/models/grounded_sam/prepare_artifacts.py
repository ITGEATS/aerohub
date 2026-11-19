"""Verify local weights and fetch the HF GroundingDINO port into ignored artifacts/.

Never overwrite a verified file; downloads go to a temporary file first and are
checked by SHA256 before being placed. Runs with any Python 3 (standard library
only) so it can execute before `uv sync`.
"""
import hashlib
import json
import os
import sys
import urllib.request
from pathlib import Path

MODEL = Path(__file__).resolve().parent
ARTIFACTS = MODEL / "artifacts"
DINO_DIR = ARTIFACTS / "grounding-dino-base"
MANIFEST = ARTIFACTS / "manifest.json"
REQUIRED_DINO = ("config.json", "model.safetensors", "preprocessor_config.json",
                 "tokenizer_config.json", "tokenizer.json", "vocab.txt")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fetch(url, timeout=60):
    request = urllib.request.Request(url, headers={"User-Agent": "aerohub-grounded-sam"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def download(url, target, timeout=600):
    temporary = target.parent / (target.name + ".download")
    if temporary.exists():
        temporary.unlink()
    request = urllib.request.Request(url, headers={"User-Agent": "aerohub-grounded-sam"})
    with urllib.request.urlopen(request, timeout=timeout) as response, temporary.open("xb") as output:
        while True:
            chunk = response.read(1 << 20)
            if not chunk:
                break
            output.write(chunk)
    temporary.replace(target)


def verify_local_weights(record):
    sam = record["sam"]
    sam_path = MODEL / "weights" / sam["filename"]
    if not sam_path.is_file():
        raise ValueError(f"SAM checkpoint missing: {sam_path}; download {sam['source']} first")
    if sam_path.stat().st_size != sam["bytes"]:
        raise ValueError(f"SAM checkpoint size mismatch: {sam_path}")
    if sha256(sam_path) != sam["sha256"]:
        raise ValueError(f"SAM checkpoint checksum mismatch: {sam_path}")
    print(f"Verified SAM checkpoint {sam_path.name}")
    reference = record["swint_reference"]
    reference_path = MODEL / "weights" / reference["filename"]
    if reference_path.is_file():
        if sha256(reference_path) != reference["sha256"]:
            raise ValueError(f"Reference checkpoint checksum mismatch: {reference_path}")
        print(f"Verified unused backup {reference_path.name} ({reference['status']})")
    else:
        print(f"Reference backup {reference['filename']} not present (optional)")


def prepare_dino(record):
    dino = record["dino"]
    endpoints = [os.environ.get("HF_ENDPOINT")] + dino["endpoints"]
    endpoints = [e for e in dict.fromkeys(endpoints) if e]
    existing = {path.name: path for path in DINO_DIR.glob("*")} if DINO_DIR.is_dir() else {}
    recorded = {}
    if MANIFEST.is_file():
        recorded = {item["path"]: item["sha256"] for item in json.loads(MANIFEST.read_text(encoding="utf-8"))["files"]}
    expected = {}
    tree_error = None
    used_endpoint = endpoints[0]
    for endpoint in endpoints:
        for attempt in range(3):
            try:
                tree = json.loads(fetch(f"{endpoint}/api/models/{dino['repo']}/tree/{dino['revision']}"))
                expected = {}
                for entry in tree:
                    if entry["type"] != "file":
                        continue
                    name = entry["path"]
                    if name.endswith(".safetensors") or name.endswith(".json") or name == "vocab.txt":
                        expected[name] = entry.get("lfs", {}).get("sha256")
                used_endpoint = endpoint
                tree_error = None
                break
            except Exception as error:  # transient 429/timeouts on mirrors
                tree_error = error
                import time as _time
                _time.sleep(5 * (attempt + 1))
        if tree_error is None:
            break
    if tree_error is not None:
        # Fallback: pinned required file list, verified only by local checksums.
        expected = {name: None for name in REQUIRED_DINO}
        expected["special_tokens_map.json"] = None
        print(f"Tree API unreachable ({tree_error}); using fallback file list on {used_endpoint}")
    missing = [name for name in REQUIRED_DINO if name not in expected and name not in existing]
    if missing:
        raise RuntimeError(f"Repository listing lacks required files {missing} on {used_endpoint}")
    manifest_files = []
    for name in sorted(expected):
        target = DINO_DIR / name
        want_sha = dino["model_sha256"] if name == "model.safetensors" else None
        want_sha = want_sha or expected.get(name) or recorded.get(name)
        if target.is_file() and want_sha:
            if sha256(target) == want_sha:
                manifest_files.append({"path": name, "sha256": want_sha})
                continue
            if name not in REQUIRED_DINO:
                target.unlink()  # stale optional file; refetch below
            else:
                raise ValueError(f"Existing {name} checksum mismatch; inspect {target} before retrying")
        elif target.is_file():
            manifest_files.append({"path": name, "sha256": sha256(target)})
            continue
        DINO_DIR.mkdir(parents=True, exist_ok=True)
        url = f"{used_endpoint}/{dino['repo']}/resolve/{dino['revision']}/{name}"
        print(f"Downloading {url}")
        download(url, target)
        got = sha256(target)
        if want_sha and got != want_sha:
            raise ValueError(f"Downloaded {name} checksum mismatch: {got} != {want_sha}")
        manifest_files.append({"path": name, "sha256": got})
    for name in REQUIRED_DINO:
        if not (DINO_DIR / name).is_file():
            raise RuntimeError(f"Required file {name} missing after prepare")
    manifest = {
        "model": "grounded_sam",
        "dino": {"repo": dino["repo"], "revision": dino["revision"], "endpoint": used_endpoint},
        "sam": record["sam"],
        "swint_reference": record["swint_reference"],
        "files": manifest_files,
    }
    ARTIFACTS.mkdir(exist_ok=True)
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared DINO files in {DINO_DIR} via {used_endpoint}")


def main():
    record = json.loads((MODEL / "artifacts.json").read_text(encoding="utf-8"))
    verify_local_weights(record)
    prepare_dino(record)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError) as error:
        print(f"Artifact preparation failed: {error}", file=sys.stderr)
        sys.exit(1)
