"""Download and verify pinned FlowPilot weights into the ignored artifacts/ directory.

Standard library only, so this runs before the model venv is synced. weights.sha256
is the single authority for hashes; URL and revision details live here.
"""
import hashlib
import shutil
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path

MODEL = Path(__file__).resolve().parent
ARTIFACTS = MODEL / "artifacts"
CHECKSUMS = MODEL / "weights.sha256"
UPSTREAM_REVISION = "f29f1bfee1399c96e5fd384276d5d7a09876286b"
FLOWPILOT_REVISION = "9e60a75951c50bd3e7fc359634e023be66535fa9"
WAN_REVISION = "921dbaf3f1674a56f47e83fb80a34bac8a8f203e"
FLOWPILOT_CKPT = "https://huggingface.co/YuDinging/FlowPilot/resolve/" + FLOWPILOT_REVISION + "/ckpt"

SOURCES = {
    "action_stats.pt": FLOWPILOT_CKPT + "/action_stats.pt",
    "coeff_stats.pt": FLOWPILOT_CKPT + "/coeff_stats.pt",
    "ckpt_final.pt": FLOWPILOT_CKPT + "/phase2/ckpt_final.pt",
    "Wan2.2_VAE.pth": "https://huggingface.co/Wan-AI/Wan2.2-TI2V-5B/resolve/" + WAN_REVISION + "/Wan2.2_VAE.pth",
}


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def expected_records():
    records = []
    for line in CHECKSUMS.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 2:
            raise ValueError("weights.sha256 must contain '<sha256>  <name>' lines")
        records.append((parts[1], parts[0]))
    if sorted(name for name, _ in records) != sorted(SOURCES):
        raise ValueError("weights.sha256 and SOURCES disagree; refusing to prepare")
    return records


def download(url, target, expected):
    with tempfile.NamedTemporaryFile(dir=ARTIFACTS, prefix=".weights.", delete=False) as handle:
        temporary = Path(handle.name)
    try:
        with urllib.request.urlopen(url, timeout=120) as response:
            with temporary.open("wb") as stream:
                shutil.copyfileobj(response, stream)
        if digest(temporary) != expected:
            raise ValueError("downloaded file does not match weights.sha256: " + target.name)
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    for name, expected in expected_records():
        target = ARTIFACTS / name
        if not target.exists():
            print("Downloading", name, "from", SOURCES[name], flush=True)
            download(SOURCES[name], target, expected)
        actual = digest(target)
        if actual != expected:
            raise ValueError("existing artifact differs from weights.sha256; preserving file: " + str(target))
        print("{}: OK ({} bytes)".format(name, target.stat().st_size))
    print("FlowPilot weights verified for upstream", UPSTREAM_REVISION)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, urllib.error.URLError) as error:
        print("FlowPilot weight preparation failed:", error, file=sys.stderr)
        sys.exit(1)
