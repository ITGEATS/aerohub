"""Build the FlowPilot delivery image from a checked, pinned source tree."""
import argparse
from pathlib import Path
import subprocess
import hashlib
import os

ROOT = Path(__file__).resolve().parents[3]


def source_fingerprint():
    raw = subprocess.check_output(['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=ROOT)
    digest = hashlib.sha256()
    for name in sorted(set(raw.split(b'\0'))):
        if not name:
            continue
        path = ROOT / os.fsdecode(name)
        if path.is_file():
            digest.update(name + b'\0' + path.read_bytes())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-image', required=True, help='Verified onboard v0.1.2 image reference/digest')
    parser.add_argument('--uv-image', required=True, help='Pinned Linux uv image supporting the committed lock')
    parser.add_argument('--tag', required=True, help='New component tag; do not overwrite a shared base tag')
    parser.add_argument('--weights-dir', required=True, type=Path, help='Prepared four-file weight context, baked into the image')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.tag == args.base_image or args.tag.split(':')[0].split('/')[-1] in ('onboard_computer', 'onboard-computer', 'flight_board', 'flight-board'):
        parser.error('component tag must not replace a shared base image')
    upstream = ROOT / 'src/tool/navigation/planner/flowpilot/upstream'
    head = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if head != 'f29f1bfee1399c96e5fd384276d5d7a09876286b' or subprocess.check_output(['git', '-C', str(upstream), 'status', '--porcelain'], text=True).strip():
        parser.error('initialize the pinned, clean FlowPilot submodule first')
    if not args.dry_run:
        import importlib.util
        model = ROOT / 'src/basic_function/inference/models/flowpilot'
        spec = importlib.util.spec_from_file_location('flowpilot_weight_verification', model/'weights.py')
        checks = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(checks)
        checks.verify_weights(args.weights_dir, model/'weights.sha256')
        expected = {line.split()[1] for line in (model/'weights.sha256').read_text().splitlines() if line.strip()}
        if {p.name for p in args.weights_dir.iterdir()} != expected:
            parser.error('weight context must contain only the four declared files')
    command = ['docker', 'buildx', 'build', '--load', '--build-context', 'flowpilot_weights=' + str(args.weights_dir.resolve()),
               '--build-arg', 'ONBOARD_IMAGE=' + args.base_image,
               '--build-arg', 'UV_IMAGE=' + args.uv_image, '--tag', args.tag,
               '--label', 'io.aerohub.flowpilot.upstream=' + head,
               '--label', 'io.aerohub.flowpilot.source-sha256=' + source_fingerprint(),
               '--file', str(ROOT / 'deployment/inference/flowpilot/Dockerfile'), str(ROOT)]
    if args.dry_run:
        import json
        print(json.dumps(command))
    else:
        subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
