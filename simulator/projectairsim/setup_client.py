#!/usr/bin/env python3
"""Install the native SDK from configured development resources."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

# Support direct invocation from the repository root or another directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulator.manager.resources import resolve_resource


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-resource', default='projectairsim_source')
    parser.add_argument('--python-resource', default='projectairsim_python')
    parser.add_argument('--check', action='store_true', help='Check resources without installing')
    args = parser.parse_args()
    sdk = resolve_resource(args.source_resource) / 'client/python/projectairsim'
    python = resolve_resource(args.python_resource)
    if not (sdk / 'pyproject.toml').is_file():
        parser.error(f'Native SDK pyproject.toml not found: {sdk}')
    if not python.is_file():
        parser.error(f'Create the configured Python virtual environment first: {python}')
    print(f'SDK source: {sdk}\nPython environment: {python}', flush=True)

    if not args.check:
        if not (python.parent.parent / 'pyvenv.cfg').is_file():
            parser.error(f'Configured Python must belong to a virtual environment: {python}')
        root = Path(__file__).resolve().parents[2]
        environment = dict(os.environ, UV_PROJECT_ENVIRONMENT=str(python.parent.parent))
        subprocess.run([
            'uv', 'sync', '--project', str(root), '--locked', '--inexact',
            '--extra', 'projectairsim', '--no-install-package', 'projectairsim',
        ], env=environment, check=True)
        subprocess.run([
            'uv', 'pip', 'install', '--python', str(python), '--no-deps',
            '--no-build-isolation', '--editable', str(sdk),
        ], check=True)
        subprocess.run(['uv', 'pip', 'check', '--python', str(python)], check=True)


if __name__ == '__main__':
    main()
