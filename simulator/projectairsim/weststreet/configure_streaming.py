#!/usr/bin/env python3
"""Set a bounded, project-local texture pool without disabling streaming.

4096 MiB is the WestStreet 24 GiB GPU profile, not a universal GPU default.
"""
import argparse
import re
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--pool-mib', type=int, default=4096)
    parser.add_argument('--backup', type=Path, required=True)
    args = parser.parse_args()
    if not 512 <= args.pool_mib <= 16384:
        parser.error('--pool-mib must be between 512 and 16384')
    config = args.project.parent / 'Config/DefaultEngine.ini'
    original = config.read_text()
    args.backup.parent.mkdir(parents=True, exist_ok=True)
    if not args.backup.exists():
        shutil.copy2(config, args.backup)
    settings = {'r.Streaming.PoolSize': str(args.pool_mib), 'r.Streaming.LimitPoolSizeToVRAM': '1'}
    lines = original.splitlines()
    result, in_section, found = [], False, False

    def append_settings():
        result.extend(f'{key}={value}' for key, value in settings.items())

    for line in lines:
        if re.match(r'^\s*\[.+\]\s*$', line):
            if in_section:
                append_settings()
            in_section = line.strip().lower() == '[systemsettings]'
            found |= in_section
        if in_section and any(re.match(r'^\s*' + re.escape(key) + r'\s*=', line, re.I) for key in settings):
            continue
        result.append(line)
    if in_section:
        append_settings()
    if not found:
        result.extend(['', '[SystemSettings]'])
        append_settings()
    config.write_text('\n'.join(result) + '\n')
    print(f'Configured WestStreet texture pool: {args.pool_mib} MiB; streaming remains enabled')


if __name__ == '__main__':
    main()
