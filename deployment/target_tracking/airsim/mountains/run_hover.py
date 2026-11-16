#!/usr/bin/env python3
"""Dedicated hover-observe entry; the default run.py ground mode is unchanged."""
import sys
from run import main
if __name__=='__main__':
    sys.argv.append('--hover-observe')
    main()
