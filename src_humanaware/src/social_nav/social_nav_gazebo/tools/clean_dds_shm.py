#!/usr/bin/env python3
"""Delete ORPHANED /dev/shm/fastrtps_* files (not opened by any process).

Repeated `kill -9` restarts leave Fast DDS shared-memory files behind; their number
grows with every restart and slows Nav2 start-up until it times out. Files still
used by a running process (e.g. an open RViz) must not be removed, so only files
that appear in no /proc/*/maps or /proc/*/fd are deleted.
"""
import glob
import os

in_use = set()
for proc in glob.glob('/proc/[0-9]*'):
    try:
        with open(proc + '/maps') as f:
            for line in f:
                i = line.find('/dev/shm/fastrtps')
                if i >= 0:
                    in_use.add(line[i:].strip())
    except OSError:
        pass
    try:
        for fd in glob.glob(proc + '/fd/*'):
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if target.startswith('/dev/shm/fastrtps'):
                in_use.add(target)
    except OSError:
        pass

all_files = set(glob.glob('/dev/shm/fastrtps*'))
removed = 0
for path in all_files - in_use:
    try:
        os.unlink(path)
        removed += 1
    except OSError:
        pass
print(f'/dev/shm: {len(all_files)} fastrtps files, {len(in_use)} in use, {removed} orphans removed')
