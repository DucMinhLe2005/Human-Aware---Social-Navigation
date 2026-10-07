#!/usr/bin/env python3
"""Export the YOLO pose model to ONNX for CPU inference.

Usage: python3 export_onnx.py <model.pt> [output_dir=weights]
then `colcon build --packages-select social_nav_perception`.

The model is exported at 256x320 (height, width), not 640x640: at the same input
size the ONNX model ran at ~33 ms per frame on an Intel i3-6100U.
"""
import shutil
import sys
from pathlib import Path

from ultralytics import YOLO

IMGSZ = (256, 320)  # (height, width)


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    source = Path(sys.argv[1])
    out = Path(sys.argv[2] if len(sys.argv) > 2 else 'weights')
    out.mkdir(parents=True, exist_ok=True)
    path = YOLO(str(source)).export(format='onnx', imgsz=IMGSZ, simplify=True, dynamic=False)
    destination = out / f'{source.stem}.onnx'
    shutil.copy(path, destination)
    print(f'-> {destination} ({destination.stat().st_size / 1e6:.1f} MB)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
