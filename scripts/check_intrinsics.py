#!/usr/bin/env python3
"""Print the camera intrinsics that swarm_aruco would actually load.

Mirrors ArucoMarker.load_calibration() exactly: same share-directory lookup,
same cv2.FileStorage read, same 'K'/'D' node names. Run it to confirm fx/fy/
cx/cy per robot *before* trusting a pose estimate.

Usage:
    source install/setup.bash
    python3 scripts/check_intrinsics.py             # all serials found
    python3 scripts/check_intrinsics.py 159CKC50070ECX
"""
import os
import sys

import cv2
from ament_index_python.packages import get_package_share_directory


def calibration_root() -> str:
    return os.path.join(
        get_package_share_directory('swarm_bringup'), 'calibration')


def report(root: str, sn: str) -> bool:
    # Same path construction as load_calibration().
    filepath = os.path.join(root, sn, sn + '_cam_cal.yaml')
    print(f'--- {sn}')
    print(f'    path: {filepath}')

    if not os.path.isfile(filepath):
        print('    MISSING -> load_calibration() leaves CameraMatrix = None, '
              'estimatePoseSingleMarkers will raise or return garbage')
        return False

    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    try:
        k = fs.getNode('K').mat()
        d = fs.getNode('D').mat()
    finally:
        fs.release()

    if k is None:
        print("    NO 'K' NODE -> CameraMatrix = None (silent failure)")
        return False

    fx, fy = k[0][0], k[1][1]
    cx, cy = k[0][2], k[1][2]
    print(f'    fx={fx:.3f}  fy={fy:.3f}  cx={cx:.3f}  cy={cy:.3f}')
    # cx/cy sit near the image centre, so they imply the resolution the
    # calibration was captured at. Compare this to the live stream.
    print(f'    implied capture resolution ~= {2 * cx:.0f} x {2 * cy:.0f}')
    if d is not None:
        print(f'    D={[round(float(v), 6) for v in d.ravel()]}')
    return True


def main() -> int:
    root = calibration_root()
    print(f'calibration root: {root}\n')

    if len(sys.argv) > 1:
        serials = sys.argv[1:]
    else:
        serials = sorted(
            n for n in os.listdir(root)
            if os.path.isdir(os.path.join(root, n)) and n != 'beacon')

    ok = all([report(root, sn) for sn in serials])
    print('\nNOTE: this reads the *installed* share directory, not src/.'
          '\nEdits under ros2_swarm/src/swarm_bringup/calibration do not take'
          '\neffect until you rebuild swarm_bringup.')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
