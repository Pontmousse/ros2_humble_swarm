# RoboMaster Camera Calibration — Cheat Sheet

Native (non-Docker) camera intrinsics calibration via `launch_centralized_swarm.py`.

Chessboard: **9x6 inner corners, 23 mm squares**. 20 photos, one every 6 s (~2 min).

---

## Robot index map

`ROBOT_IDX` is 1-based against these lists in `launch_centralized_swarm.py:22-32`.

| IDX | Name | Serial | Calibration folder exists? |
|-----|------|--------|---------------------------|
| 1 | RM1 | 159CKC50070ECX | yes |
| 2 | RM2 | 159CKC50070E5N | yes |
| 3 | RM3 | 159CG9J0050797 | yes |
| 4 | RM4 | 159CG9V0050HED | yes |
| 5 | RM5 | 159CKCH0070F8S | **NO — must create, see Step 1** |

---

## Step 0 — One-time source fix (`av.AVError`)

PyAV 14.x removed `av.AVError`. The handler at
`ros2_swarm/src/swarm_aruco/swarm_aruco/aruco_detector.py:173` still references it, so *any*
exception inside the decode `try` block gets replaced by an opaque `AttributeError` and the
node dies. This masks the real error — fix it before debugging anything else.

Edit line 173:

```python
        except av.AVError as parse_err:          # before
        except av.FFmpegError as parse_err:      # after
```

Leave line 168 (`av.error.InvalidDataError`) alone — still valid.

**No rebuild needed.** `build/swarm_aruco/swarm_aruco` is a symlink to
`src/swarm_aruco/swarm_aruco` (verified: same inode), so the `build/...` path in tracebacks is the
*same file* you edit in `src/`. Just relaunch.

```bash
# verify the symlink chain yourself if in doubt:
cd ~/projects/ros2_humble_swarm/ros2_swarm
readlink build/swarm_aruco/swarm_aruco
stat -c '%i %n' src/swarm_aruco/swarm_aruco/aruco_detector.py \
                build/swarm_aruco/swarm_aruco/aruco_detector.py   # same inode
```

Rebuild is only required for `setup.py`/entry-point changes, `.msg`/`.srv`, or C++.

---

## Step 1 — Create the output folder (or photos vanish silently)

`take_photo` writes via `get_package_share_directory('swarm_bringup')` → the **install share**
path. `cv2.imwrite` fails **silently** if the directory is missing: you get 20 "Photo N/20 taken"
log lines and zero files on disk.

```bash
cd ~/projects/ros2_humble_swarm/ros2_swarm
SN=159CKCH0070F8S          # change to your robot's serial
mkdir -p src/swarm_bringup/calibration/$SN
mkdir -p install/swarm_bringup/share/swarm_bringup/calibration/$SN
```

Note: for the four pre-existing serials, files inside the install folder are symlinks back into
`src/`, so writes pass straight through. A folder created now is **not** symlinked, so output
stays in `install/` only — copy it to `src/` at the end (Step 6) since only `src/` is under git.

---

## Step 2 — Launch file edits

File: `ros2_swarm/src/swarm_bringup/launch/launch_centralized_swarm.py`

| Line | Change | Why |
|------|--------|-----|
| **397** | `"calibrate_camera": 0,` → `1` | Enables calibration on `landmark_detector`. It is an **int**, not a bool. |
| 457 | **leave at `0`** | `target_detector` shares the same camera + output folder; setting both to `1` makes them overwrite each other's photos. |
| 289 | `ld.add_action(mm)` → `# ld.add_action(mm)` | Kills the Marvelmind `/dev/ttyACM0` retry spam that buries the countdown. Not needed for ArUco. |
| 315 | `ld.add_action(hedge)` → `# ...` (optional, 8-space indent) | `hedgehog_obs` only subscribes to `mm` output; silent anyway. Tidiness only. |

Nothing else needs commenting/uncommenting. Comment only the `ld.add_action(...)` calls, never the
`Node(...)` definitions. `swarm_magnet` is already commented out at line 599.

Launch files **are** symlinked into `install/`, so launch edits are live with **no rebuild**.

---

## Step 3 — Run it

`LAUNCH_FILE` is a **Docker-entrypoint-only** variable — the launch file never reads it. Natively
you pass the file as an argument. Of everything `bootstrap.sh` exports, this launch reads only
`ROBOT_IDX`, `ROBOT_IPS`, `GPIO_LINE`, `RM_DIAGNOSTICS`.

```bash
source /opt/ros/humble/setup.bash
source ~/projects/ros2_humble_swarm/ros2_robomaster/install/setup.bash
source ~/projects/ros2_humble_swarm/ros2_swarm/install/setup.bash

export ROBOT_IDX=5
ros2 launch swarm_bringup launch_centralized_swarm.py
```

Source order matters: `ros2_robomaster` **before** `ros2_swarm` (the launch includes
`robomaster_ros/launch/s1.launch`).

**Calibrate one robot at a time.** The detector nodes are built inside `for i in range(N)`
(line 376), so `calibrate_camera: 1` activates for *every* robot in `ROBOT_IDX` simultaneously.

If SN broadcast discovery (UDP 40927) fails on your network, pin the IP:

```bash
export ROBOT_IPS="RM5=192.168.x.x"
```

---

## Step 4 — Confirm the camera before waving the board

```bash
ros2 topic hz /RM5/camera/image_h264        # expect steady ~30 Hz
```

Silent = driver never connected. Fix that first; no `calibrate_camera` setting helps.

Expected log:

```
Calibration started. Be ready to place (9x6) chessboard for taking photos.
5...  4...  3...  2...  1...
Photo 1/20 taken.
```

Check files are landing **during** the run — if empty after photo 2, abort:

```bash
ls ~/projects/ros2_humble_swarm/ros2_swarm/install/swarm_bringup/share/swarm_bringup/calibration/159CKCH0070F8S/
```

Move the board between every countdown: vary angle, distance, and cover all corners of frame.

---

## Step 5 — Verify the result

Success log: `Calibration Complete!  total error: X` then `Calibration complete. Files loaded.`
`total error` is mean reprojection error in pixels — **under ~1.0 is good**, several pixels means
redo it.

```bash
cd ~/projects/ros2_humble_swarm/ros2_swarm
CAL=install/swarm_bringup/share/swarm_bringup/calibration/159CKCH0070F8S

ls -la $CAL                                    # want 20 jpgs + <SN>_cam_cal.yaml

python3 -c "import cv2;print(cv2.imread('$CAL/h264_photo_1.jpg').shape)"
# MUST be (360, 640, 3) — frameSize is hardcoded (640,360) in aruco.py:91.
# Wrong resolution = silently wrong intrinsics even with low reprojection error.

python3 -c "
import cv2
f=cv2.FileStorage('$CAL/159CKCH0070F8S_cam_cal.yaml',cv2.FILE_STORAGE_READ)
K=f.getNode('K').mat();print(K);print(f.getNode('D').mat())"
```

Sanity: `cx,cy` near image centre (~320, ~180); `fx` ≈ `fy` within a few percent.

Reference values from the existing four robots:

```
159CG9J0050797  fx=182.4 fy=179.1 cx=329.6 cy=184.7
159CG9V0050HED  fx=302.7 fy=301.6 cx=297.4 cy=210.1
159CKC50070E5N  fx=159.1 fy=156.0 cx=312.4 cy=197.5
159CKC50070ECX  fx=198.8 fy=201.8 cx=344.1 cy=189.9
```

(These disagree more than identical cameras should — `fx` spans 159–303 — so treat them as a rough
ballpark, not gospel. Some are probably poor calibrations.)

---

## Step 6 — Restore and persist

1. Set line **397 back to `0`**, or it re-calibrates on every launch instead of detecting markers.
2. Uncomment lines 289 / 315 when you next need Marvelmind.
3. Copy results into git-tracked `src/`:

```bash
cp -r install/swarm_bringup/share/swarm_bringup/calibration/159CKCH0070F8S/* \
      src/swarm_bringup/calibration/159CKCH0070F8S/
```

4. End-to-end check — relaunch and hold up a **DICT_6X6_250** marker:

```bash
ros2 topic echo /RM5/landmarks_unf
```

Non-empty `markers` = working. Also, in normal mode `detect_arucos` writes
`aruco_corner_frame.jpg` every frame into the directory you launched from — open it in an image
viewer for live visual proof.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `cv2.error: (-215) nimages > 0 in calibrateCameraRO` | Zero photos saved — output folder missing, `imwrite` failed silently | Step 1. Verify with `ls` mid-run. |
| `AttributeError: module 'av' has no attribute 'AVError'` | PyAV 14 removed it; handler at `aruco_detector.py:173` | Step 0 + rebuild. Note this **masks the real exception** — fix it, re-run, read the actual error. |
| `marvelmind_obs: OS error (possibly serial port != available)` in a tight loop | No beacon on `/dev/ttyACM0` (you're on a laptop) | Harmless. Comment line 289. |
| Traceback points at `build/swarm_aruco/...` — "am I editing the wrong copy?" | No. `build/swarm_aruco/swarm_aruco` is a **symlink** to `src/` (same inode) | Edit `src/`, relaunch. No rebuild. |
| Source edit has no effect | Stale process, or a genuinely non-symlinked package | Confirm with `readlink build/<pkg>/<pkg>`. If it's a real dir, `colcon build --symlink-install --packages-select <pkg> && source install/setup.bash`. |
| Launch-file edit has no effect | Editing the wrong file | Launch files are symlinked — edit `src/`, no rebuild. Confirm it's `launch_centralized_swarm.py`, not `launch_decentralized_agent.py` (there is no `launch_centralized_agent.py`). |
| No countdown, no photos | No camera stream | `ros2 topic hz /RM5/camera/image_h264`; set `ROBOT_IPS` if discovery fails. |
| 20 photos "taken", zero on disk | Silent `imwrite` failure | Step 1. |
| `total error` several px | Poor board coverage | Redo; vary angle/distance, fill the frame corners. |
| Opaque crash in `pose_estimation` | `load_calibration` doesn't check the file exists — missing yaml leaves `K = None` | Confirm the yaml exists (Step 5). |
| Photos not 640x360 | Stream resolution != hardcoded `frameSize` | Force 360p; otherwise intrinsics are silently wrong. |

---

## Gotcha summary

- `calibrate_camera` is an **int** (`0`/`1`), not a bool.
- `LAUNCH_FILE` does nothing outside Docker.
- `cv2.imwrite` fails **silently** on a missing directory.
- `frameSize` is hardcoded `(640, 360)` — no warning if the stream differs.
- `load_calibration` doesn't validate the file exists.
- Both launch files and `swarm_aruco` Python are symlinked from `build`/`install` back to `src/` —
  edits are live, no rebuild. A `build/...` path in a traceback is not a stale copy.
- Output goes to **install share**, not `src/`, for newly created folders.
