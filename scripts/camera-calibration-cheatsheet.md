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

## Step 2 — No launch file edits needed

**This used to require four hand edits that had to be reverted afterwards (and silently
re-calibrated on every launch if you forgot). It is now a single environment variable.**

```bash
export CALIBRATE_CAMERA=1
```

That one variable does all of the following in `launch_centralized_swarm.py`:

| Effect | Why |
|---|---|
| `landmark_detector` gets `calibrate_camera: 1` | Enters chessboard-capture mode. |
| `target_detector` stays at `0` | Shares the same camera **and the same output folder** — two capturers overwrite each other's photos. Deliberately not wired. |
| `marvelmind_obs` not launched | Its `/dev/ttyACM0` retry loop buries the photo countdown. |
| `hedgehog_obs` not launched | Only consumes `mm` output; silent anyway. |

Because it keys off the environment, **an unset variable is the safe default** — there is nothing to
revert. Both toggles print a confirmation line at launch:

```
CALIBRATE_CAMERA=1 -- landmark_detector will capture chessboard photos instead of detecting markers.
CALIBRATE_CAMERA=1 -- skipping marvelmind_obs (log spam would bury the photo countdown).
```

**Calibrate one robot at a time.** The detector nodes are built in a loop over `ROBOT_IDX`, so this
activates for every selected robot simultaneously.

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
export CALIBRATE_CAMERA=1
ros2 launch swarm_bringup launch_centralized_swarm.py
```

Source order matters: `ros2_robomaster` **before** `ros2_swarm` (the launch includes
`robomaster_ros/launch/s1.launch`).

### Env vars this launch reads

| Var | Default | Meaning |
|---|---|---|
| `ROBOT_IDX` | *required* | 1-based robot selection, e.g. `2` or `1,2`. |
| `CALIBRATE_CAMERA` | `0` | `1` = capture chessboard photos (Step 2). |
| `STRICT_CALIBRATION` | `1` | `0` = downgrade implausible-intrinsics rejection to a warning. |
| `ROBOT_IPS` | discovery | Pin IPs, e.g. `RM5=192.168.50.155`. |
| `GPIO_LINE`, `RM_DIAGNOSTICS` | — | Unrelated to calibration. |

`LAUNCH_FILE` does nothing outside Docker.

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

Success log: `Calibration Complete!` → `RMS reprojection error: X px` → `Calibration complete.
Files loaded.`

**Under ~1.0 px is good. Over ~2 px, redo it** — the intrinsics may still pass the plausibility
check while being too imprecise to trust.

> The old `total error: X` line was **understated by a factor of sqrt(54) ≈ 7.35** — it divided the
> L2 norm by the number of corners instead of its square root. The "under 1.0" rule was therefore
> accepting anything under ~7 px RMS, which is how every stored calibration looked fine while
> producing visibly wrong focal lengths. Fixed; the printed figure now matches
> `cv2.calibrateCamera`'s own return value.

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

**This is now checked automatically.** `ArucoMarker._sanity_check_intrinsics()` runs both after
`calibrateCamera` (before writing, so a bad run cannot clobber a good file) and on every
`load_calibration()`. Inspect any time with:

```bash
source /opt/ros/humble/setup.bash && source ros2_swarm/install/setup.bash
python3 scripts/check_intrinsics.py            # all robots
python3 scripts/check_intrinsics.py 159CKC50070E5N
```

### The reference value

The driver publishes its own factory intrinsics on `camera/camera_info`. At 640×360 that is
**fx ≈ 317.0, fy ≈ 316.3, cx ≈ 317.3, cy ≈ 182.6** (`rm_camera_calibration_360.yaml`). Same lens on
every robot, so this is the number a good self-calibration must land near. `fx` scales linearly with
image width: ×1.5 at 960, ×2 at 1280.

Thresholds in `aruco.py`: `FX_TOLERANCE = 0.25` (±25% of reference), `fx`/`fy` must agree within
10%, `cx`/`cy` within 15% of the image centre, and `MIN_CALIBRATION_VIEWS = 8` photos with a
detected board.

### Status of the stored calibrations (as of 2026-10-02)

| SN | fx | vs reference 317 | verdict |
|---|---|---|---|
| 159CG9V0050HED | 302.7 | 0.96 | **PASS** |
| 159CKC50070ECX | 198.8 | 0.63 | REJECTED — distances ~1.59× too large |
| 159CG9J0050797 | 182.4 | 0.58 | REJECTED — ~1.74× |
| 159CKCH0070F8S | 174.1 | 0.55 | REJECTED — ~1.82× |
| 159CKC50070E5N | 159.1 | 0.50 | REJECTED — ~1.99× |
| 159CKC50070EVP | — | — | file missing entirely |

Five of six need re-shooting. The likely cause was a bug at `aruco.py:143`: `cornerSubPix` was
computed into `corners2` and then **thrown away**, with the raw `corners` appended to `imgpoints`
instead. Sub-pixel noise biases focal length much more than principal point — which is exactly the
observed signature (`fx` wrong, `cx`/`cy` fine). Fixed; re-calibrations from now on use the refined
corners.

### Running before you have re-calibrated

`strict_calibration` (int, default `1`) downgrades rejection to a warning:

```bash
ros2 launch swarm_bringup launch_centralized_swarm.py strict_calibration:=0
```

Poses will be mis-scaled — only for bring-up, never for recorded runs.

### Note on `aruco_scale`

`ArucoMarker.__init__` takes `scale=(1.5, 0.80)`, applied to `tvecs` in `pose_estimation()`. Those
asymmetric factors look like empirical compensation for the bad `fx`. **After re-calibrating,
re-check them** — with correct intrinsics they should both be near `1.0`. Leaving `1.5/0.8` on top
of a corrected `fx` will double-count the correction.

---

## Step 6 — Restore and persist

1. `unset CALIBRATE_CAMERA` — or just open a new shell. Marvelmind and `hedgehog_obs` come back
   automatically; there are no commented-out lines to restore.
2. Verify the new intrinsics pass in strict mode:

```bash
python3 scripts/check_intrinsics.py <SN>
```

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
| `FileNotFoundError: no calibration file at ...` at startup | No yaml for that serial. Previously this was silent (`K = None`) until a crash deep in `pose_estimation` | Create the calibration, or check `serial_number`. |
| `RuntimeError: implausible loaded intrinsics` at startup | The sanity check rejected a stored calibration | Re-calibrate. To run anyway: `strict_calibration:=0`. |
| `RuntimeError: Only N of M photos had a detectable chessboard` | Fewer than `MIN_CALIBRATION_VIEWS` usable photos | Re-shoot; board must be fully visible and well lit. |
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
