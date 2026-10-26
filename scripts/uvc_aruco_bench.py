#!/usr/bin/env python3
"""Benchmark ArUco detection off a UVC camera (e-con See3CAM_CU30) on a Pi.

Standalone: needs only cv2 + numpy, so it runs on a bare Pi with no ROS and no
workspace build. Reports per-stage timings, achieved fps, and CPU/thermal
headroom so "can the Pi bear it" has a number attached.

    ./uvc_aruco_bench.py --probe
    ./uvc_aruco_bench.py --size 1280x720 --fourcc MJPG --path gray
    ./uvc_aruco_bench.py --synthetic --size 1920x1080
"""

import argparse
import fcntl
import os
import struct
import subprocess
import sys
import time

import cv2
import numpy as np

ARUCO_DICTS = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}

CLK_TCK = os.sysconf("SC_CLK_TCK")


def read_proc_times():
    """(process_cpu_seconds, system_busy_seconds, system_total_seconds)."""
    with open("/proc/self/stat") as fh:
        fields = fh.read().rsplit(") ", 1)[1].split()
    proc = (int(fields[11]) + int(fields[12])) / CLK_TCK

    with open("/proc/stat") as fh:
        cpu = [int(v) for v in fh.readline().split()[1:]]
    total = sum(cpu)
    idle = cpu[3] + cpu[4]
    return proc, (total - idle) / CLK_TCK, total / CLK_TCK


def read_thermals():
    """Pi SoC temperature in C and the throttling bitmask, when available."""
    temp = None
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as fh:
            temp = int(fh.read().strip()) / 1000.0
    except OSError:
        pass

    throttled = None
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=5
        )
        if out.returncode == 0:
            throttled = int(out.stdout.strip().split("=")[1], 16)
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        pass
    return temp, throttled


def describe_throttling(mask):
    # Bit meanings from the Raspberry Pi firmware docs. Low half = happening
    # now, high half = has happened since boot.
    now = [
        (0, "under-voltage"),
        (1, "arm frequency capped"),
        (2, "currently throttled"),
        (3, "soft temperature limit"),
    ]
    ever = [
        (16, "under-voltage"),
        (17, "arm frequency capped"),
        (18, "throttled"),
        (19, "soft temperature limit"),
    ]
    active = [label for bit, label in now if mask & (1 << bit)]
    history = [label for bit, label in ever if mask & (1 << bit)]
    parts = []
    if active:
        parts.append("NOW: " + ", ".join(active))
    if history:
        parts.append("since boot: " + ", ".join(history))
    return "; ".join(parts) if parts else "clean"


# V4L2 enumeration via raw ioctl. Neither the Pi host nor the robot container
# ships v4l-utils, and OpenCV's negotiation lies by silently substituting modes,
# so ask the driver directly. Request codes are _IOWR('V', nr, sizeof(struct)).
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_TYPE_DISCRETE = 1
VIDIOC_ENUM_FMT = 0xC0405602           # nr=2,  struct v4l2_fmtdesc      (64 B)
VIDIOC_ENUM_FRAMESIZES = 0xC02C564A    # nr=74, struct v4l2_frmsizeenum  (44 B)
VIDIOC_ENUM_FRAMEINTERVALS = 0xC034564B  # nr=75, struct v4l2_frmivalenum (52 B)


def _fourcc_str(value):
    return "".join(chr((value >> (8 * i)) & 0xFF) for i in range(4))


def _enum(fd, request, size, prefix, type_offset=None):
    """Yield successive ioctl results until the driver reports EINVAL."""
    for index in range(128):
        buf = bytearray(size)
        struct.pack_into(f"{len(prefix) + 1}I", buf, 0, index, *prefix)
        try:
            fcntl.ioctl(fd, request, buf, True)
        except OSError:
            return
        if type_offset is not None:
            kind, = struct.unpack_from("I", buf, type_offset)
            if kind != V4L2_TYPE_DISCRETE:
                yield buf, kind
                return
            yield buf, kind
        else:
            yield buf, None


def probe_device(device):
    """Dump the camera's real V4L2 capabilities rather than guessing."""
    print(f"=== v4l2 capabilities for {device} ===")
    try:
        fd = os.open(device, os.O_RDWR)
    except OSError as err:
        print(f"cannot open {device}: {err}")
        return

    try:
        formats = []
        for buf, _ in _enum(fd, VIDIOC_ENUM_FMT, 64, (V4L2_BUF_TYPE_VIDEO_CAPTURE,)):
            desc = bytes(buf[12:44]).split(b"\0")[0].decode("ascii", "replace")
            pixfmt, = struct.unpack_from("I", buf, 44)
            formats.append((pixfmt, desc))

        if not formats:
            print("driver enumerated no capture formats "
                  "(is this a metadata-only node?)")
            return

        for pixfmt, desc in formats:
            print(f"\n[{_fourcc_str(pixfmt)}] {desc}")
            for buf, kind in _enum(fd, VIDIOC_ENUM_FRAMESIZES, 44,
                                   (pixfmt,), type_offset=8):
                if kind != V4L2_TYPE_DISCRETE:
                    lo_w, hi_w, step_w, lo_h, hi_h, step_h = struct.unpack_from(
                        "6I", buf, 12)
                    print(f"  stepwise {lo_w}-{hi_w} step {step_w} x "
                          f"{lo_h}-{hi_h} step {step_h}")
                    continue
                width, height = struct.unpack_from("II", buf, 12)
                rates = []
                for ibuf, ikind in _enum(fd, VIDIOC_ENUM_FRAMEINTERVALS, 52,
                                         (pixfmt, width, height), type_offset=16):
                    if ikind != V4L2_TYPE_DISCRETE:
                        break
                    num, den = struct.unpack_from("II", ibuf, 20)
                    if num:
                        rates.append(den / num)
                shown = ", ".join(f"{r:g}" for r in rates) or "?"
                print(f"  {width:>5} x {height:<5}  {shown} fps")
    finally:
        os.close(fd)


def diagnose(args, detector):
    """Dump every plausible reading of the raw buffer, plus a colour reference.

    Zero detections has two very different causes -- no marker in front of the
    lens, or a correctly captured frame that we are unpacking wrongly -- and
    they are indistinguishable from the counters alone. Writing the images
    settles it in one run.
    """
    def grab(convert_rgb):
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise SystemExit(f"could not open {args.device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1 if convert_rgb else 0)
        frame = None
        for _ in range(15):  # let auto-exposure settle
            ok, got = cap.read()
            if ok:
                frame = got
        cap.release()
        return frame

    def report(name, gray):
        if gray is None:
            return
        corners, ids, _ = detector.detectMarkers(gray)
        n = 0 if ids is None else len(ids)
        path = f"{args.diag}_{name}.png"
        cv2.imwrite(path, gray)
        print(f"  {path:<28} {gray.shape} mean={gray.mean():6.1f} "
              f"std={gray.std():5.1f}  markers={n}"
              + (f"  ids={sorted(int(i) for i in ids.flatten())}" if n else ""))

    print(f"=== diagnostics: {args.fourcc} {args.width}x{args.height} ===")

    raw = grab(convert_rgb=False)
    if raw is None:
        print("no raw frame captured")
    else:
        print(f"raw buffer shape={raw.shape} dtype={raw.dtype} "
              f"({raw.nbytes} bytes, expected {args.width * args.height * 2} "
              f"for packed 4:2:2)")
        if args.fourcc == "MJPG":
            report("mjpg_luma", cv2.imdecode(raw.reshape(-1), cv2.IMREAD_GRAYSCALE))
        else:
            flat = raw.reshape(args.height, -1)
            # Both byte phases: whichever is the real luma plane will have
            # structure, the other will look flat and featureless.
            report("offset0_YUYV", np.ascontiguousarray(flat[:, 0::2]))
            report("offset1_UYVY", np.ascontiguousarray(flat[:, 1::2]))

    colour = grab(convert_rgb=True)
    if colour is not None and colour.ndim == 3:
        cv2.imwrite(f"{args.diag}_bgr.png", colour)
        print(f"  {args.diag}_bgr.png{'':<13} {colour.shape} "
              f"(OpenCV's own conversion -- ground truth for what is in frame)")
        report("bgr2gray", cv2.cvtColor(colour, cv2.COLOR_BGR2GRAY))


def scan_dictionaries(args, grab_frames):
    """Try every predefined dictionary against live frames.

    "It is a 5x5 marker" does not identify a dictionary -- DICT_5X5_50/100/250/
    1000 are four different codebooks and a tag from the wrong one simply never
    decodes. Scanning removes the guesswork, and a total miss across all
    seventeen is itself informative: it means focus, scale or framing, not the
    dictionary.
    """
    frames = grab_frames()
    if not frames:
        print("no frames captured")
        return
    print(f"scanning {len(ARUCO_DICTS)} dictionaries over {len(frames)} frames "
          f"with default (most sensitive) parameters\n")

    any_hit = False
    for name in sorted(ARUCO_DICTS):
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[name]),
            cv2.aruco.DetectorParameters())
        hits, found = 0, set()
        for gray in frames:
            _, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                hits += 1
                found.update(int(i) for i in ids.flatten())
        if hits:
            any_hit = True
            print(f"  {name:<22} {hits}/{len(frames)} frames   "
                  f"ids={sorted(found)}")
    if not any_hit:
        print("  no dictionary matched anything.\n"
              "  The tag is not the problem -- look at focus, marker size in "
              "frame, glare, or whether it is in view at all.")


def serve_stream(args, detector, port, bind):
    """Stream annotated frames as MJPEG over HTTP.

    The Pi is headless and the lens focuses by hand, so a still dumped after the
    fact is a poor feedback loop -- you turn the ring, wait, copy a file, look.
    A browser view closes that loop. The focus number is a Laplacian variance:
    it is not calibrated to anything, you just turn the ring until it peaks.
    """
    import http.server
    import threading

    latest = {"jpeg": None}
    lock = threading.Lock()
    stop = threading.Event()

    def capture_loop():
        cap = open_camera(args)
        ema_fps, prev = 0.0, time.perf_counter()
        while not stop.is_set():
            ok, raw = cap.read()
            if not ok:
                continue
            gray = to_gray(raw, args)
            t0 = time.perf_counter()
            corners, ids, _ = detector.detectMarkers(gray)
            detect_ms = (time.perf_counter() - t0) * 1e3

            now = time.perf_counter()
            dt = now - prev
            prev = now
            if dt > 0:
                ema_fps = 0.9 * ema_fps + 0.1 * (1.0 / dt) if ema_fps else 1.0 / dt

            focus = cv2.Laplacian(gray, cv2.CV_64F).var()
            canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(canvas, corners, ids)
            found = "none" if ids is None else ",".join(
                str(int(i)) for i in ids.flatten())
            for i, line in enumerate((
                    f"{args.width}x{args.height} {args.fourcc}  {ema_fps:4.1f} fps",
                    f"detect {detect_ms:5.1f} ms   ids: {found}",
                    f"focus {focus:7.1f}  (turn the lens until this peaks)")):
                cv2.putText(canvas, line, (8, 28 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1,
                            cv2.LINE_AA)

            ok, buf = cv2.imencode(".jpg", canvas,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with lock:
                    latest["jpeg"] = buf.tobytes()
        cap.release()

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path not in ("/", "/stream.mjpg"):
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while not stop.is_set():
                    with lock:
                        jpeg = latest["jpeg"]
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " +
                                     str(len(jpeg)).encode() + b"\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(1.0 / max(1, args.fps))
            except (BrokenPipeError, ConnectionResetError):
                pass

    worker = threading.Thread(target=capture_loop, daemon=True)
    worker.start()

    server = http.server.ThreadingHTTPServer((bind, port), Handler)
    print(f"serving MJPEG on http://{bind}:{port}/  (Ctrl-C to stop)")
    print("  unauthenticated, and reachable by anything on this LAN")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        stop.set()
        server.server_close()
        worker.join(timeout=2.0)


def open_camera(args):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"could not open {args.device}")

    # Order matters: the pixel format has to be selected before the frame size,
    # otherwise the driver negotiates the size against the default format and
    # silently keeps a resolution the requested format cannot deliver.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if args.path == "gray":
        # Hand us the raw payload. For YUYV that lets us slice the luma plane
        # straight out (ArUco only ever needs gray), and for MJPG it lets
        # imdecode do a luma-only JPEG decode. Both skip a full-frame colour
        # conversion, which is the single biggest CPU win on a Pi.
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

    got = "".join(
        chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> (8 * i)) & 0xFF) for i in range(4)
    )
    actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
              int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(f"requested {args.width}x{args.height} {args.fourcc} @{args.fps} -> "
          f"negotiated {actual[0]}x{actual[1]} {got} @{cap.get(cv2.CAP_PROP_FPS):g}")
    if got != args.fourcc or actual != (args.width, args.height):
        print("  WARNING: driver substituted a different mode. Timings below "
              "describe what you got, not what you asked for.")
    return cap


# Byte position of luma within each 2-byte packed-422 pixel. YUYV is Y0 U Y1 V
# so luma sits on even bytes; UYVY is U Y0 V Y1 so it sits on odd ones. The
# See3CAM_CU30 emits UYVY, and reading the wrong offset silently yields a
# chroma plane that looks like plausible grey mush and detects nothing.
LUMA_OFFSET = {"YUYV": 0, "UYVY": 1}


def to_gray(frame, args):
    """Normalise whatever the capture handed back into a single-channel image."""
    if args.path == "bgr":
        return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if args.fourcc == "MJPG":
        return cv2.imdecode(frame.reshape(-1), cv2.IMREAD_GRAYSCALE)

    # Slice the luma plane straight out of the packed 4:2:2 buffer -- ArUco only
    # ever needs grey, so this replaces a full-frame colour conversion (or a
    # JPEG decode) with a strided copy.
    #
    # The raw buffer comes back as (H, W, 2), (H, W*2) or flat (1, H*W*2)
    # depending on build and driver -- this camera gives the flat form. Slicing
    # that directly yields a 1-pixel-tall image that still has the right pixel
    # count, so it benchmarks plausibly while detecting nothing. Reshape first.
    off = LUMA_OFFSET[args.fourcc]
    return np.ascontiguousarray(frame.reshape(args.height, -1)[:, off::2])


def make_synthetic(width, height, dictionary, n_markers, noise, rng):
    """A frame with real markers plus a little sensor-like noise.

    Detector cost is driven by how many candidate quads adaptive thresholding
    throws up, so image content matters enormously: measured single-threaded at
    1280x720, a clean render costs 4.7 ms, a real photo 10 ms, and the same
    render with sigma=8 noise 136 ms. Keep `noise` small -- synthetic mode is a
    rough proxy, and `--image` with a real still is the trustworthy offline run.
    """
    img = np.full((height, width), 190, np.uint8)
    size = max(48, min(width, height) // 8)
    for i in range(n_markers):
        tag = cv2.aruco.generateImageMarker(dictionary, i, size)
        x = rng.integers(0, max(1, width - size))
        y = rng.integers(0, max(1, height - size))
        img[y:y + size, x:x + size] = tag
    if noise > 0:
        img = np.clip(img.astype(np.int16) + rng.normal(0, noise, (height, width)),
                      0, 255).astype(np.uint8)
    return img


def summarise(name, samples):
    if not samples:
        return f"  {name:<10} (no samples)"
    arr = np.array(samples)
    return (f"  {name:<10} mean {arr.mean():6.2f}  p50 {np.percentile(arr, 50):6.2f}  "
            f"p90 {np.percentile(arr, 90):6.2f}  max {arr.max():6.2f}  ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--size", default="1280x720", help="WIDTHxHEIGHT")
    parser.add_argument("--fourcc", default="MJPG",
                        choices=["MJPG", "YUYV", "UYVY"])
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--path", default="gray", choices=["gray", "bgr"],
                        help="gray = luma-only capture (fast); bgr = full colour "
                             "conversion then cvtColor, matching the current "
                             "swarm_aruco behaviour")
    parser.add_argument("--dict", dest="dict_name", default="DICT_5X5_100",
                        choices=sorted(ARUCO_DICTS))
    parser.add_argument("--thresh-win", default="3:23:10", metavar="MIN:MAX:STEP",
                        help="adaptive threshold window sweep. The default "
                             "3:23:10 runs three full passes over the image and "
                             "dominates detector cost; 7:7:10 runs one and "
                             "measured ~3x faster. Narrowing it trades "
                             "robustness to lighting and marker scale for speed.")
    parser.add_argument("--min-perim", type=float, default=0.03,
                        help="minMarkerPerimeterRate; raising it discards small "
                             "candidates, which bounds how far away a marker can "
                             "still be seen")
    parser.add_argument("--marker-length", type=float, default=0.025,
                        help="marker side in metres, for solvePnP")
    parser.add_argument("--no-pose", action="store_true",
                        help="detect only, skip pose estimation")
    parser.add_argument("--downscale", type=int, default=1,
                        help="detect on a 1/N resized image")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--warmup", type=float, default=2.0,
                        help="discard this many seconds before measuring")
    parser.add_argument("--threads", type=int, default=-1,
                        help="cv2.setNumThreads value; -1 leaves OpenCV's default")
    parser.add_argument("--synthetic", action="store_true",
                        help="no camera: generate frames, isolating detector cost")
    parser.add_argument("--synthetic-markers", type=int, default=4)
    parser.add_argument("--noise", type=float, default=2.0,
                        help="synthetic sensor noise sigma; detector cost is "
                             "very sensitive to this, so keep it small")
    parser.add_argument("--image",
                        help="no camera: replay this still, resized to --size. "
                             "A real photo of the actual scene is the most "
                             "trustworthy offline estimate of detector cost.")
    parser.add_argument("--probe", action="store_true",
                        help="dump V4L2 capabilities and exit")
    parser.add_argument("--save", metavar="PATH",
                        help="write the frame that had the most markers, with "
                             "detections drawn on it. Writes the last frame "
                             "instead when nothing was ever detected, which is "
                             "exactly the case you need to look at.")
    parser.add_argument("--scan-dicts", action="store_true",
                        help="try every predefined dictionary against live "
                             "frames and report which ones match, then exit")
    parser.add_argument("--serve", type=int, metavar="PORT", nargs="?",
                        const=8080,
                        help="stream annotated live frames as MJPEG over HTTP "
                             "on PORT (default 8080) and never return. Open it "
                             "in a browser to aim and focus the lens.")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="interface for --serve (default all)")
    parser.add_argument("--diag", metavar="PREFIX",
                        help="capture a few frames and dump every plausible "
                             "interpretation of the raw buffer as an image, "
                             "then exit. Use when detection returns zero and "
                             "you need to see what the camera actually sends.")
    args = parser.parse_args()

    if args.probe:
        probe_device(args.device)
        return 0

    args.width, args.height = (int(v) for v in args.size.lower().split("x"))
    if args.threads >= 0:
        cv2.setNumThreads(args.threads)

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[args.dict_name])
    params = cv2.aruco.DetectorParameters()
    win_min, win_max, win_step = (int(v) for v in args.thresh_win.split(":"))
    params.adaptiveThreshWinSizeMin = win_min
    params.adaptiveThreshWinSizeMax = win_max
    params.adaptiveThreshWinSizeStep = win_step
    params.minMarkerPerimeterRate = args.min_perim
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    passes = len(range(win_min, win_max + 1, win_step))

    # A plausible pinhole guess. Pose cost does not depend on the values being
    # correct, only on the geometry being solvable, so an uncalibrated run still
    # gives a valid CPU number.
    focal = args.width
    camera_matrix = np.array([[focal, 0, args.width / 2],
                              [0, focal, args.height / 2],
                              [0, 0, 1]], np.float32)
    dist_coeffs = np.zeros(5, np.float32)
    half = args.marker_length / 2
    marker_points = np.array([[-half, half, 0], [half, half, 0],
                              [half, -half, 0], [-half, -half, 0]], np.float32)

    if args.diag:
        diagnose(args, detector)
        return 0

    if args.serve:
        serve_stream(args, detector, args.serve, args.bind)
        return 0

    if args.scan_dicts:
        def grab_frames(count=12):
            if args.image:
                still = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
                return [cv2.resize(still, (args.width, args.height))] if still is not None else []
            cap = open_camera(args)
            out = []
            for _ in range(count + 8):  # discard the auto-exposure ramp
                ok, raw = cap.read()
                if ok:
                    out.append(to_gray(raw, args))
            cap.release()
            saved = out[-count:]
            if saved and args.save:
                cv2.imwrite(args.save, saved[-1])
                print(f"wrote {args.save}")
            return saved

        scan_dictionaries(args, grab_frames)
        return 0

    rng = np.random.default_rng(0)
    offline = args.synthetic or args.image
    still = None
    if args.image:
        still = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if still is None:
            raise SystemExit(f"could not read {args.image}")
        still = cv2.resize(still, (args.width, args.height))
        print(f"replaying {args.image} at {args.width}x{args.height}")
    elif args.synthetic:
        print(f"synthetic frames, noise sigma={args.noise} -- a proxy only; "
              f"prefer --image with a real photo")

    cap = None if offline else open_camera(args)

    print(f"OpenCV {cv2.__version__}, threads={cv2.getNumThreads()}, "
          f"cores={os.cpu_count()}, dict={args.dict_name}, "
          f"path={args.path}, downscale={args.downscale}")
    print(f"thresh-win {args.thresh_win} ({passes} pass"
          f"{'' if passes == 1 else 'es'}), min-perim {args.min_perim}")

    t_cap, t_gray, t_detect, t_pose, t_loop = [], [], [], [], []
    marker_counts = []
    read_failures = 0
    id_hits = {}
    best = (-1, None, None, None)  # count, gray, corners, ids

    measuring = False
    start = time.perf_counter()
    deadline = start + args.warmup + args.seconds
    p0 = s_busy0 = s_tot0 = 0.0
    t_measure_start = 0.0
    frames = 0

    while time.perf_counter() < deadline:
        if not measuring and time.perf_counter() - start >= args.warmup:
            measuring = True
            p0, s_busy0, s_tot0 = read_proc_times()
            t_measure_start = time.perf_counter()

        loop_t0 = time.perf_counter()

        t0 = time.perf_counter()
        if still is not None:
            raw, ok = still, True
        elif args.synthetic:
            raw = make_synthetic(args.width, args.height, dictionary,
                                 args.synthetic_markers, args.noise, rng)
            ok = True
        else:
            ok, raw = cap.read()
        t1 = time.perf_counter()

        if not ok or raw is None:
            read_failures += 1
            continue

        gray = raw if offline else to_gray(raw, args)
        if gray is None:
            read_failures += 1
            continue
        if args.downscale > 1:
            gray = cv2.resize(gray, None, fx=1 / args.downscale, fy=1 / args.downscale,
                              interpolation=cv2.INTER_AREA)
        t2 = time.perf_counter()

        corners, ids, _ = detector.detectMarkers(gray)
        t3 = time.perf_counter()

        if not args.no_pose:
            for c in corners:
                cv2.solvePnP(marker_points, c, camera_matrix, dist_coeffs,
                             False, cv2.SOLVEPNP_IPPE_SQUARE)
        t4 = time.perf_counter()

        if measuring:
            frames += 1
            t_cap.append((t1 - t0) * 1e3)
            t_gray.append((t2 - t1) * 1e3)
            t_detect.append((t3 - t2) * 1e3)
            t_pose.append((t4 - t3) * 1e3)
            t_loop.append((t4 - loop_t0) * 1e3)
            found = 0 if ids is None else len(ids)
            marker_counts.append(found)
            if found:
                for marker_id in ids.flatten():
                    id_hits[int(marker_id)] = id_hits.get(int(marker_id), 0) + 1
                if found > best[0]:
                    best = (found, gray.copy(), corners, ids)
            if args.save and best[1] is None:
                best = (0, gray.copy(), (), None)

    elapsed = time.perf_counter() - t_measure_start if measuring else 0.0
    if cap is not None:
        cap.release()

    if not frames:
        print("no frames captured -- check the device, format, and cabling")
        return 1

    p1, s_busy1, s_tot1 = read_proc_times()
    proc_cpu = (p1 - p0) / elapsed * 100
    sys_cpu = (s_busy1 - s_busy0) / (s_tot1 - s_tot0) * 100
    cores = os.cpu_count() or 1

    print(f"\n=== {frames} frames in {elapsed:.1f}s ===")
    print(f"throughput      {frames / elapsed:6.2f} fps "
          f"(requested {args.fps})")
    hits = sum(1 for c in marker_counts if c)
    print(f"markers/frame   {np.mean(marker_counts):.2f} "
          f"(max {max(marker_counts)})")
    print(f"detection rate  {100 * hits / frames:5.1f}%  "
          f"({hits}/{frames} frames saw at least one marker)")
    if id_hits:
        detail = ", ".join(f"id {k}: {100 * v / frames:.0f}%"
                           for k, v in sorted(id_hits.items()))
        print(f"per-id          {detail}")
    else:
        print("per-id          NOTHING DETECTED -- check the marker is in view, "
              "in focus, and really is " + args.dict_name)
    if read_failures:
        print(f"read failures   {read_failures}")
    print("per-stage latency:")
    print(summarise("capture", t_cap))
    print(summarise("to-gray", t_gray))
    print(summarise("detect", t_detect))
    print(summarise("pose", t_pose))
    print(summarise("loop", t_loop))

    print("\ncpu:")
    print(f"  this process  {proc_cpu:6.1f}%  of one core "
          f"({proc_cpu / cores:.1f}% of the {cores}-core machine)")
    print(f"  system-wide   {sys_cpu:6.1f}%  busy across all cores")

    temp, throttled = read_thermals()
    if temp is not None:
        print(f"  soc temp      {temp:6.1f} C")
    if throttled is not None:
        print(f"  throttling    {describe_throttling(throttled)}")

    if args.save:
        if best[1] is None:
            print(f"\nnot writing {args.save}: no frame was ever captured")
        else:
            canvas = cv2.cvtColor(best[1], cv2.COLOR_GRAY2BGR)
            if best[0]:
                cv2.aruco.drawDetectedMarkers(canvas, best[2], best[3])
            cv2.imwrite(args.save, canvas)
            print(f"\nwrote {args.save} ({best[0]} markers drawn)"
                  + ("" if best[0] else " -- nothing detected, so this is a "
                                        "plain frame: look at whether the image "
                                        "is even sensible"))

    # Judge on processing time only. `capture` is dominated by blocking until
    # the camera hands over the next frame, which is idle waiting, not load --
    # counting it marks a run that already achieves full frame rate as failing.
    budget_ms = 1000.0 / args.fps
    work = np.array(t_gray) + np.array(t_detect) + np.array(t_pose)
    p90 = np.percentile(work, 90)
    print(f"\nverdict: p90 processing {p90:.1f} ms (to-gray + detect + pose, "
          f"excluding the wait for the next frame) vs {budget_ms:.1f} ms "
          f"budget at {args.fps} fps -> ", end="")
    if p90 <= budget_ms * 0.7:
        print("comfortable headroom")
    elif p90 <= budget_ms:
        print("fits, but little margin for other nodes")
    else:
        print(f"CANNOT keep up; sustainable rate is ~{1000 / p90:.1f} fps")
    print(f"         achieved {frames / elapsed:.1f} fps end to end at "
          f"{proc_cpu / cores:.1f}% of the machine")
    return 0


if __name__ == "__main__":
    sys.exit(main())
