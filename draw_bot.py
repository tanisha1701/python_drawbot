#!/usr/bin/env python3
"""
draw_bot.py - laptop side of the finger-drawn-path robot.

  1. Webcam + MediaPipe track your hand. Touch thumb and index finger together
     = pen down, separate them = pen up. You can only draw inside the green box.
  2. ENTER turns the drawing into points that are SPACING_M apart (sharp corners
     are kept), then into motor steps: (left PWM, right PWM, duration in ms).
  3. The whole step list is uploaded to the ESP32 (it runs its own Wi-Fi access
     point) and the ESP32 just replays it. No maths happens on the robot.

Setup
    pip install -r req.txt        (pins mediapipe==0.10.35 - do NOT use mediapipe 1.x on a Mac)
    put hand_landmarker.task next to this file (same file as your old project)
    join the "DrawBot" Wi-Fi network from the laptop, then:

        python draw_bot.py                   normal use
        python draw_bot.py --dry-run         everything except the robot
        python draw_bot.py --calibrate straight|spin|square     tuning runs
        python draw_bot.py --ramp                    find the PWM where the wheels really move
        python draw_bot.py --test-pwm 230 --seconds 3   both wheels at PWM 230 for 3 s
        python draw_bot.py --test-pwm 200 -200          left 200 forward, right 200 reverse

Keys
    EDIT      ENTER preview | u undo last stroke | l close loop | c clear | q quit
    PREVIEW   ENTER send + GO | e keep editing | c clear | q quit
    RUNNING   SPACE or ESC = emergency stop
"""
from __future__ import annotations

import argparse
import math
import os
import select
import socket
import sys
import time
import types
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:          # only needed for the drawing UI
    cv2 = None

# =============================================================================
# SETTINGS - everything you may want to tune lives here
# =============================================================================

# ---- Connection to the ESP32 (its own access point) --------------------------
ESP_HOST = "192.168.4.1"
ESP_PORT = 8080
ESP_MAX_STEPS = 1500              # must match MAX_STEPS in draw_bot.ino
MAX_STEP_MS = 60000               # ESP stores durations as 16-bit (max 65535 ms)

# ---- Camera / drawing box ----------------------------------------------------
CAMERA_INDEX = 0
CAMERA_W, CAMERA_H = 1280, 720
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "hand_landmarker.task")
BOX_FRACTION = 0.80               # square drawing box = this fraction of frame height
ARENA_SIDE_M = 1.0                # real floor distance the box side represents (m)

# ---- Pinch gesture (thumb tip to index tip, relative to hand size) ------------
PINCH_ON = 0.28                   # ratio below this  -> pen down
PINCH_OFF = 0.42                  # ratio above this  -> pen up   (hysteresis)
SMOOTH_ALPHA = 0.5                # fingertip smoothing: 1 = raw, lower = smoother
MIN_POINT_PX = 4                  # ignore movement smaller than this (pixels)
MIN_STROKE_PX = 15                # strokes shorter than this are accidental taps: ignored

# ---- Path sampling -------------------------------------------------------------
SPACING_M = 0.10                  # distance between the points sent to the robot
FINE_STEP_M = 0.01                # internal resolution used for smoothing/corners
SMOOTH_WINDOW = 5                 # moving-average window (in fine steps)
CORNER_WINDOW_M = 0.04            # look this far each side of a point for a corner
CORNER_DETECT_DEG = 55.0          # turn sharper than this = keep as a corner point

# ---- Robot geometry / motor calibration (tune with --ramp, --test-pwm, --calibrate)
#  PWM_MIN : run `--ramp`, note the lowest PWM at which BOTH wheels turn steadily on the
#            floor with the robot's weight, then add ~10-15 as margin.
#  V_MAX   : run `--test-pwm 255 --seconds 3`, measure the distance travelled,
#            speed = distance / 3. (V_MIN can stay 0.0 - the old project did the same.)
WHEEL_BASE_M = 0.14               # distance between the two wheels
PWM_MIN = 180                     # lowest PWM at which the wheels really turn
PWM_MAX = 255
V_MIN = 0.0                       # wheel speed (m/s) at PWM_MIN. 0.0 = same as the old rc_car project:
                                  # speed grows linearly from 0 at PWM_MIN up to V_MAX at PWM_MAX
V_MAX = 0.28                     # wheel speed (m/s) at PWM_MAX
V_CRUISE = 0.26                # normal driving speed of the robot centre (m/s)
SPIN_SPEED = 1               # wheel speed (m/s) used when turning on the spot
SPIN_RATE_DEG_S = 283.0       # measured rotation rate while spinning on the spot
TRIM_LEFT = 0.85                  # >1 speeds the left wheel up (fix drifting)
TRIM_RIGHT = 1.0
POLARITY_LEFT = 1                 # set to -1 if a motor is wired backwards
POLARITY_RIGHT = 1

# ---- Motion planning -----------------------------------------------------------
START_HEADING_DEG = 90.0          # robot starts facing UP on the screen
CORNER_PIVOT_DEG = 35.0           # heading change above this -> spin on the spot
PIVOT_TOL_DEG = 6.0               # ignore spins smaller than this (too short to be reliable)
SETTLE_MS = 150                   # pause before/after a spin so the robot settles


# =============================================================================
# GEOMETRY: raw finger track -> evenly spaced waypoints
# =============================================================================
def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _dedupe(pts: np.ndarray, eps: float = 1e-4) -> np.ndarray:
    keep = [0]
    for i in range(1, len(pts)):
        if np.hypot(*(pts[i] - pts[keep[-1]])) > eps:
            keep.append(i)
    return pts[keep]


def _cum(pts: np.ndarray) -> np.ndarray:
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    return np.concatenate([[0.0], np.cumsum(seg)])


def _uniform(pts: np.ndarray, step: float) -> np.ndarray:
    """Resample a polyline so consecutive points are (almost exactly) `step` apart."""
    c = _cum(pts)
    total = c[-1]
    n = max(1, int(round(total / step)))
    s = np.linspace(0.0, total, n + 1)
    return np.column_stack([np.interp(s, c, pts[:, 0]), np.interp(s, c, pts[:, 1])])


def _smooth(pts: np.ndarray, w: int) -> np.ndarray:
    if w < 3 or len(pts) < w:
        return pts
    k = w // 2
    pad = np.vstack([np.repeat(pts[:1], k, 0), pts, np.repeat(pts[-1:], k, 0)])
    ker = np.ones(w) / w
    out = np.column_stack([np.convolve(pad[:, 0], ker, "valid"),
                           np.convolve(pad[:, 1], ker, "valid")])
    out[0], out[-1] = pts[0], pts[-1]
    return out


def _find_corners(fine: np.ndarray, k: int, thresh_rad: float) -> List[int]:
    """Indices of sharp turns in a finely, evenly sampled path."""
    n = len(fine)
    if n < 2 * k + 1:
        return []
    a = fine[k:n - k] - fine[0:n - 2 * k]        # p[i]   - p[i-k]
    b = fine[2 * k:n] - fine[k:n - k]            # p[i+k] - p[i]
    turn = np.abs(np.arctan2(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0],
                             a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    corners: List[int] = []
    for j in range(len(turn)):
        if turn[j] < thresh_rad:
            continue
        lo, hi = max(0, j - k), min(len(turn), j + k + 1)
        if turn[j] >= turn[lo:hi].max():
            i = j + k
            if not corners or i - corners[-1] >= k:
                corners.append(i)
    return corners


def build_waypoints(raw_m: np.ndarray) -> Optional[np.ndarray]:
    """
    raw_m: (N,2) drawn points in metres (x right, y up).
    Returns waypoints spaced ~SPACING_M apart. Sharp corners are kept exactly,
    and each stretch between corners is divided into equal spacing.
    """
    pts = _dedupe(np.asarray(raw_m, dtype=float))
    if len(pts) < 2 or _cum(pts)[-1] < SPACING_M * 0.5:
        return None
    fine = _smooth(_uniform(pts, FINE_STEP_M), SMOOTH_WINDOW)
    k = max(1, int(round(CORNER_WINDOW_M / FINE_STEP_M)))
    idx = [0] + _find_corners(fine, k, math.radians(CORNER_DETECT_DEG)) + [len(fine) - 1]
    out = [fine[0]]
    for i0, i1 in zip(idx[:-1], idx[1:]):
        if i1 > i0:
            out.extend(_uniform(fine[i0:i1 + 1], SPACING_M)[1:])
    return np.array(out)


def detect_and_perfect_shape(raw_m: np.ndarray) -> Tuple[Optional[np.ndarray], str]:
    """Check if drawn points form a circle or polygon, and return perfect waypoints and shape name."""
    if len(raw_m) < 10: return None, ""
    
    start_pt, end_pt = raw_m[0], raw_m[-1]
    dist_closed = np.hypot(start_pt[0] - end_pt[0], start_pt[1] - end_pt[1])
    center = np.mean(raw_m, axis=0)
    radii = np.hypot(raw_m[:, 0] - center[0], raw_m[:, 1] - center[1])
    mean_r, std_r = np.mean(radii), np.std(radii)
    
    # Must be somewhat closed to be considered a deliberate shape
    if dist_closed > 0.4 * mean_r or mean_r < 0.05:
        return None, ""
        
    # 1. Count actual sharp corners in the drawing
    # A circle is a continuous curve with NO sharp corners. Polygons have 3+ corners.
    fine = _smooth(_uniform(raw_m, FINE_STEP_M), SMOOTH_WINDOW)
    k = max(1, int(round(CORNER_WINDOW_M / FINE_STEP_M)))
    sharp_corners = _find_corners(fine, k, math.radians(35.0))
    
    # 2. If it lacks sharp corners, it's definitively a Circle
    if len(sharp_corners) < 3:
        shape_name = "Circle"
        start_angle = math.atan2(start_pt[1] - center[1], start_pt[0] - center[0])
        vecs = raw_m - center
        cross_prods = vecs[:-1, 0] * vecs[1:, 1] - vecs[:-1, 1] * vecs[1:, 0]
        is_ccw = np.sum(cross_prods) > 0
        n_pts = max(10, int(round((2 * math.pi * mean_r) / SPACING_M)))
        angles = np.linspace(0, 2 * math.pi, n_pts + 1)
        if not is_ccw: angles = -angles
        angles += start_angle
        perfect_pts = np.column_stack([center[0] + mean_r * np.cos(angles), 
                                       center[1] + mean_r * np.sin(angles)])
        return perfect_pts, shape_name
        
    # 3. It has sharp corners -> Polygon. Find the optimal geometric vertices.
    curve = raw_m.astype(np.float32)
    peri = cv2.arcLength(curve, True)
    
    # Tighter epsilon (4% of perimeter) so it doesn't skip messy/indented corners
    approx = cv2.approxPolyDP(curve, 0.04 * peri, True)
    num_vertices = len(approx)
    
    shape_name = ""
    perfect_pts = None
    
    if num_vertices == 3:
        shape_name = "Triangle"
    elif num_vertices == 4:
        shape_name = "Rectangle"
    elif num_vertices == 5:
        shape_name = "Pentagon"
    elif num_vertices == 6:
        shape_name = "Hexagon"
        
    if shape_name:
        # Extract the corners from approx PolyDP
        corners = approx.reshape(-1, 2)
        corners = np.vstack([corners, corners[0]]) # close the loop
        
        # Connect the corners with perfectly straight lines spaced correctly
        out = [corners[0]]
        for i in range(len(corners) - 1):
            segment = corners[i:i+2]
            out.extend(_uniform(segment, SPACING_M)[1:])
        perfect_pts = np.array(out)
        return perfect_pts, shape_name
        
    return None, ""



# =============================================================================
# MOTION PLANNER: waypoints -> [(left_pwm, right_pwm, duration_ms), ...]
# =============================================================================
def speed_to_pwm(v: float) -> int:
    v = abs(v)
    if v < 1e-9:
        return 0
    r = min(max((v - V_MIN) / (V_MAX - V_MIN), 0.0), 1.0)
    return int(round(PWM_MIN + r * (PWM_MAX - PWM_MIN)))


def pwm_to_speed(pwm: float) -> float:
    pwm = abs(pwm)
    if pwm < PWM_MIN:
        return 0.0
    return V_MIN + (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN) * (V_MAX - V_MIN)


def wheel_pwm(v_left: float, v_right: float) -> Tuple[int, int]:
    """Signed wheel speeds (m/s) -> signed PWM for the two motors."""
    def one(v: float, trim: float, pol: int) -> int:
        p = speed_to_pwm(abs(v) * trim)
        return int(math.copysign(p, v) * pol) if p else 0
    return one(v_left, TRIM_LEFT, POLARITY_LEFT), one(v_right, TRIM_RIGHT, POLARITY_RIGHT)


@dataclass
class Plan:
    steps: List[Tuple[int, int, int]]     # (left_pwm, right_pwm, duration_ms)
    total_s: float


class _Builder:
    def __init__(self) -> None:
        self.steps: List[Tuple[int, int, int]] = []

    def hold(self, vl: float, vr: float, seconds: float) -> None:
        l, r = wheel_pwm(vl, vr)
        ms = int(round(seconds * 1000))
        while ms > 0:
            chunk = min(ms, MAX_STEP_MS)
            self.steps.append((l, r, chunk))
            ms -= chunk

    def pause(self) -> None:
        if self.steps and self.steps[-1][:2] != (0, 0):
            self.steps.append((0, 0, SETTLE_MS))

    def pivot(self, angle: float) -> None:
        """Spin on the spot by `angle` rad (positive = counter-clockwise / left)."""
        self.pause()
        sign = 1.0 if angle > 0 else -1.0
        self.hold(-sign * SPIN_SPEED, sign * SPIN_SPEED,
                  abs(angle) / math.radians(SPIN_RATE_DEG_S))
        self.steps.append((0, 0, SETTLE_MS))


def plan_steps(wp: np.ndarray) -> Plan:
    """
    Turn waypoints into motor steps.
      * gentle bends  -> one arc per segment (inner wheel slower than outer wheel)
      * sharp corners, or arcs too tight for the wheels -> spin on the spot
        (wheels in opposite directions), then drive straight
    """
    d = np.hypot(np.diff(wp[:, 0]), np.diff(wp[:, 1]))
    h = np.arctan2(np.diff(wp[:, 1]), np.diff(wp[:, 0]))         # chord headings
    m = len(d)
    corner_rad = math.radians(CORNER_PIVOT_DEG)
    tol = math.radians(PIVOT_TOL_DEG)
    mean_angle = lambda a, b: a + wrap(b - a) / 2.0

    bld = _Builder()
    theta = math.radians(START_HEADING_DEG)                      # robot's believed heading

    for i in range(m):
        is_start_corner = i == 0 or abs(wrap(h[i] - h[i - 1])) > corner_rad
        is_end_corner = i == m - 1 or abs(wrap(h[i + 1] - h[i])) > corner_rad
        a = h[i] if is_start_corner else mean_angle(h[i - 1], h[i])   # heading entering segment
        b = h[i] if is_end_corner else mean_angle(h[i], h[i + 1])     # heading leaving segment

        # Big heading error (corner): spin on the spot first.
        if abs(wrap(a - theta)) > tol:
            bld.pivot(wrap(a - theta))
            theta = a

        # Drive the segment as one constant-curvature arc from the robot's real
        # heading to the wanted end heading (this also steers out any small
        # residual error that was too small to be worth a spin).
        delta = wrap(b - theta)
        arc_len = d[i] * ((delta / 2) / math.sin(delta / 2) if abs(delta) > 1e-6 else 1.0)
        t = arc_len / V_CRUISE
        omega = delta / t
        vl = V_CRUISE - omega * WHEEL_BASE_M / 2
        vr = V_CRUISE + omega * WHEEL_BASE_M / 2
        if V_MIN - 1e-9 <= min(vl, vr) and max(vl, vr) <= V_MAX + 1e-9:
            bld.hold(vl, vr, t)
            theta = b
        else:
            # arc too tight for the wheels: face the chord, then go straight
            if abs(wrap(h[i] - theta)) > math.radians(2.0):
                bld.pivot(wrap(h[i] - theta))
            bld.hold(V_CRUISE, V_CRUISE, d[i] / V_CRUISE)
            theta = h[i]

    total = sum(ms for _, _, ms in bld.steps) / 1000.0
    return Plan(bld.steps, total)


# =============================================================================
# ROBOT LINK (TCP to the ESP32 access point)
# =============================================================================
class Robot:
    def __init__(self, host: str, port: int) -> None:
        self.host, self.port = host, port
        self.sock: Optional[socket.socket] = None
        self.buf = b""

    def connect(self, timeout: float = 3.0) -> None:
        self.close()
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        except OSError as e:
            self.sock = None
            raise ConnectionError(
                f"cannot reach {self.host}:{self.port} - is the laptop on the "
                f"DrawBot Wi-Fi and the ESP32 powered? ({e})")
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(3.0)
        self.buf = b""

    def poll(self, timeout: float = 0.0) -> List[str]:
        """Return any complete reply lines from the robot (non-blocking)."""
        if self.sock is None:
            return []
        readable, _, _ = select.select([self.sock], [], [], timeout)
        if readable:
            data = self.sock.recv(1024)
            if not data:
                raise ConnectionError("robot closed the connection")
            self.buf += data
        lines = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            text = raw.decode(errors="ignore").strip()
            if text:
                lines.append(text)
        return lines

    def upload_and_start(self, steps: List[Tuple[int, int, int]]) -> None:
        payload = "CLR\n" + "".join(f"S,{l},{r},{ms}\n" for l, r, ms in steps)
        payload += f"RUN,{len(steps)}\n"
        self.sock.sendall(payload.encode("ascii"))
        deadline = time.time() + 5.0
        while time.time() < deadline:
            for line in self.poll(0.1):
                if line.startswith("RUNNING"):
                    return
                if line.startswith("ERR"):
                    raise RuntimeError(f"robot said '{line}' (sent {len(steps)} steps)")
        raise TimeoutError("robot did not confirm the start")

    def stop(self) -> None:
        if self.sock is not None:
            try:
                self.sock.sendall(b"STOP\n")
            except OSError:
                pass

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def run_blocking(robot: Robot, plan: Plan,
                 marks: Optional[List[Tuple[float, str]]] = None) -> None:
    """Upload and wait for DONE (used by --calibrate). Ctrl+C = stop.
    marks: optional [(seconds_from_start, text)] printed as the run progresses."""
    robot.connect()
    robot.upload_and_start(plan.steps)
    print(f"Running {len(plan.steps)} steps, about {plan.total_s:.1f} s. Ctrl+C to stop.")
    t0 = time.time()
    pending = list(marks or [])
    try:
        while True:
            while pending and time.time() - t0 >= pending[0][0]:
                print(pending.pop(0)[1], flush=True)
            for line in robot.poll(0.05):
                if line == "DONE":
                    print("Done.")
                    return
                if line.startswith("ERR"):
                    raise RuntimeError(line)
    except KeyboardInterrupt:
        robot.stop()
        print("Stopped.")
    finally:
        robot.close()


# =============================================================================
# HAND TRACKING (MediaPipe HandLandmarker, same API as your old project)
# =============================================================================
def _patch_environment() -> None:
    """Workarounds carried over from your old run.py; only act if imports are broken."""
    try:
        import certifi  # noqa: F401
    except Exception:
        fake = types.ModuleType("certifi")
        fake.where = lambda: "/dev/null"
        sys.modules["certifi"] = fake
    try:
        import matplotlib.pyplot  # noqa: F401
    except Exception:
        for name in ("matplotlib", "matplotlib.pyplot", "matplotlib.patches",
                     "matplotlib.colors", "matplotlib.figure", "matplotlib.axes",
                     "matplotlib.ticker", "matplotlib.transforms", "matplotlib.rcsetup"):
            sys.modules[name] = types.ModuleType(name)


def _check_mediapipe_version() -> None:
    """
    MediaPipe 1.x hard-crashes (abort, not a Python exception) on Macs while the hand
    model loads: "graph_service.h Check failed: service_ Service is unavailable".
    Choosing the CPU delegate does not avoid it. Stop early with a readable message.
    """
    if sys.platform != "darwin":
        return
    try:
        from importlib.metadata import version
        ver = version("mediapipe")
        major = int(ver.split(".")[0])
    except Exception:
        return
    if major >= 1:
        sys.exit(
            f"mediapipe {ver} crashes on macOS when loading the hand model.\n"
            "Fix it with:\n"
            "    pip uninstall -y mediapipe opencv-python opencv-contrib-python\n"
            "    pip install -r req.txt        (installs mediapipe==0.10.35)")


@dataclass
class Hand:
    pen: np.ndarray        # midpoint of thumb tip and index tip (pixels)
    thumb: np.ndarray
    index: np.ndarray
    ratio: float           # thumb-index distance / hand size (small = pinching)


class HandTracker:
    def __init__(self, model_path: str) -> None:
        _patch_environment()
        _check_mediapipe_version()
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision

        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"hand_landmarker.task not found at {model_path}\n"
                "Copy it from your old project, or download it from:\n"
                "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                "hand_landmarker/float16/1/hand_landmarker.task")

        options = mp_vision.HandLandmarkerOptions(
            # CPU delegate: fine for this small model on every platform. (It does NOT
            # cure the macOS crash of mediapipe 1.x - that needs mediapipe==0.10.35.)
            base_options=mp_python.BaseOptions(
                model_asset_path=model_path,
                delegate=mp_python.BaseOptions.Delegate.CPU),
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        if sys.platform == "win32":                 # same Windows fix as your old project
            try:
                import ctypes
                import importlib.resources as _res
                from mediapipe.tasks.python.core import mediapipe_c_bindings as _mcb
                if _mcb._shared_lib is None:
                    dll = str(_res.files("mediapipe.tasks.c") / "libmediapipe.dll")
                    _mcb._shared_lib = ctypes.CDLL(dll)
                if not hasattr(_mcb._shared_lib, "free"):
                    _mcb._shared_lib.free = ctypes.cdll.msvcrt.free
            except Exception:
                pass
        self._mp = mp
        self._lm = mp_vision.HandLandmarker.create_from_options(options)

    def process(self, frame_bgr: np.ndarray) -> Optional[Hand]:
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        res = self._lm.detect(self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb))
        if not res.hand_landmarks:
            return None
        lm = res.hand_landmarks[0]
        pt = lambda i: np.array([lm[i].x * w, lm[i].y * h])
        thumb, index, wrist, mcp = pt(4), pt(8), pt(0), pt(9)
        size = max(float(np.linalg.norm(wrist - mcp)), 1e-6)
        return Hand(pen=(thumb + index) / 2, thumb=thumb, index=index,
                    ratio=float(np.linalg.norm(thumb - index)) / size)

    def close(self) -> None:
        self._lm.close()


# =============================================================================
# DRAWING APP
# =============================================================================
EDIT, PREVIEW, RUN = "EDIT", "PREVIEW", "RUNNING"
WIN = "DrawBot"
GREEN, CYAN, ORANGE = (0, 220, 60), (255, 220, 0), (0, 160, 255)
PINK, RED, WHITE, DARK = (200, 50, 255), (60, 60, 255), (240, 240, 240), (20, 20, 20)


class App:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.tracker = HandTracker(args.model)
        self.cap = cv2.VideoCapture(args.camera)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
        if not self.cap.isOpened():
            raise SystemExit("Cannot open the camera (try --camera 1).")
        self.robot = None if args.dry_run else Robot(args.host, args.port)

        self.state = EDIT
        self.box: Optional[Tuple[int, int, int, int]] = None
        self.strokes: List[List[np.ndarray]] = []
        self.pos: Optional[np.ndarray] = None
        self.pinching = False
        self.pen_down = False
        self.wp: Optional[np.ndarray] = None
        self.plan: Optional[Plan] = None
        self.run_t0 = 0.0
        self.msg = ""

    # ---- coordinates ----------------------------------------------------------
    def _inside(self, p: np.ndarray) -> bool:
        x0, y0, x1, y1 = self.box
        return x0 <= p[0] <= x1 and y0 <= p[1] <= y1

    def _to_m(self, px: np.ndarray) -> np.ndarray:
        x0, _, x1, y1 = self.box
        side = x1 - x0
        return np.column_stack([(px[:, 0] - x0) / side * ARENA_SIDE_M,
                                (y1 - px[:, 1]) / side * ARENA_SIDE_M])

    def _to_px(self, m: np.ndarray) -> np.ndarray:
        x0, _, x1, y1 = self.box
        side = x1 - x0
        return np.column_stack([x0 + m[:, 0] / ARENA_SIDE_M * side,
                                y1 - m[:, 1] / ARENA_SIDE_M * side]).astype(np.int32)

    # ---- EDIT: pen logic ------------------------------------------------------
    def _update_drawing(self, hand: Optional[Hand]) -> None:
        if hand is None:
            self.pos, self.pen_down, self.pinching = None, False, False
            return
        p = hand.pen
        was_pinching = self.pinching
        if hand.ratio < PINCH_ON:
            self.pinching = True
        elif hand.ratio > PINCH_OFF:
            self.pinching = False
        if self.pos is None or (self.pinching and not was_pinching):
            self.pos = p.copy()            # pen just touched down: start exactly here
        else:
            self.pos = SMOOTH_ALPHA * p + (1 - SMOOTH_ALPHA) * self.pos

        if self.pinching and self._inside(self.pos):
            if not self.pen_down:
                self.strokes.append([])
                self.pen_down = True
            stroke = self.strokes[-1]
            if not stroke or np.hypot(*(self.pos - stroke[-1])) >= MIN_POINT_PX:
                stroke.append(self.pos.copy())
        else:
            self.pen_down = False          # lifted, or left the box

    # ---- EDIT -> PREVIEW --------------------------------------------------------
    def _compile(self) -> None:
        strokes = [np.array(s) for s in self.strokes if len(s) >= 2]
        strokes = [s for s in strokes if _cum(s)[-1] >= MIN_STROKE_PX]
        if not strokes:
            self.msg = "Draw something first: pinch thumb + index inside the box."
            return
            
        raw_m = self._to_m(np.vstack(strokes))
        perfect_shape, shape_name = detect_and_perfect_shape(raw_m)
        
        if perfect_shape is not None:
            wp = perfect_shape
            is_perfect = True
        else:
            wp = build_waypoints(raw_m)
            is_perfect = False
            
        if wp is None or len(wp) < 2:
            self.msg = "Path too short."
            return
        plan = plan_steps(wp)
        if len(plan.steps) > ESP_MAX_STEPS:
            self.msg = f"Too many steps ({len(plan.steps)}). Draw less or raise SPACING_M."
            return
            
        self.wp, self.plan, self.state = wp, plan, PREVIEW
        self.msg = f"Auto-corrected to {shape_name}!" if is_perfect else ""
        print(f"Path: {len(wp)} points, {len(plan.steps)} steps, ~{plan.total_s:.1f} s")

    # ---- PREVIEW -> RUN ---------------------------------------------------------
    def _go(self) -> None:
        if self.robot is not None:
            try:
                self.robot.connect()
                self.robot.upload_and_start(self.plan.steps)
            except Exception as e:
                self.msg = f"Robot: {e}"
                print(self.msg)
                self.robot.close()
                return
        self.run_t0 = time.time()
        self.state, self.msg = RUN, ""

    def _finish_run(self, msg: str) -> None:
        if self.robot is not None:
            self.robot.close()
        self.state, self.msg = PREVIEW, msg

    def _poll_run(self) -> None:
        if self.robot is None:                                   # dry run: fake timer
            if time.time() - self.run_t0 >= self.plan.total_s:
                self._finish_run("Dry run finished.")
            return
        try:
            for line in self.robot.poll():
                if line == "DONE":
                    self._finish_run("Finished.")
                elif line.startswith("ERR"):
                    self._finish_run(f"Robot: {line}")
        except Exception as e:
            self._finish_run(f"Lost robot: {e}")

    def _emergency_stop(self) -> None:
        if self.robot is not None:
            self.robot.stop()
        self._finish_run("STOPPED by user.")

    # ---- keys -------------------------------------------------------------------
    def _handle_key(self, key: int) -> bool:
        """Return True to quit."""
        if key == 255:
            return False
        if self.state == RUN:
            if key in (27, 32):
                self._emergency_stop()
            elif key == ord("q"):
                self._emergency_stop()
                return True
            return False
        if key in (27, ord("q")):
            return True
        if key == ord("c"):
            self.strokes, self.pos, self.state, self.msg = [], None, EDIT, ""
        elif self.state == EDIT:
            if key in (13, 10):
                self._compile()
            elif key == ord("u") and self.strokes:
                self.strokes.pop()
            elif key == ord("l"):
                strokes = [s for s in self.strokes if len(s) >= 2]
                if strokes:
                    strokes[-1].append(strokes[0][0].copy())
        elif self.state == PREVIEW:
            if key in (13, 10):
                self._go()
            elif key == ord("e"):
                self.state, self.msg = EDIT, ""
        return False

    # ---- drawing on screen -------------------------------------------------------
    def _text(self, img, text, org, color=WHITE, scale=0.55):
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def _render(self, frame: np.ndarray, hand: Optional[Hand]) -> np.ndarray:
        out = frame
        h, w = out.shape[:2]
        x0, y0, x1, y1 = self.box
        box_color = GREEN if not self.pinching else (ORANGE if self._inside_pos() else RED)
        cv2.rectangle(out, (x0, y0), (x1, y1), box_color, 2, cv2.LINE_AA)

        for s in self.strokes:                                   # drawn strokes
            if len(s) >= 2:
                cv2.polylines(out, [np.array(s, dtype=np.int32)], False, CYAN, 2, cv2.LINE_AA)

        if self.state in (PREVIEW, RUN) and self.wp is not None:  # sampled points
            px = self._to_px(self.wp)
            cv2.polylines(out, [px], False, GREEN, 2, cv2.LINE_AA)
            for p in px:
                cv2.circle(out, tuple(int(v) for v in p), 4, ORANGE, -1, cv2.LINE_AA)
            sx, sy = int(px[0][0]), int(px[0][1])
            cv2.circle(out, (sx, sy), 9, GREEN, -1, cv2.LINE_AA)
            cv2.arrowedLine(out, (sx, sy), (sx, sy - 35), GREEN, 2, cv2.LINE_AA, tipLength=0.4)
            ex, ey = int(px[-1][0]), int(px[-1][1])
            cv2.circle(out, (ex, ey), 9, RED, -1, cv2.LINE_AA)

        if self.state == EDIT and hand is not None and self.pos is not None:   # HUD cursor
            tx, ty = int(self.pos[0]), int(self.pos[1])
            th_x, th_y = int(hand.thumb[0]), int(hand.thumb[1])
            in_x, in_y = int(hand.index[0]), int(hand.index[1])
            
            # HUD Colors (OpenCV uses BGR)
            hud_color = (255, 255, 0) if self.pinching else (200, 200, 200) # Cyan when drawing, Silver hover
            accent = (0, 165, 255) if self.pinching else (120, 120, 120)    # Orange/Gold accent vs Grey
            
            # Finger tip trackers
            cv2.circle(out, (th_x, th_y), 14, GREEN, 2, cv2.LINE_AA)
            cv2.circle(out, (in_x, in_y), 14, GREEN, 2, cv2.LINE_AA)
            
            # Reticle (Crosshair)
            r = 12
            cv2.circle(out, (tx, ty), 2, hud_color, -1, cv2.LINE_AA)
            cv2.circle(out, (tx, ty), r, hud_color, 1, cv2.LINE_AA)
            
            # Crosshair ticks
            tl = 8
            cv2.line(out, (tx - r - tl, ty), (tx - r, ty), hud_color, 2, cv2.LINE_AA)
            cv2.line(out, (tx + r, ty), (tx + r + tl, ty), hud_color, 2, cv2.LINE_AA)
            cv2.line(out, (tx, ty - r - tl), (tx, ty - r), hud_color, 2, cv2.LINE_AA)
            cv2.line(out, (tx, ty + r), (tx, ty + r + tl), hud_color, 2, cv2.LINE_AA)
            
            # Targeting brackets [ ] when "locked on" (pinching)
            if self.pinching:
                d = 22  # distance from center
                s = 8   # length of bracket segments
                thick = 2
                # Top Left
                cv2.line(out, (tx - d, ty - d), (tx - d + s, ty - d), accent, thick, cv2.LINE_AA)
                cv2.line(out, (tx - d, ty - d), (tx - d, ty - d + s), accent, thick, cv2.LINE_AA)
                # Top Right
                cv2.line(out, (tx + d, ty - d), (tx + d - s, ty - d), accent, thick, cv2.LINE_AA)
                cv2.line(out, (tx + d, ty - d), (tx + d, ty - d + s), accent, thick, cv2.LINE_AA)
                # Bottom Left
                cv2.line(out, (tx - d, ty + d), (tx - d + s, ty + d), accent, thick, cv2.LINE_AA)
                cv2.line(out, (tx - d, ty + d), (tx - d, ty + d - s), accent, thick, cv2.LINE_AA)
                # Bottom Right
                cv2.line(out, (tx + d, ty + d), (tx + d - s, ty + d), accent, thick, cv2.LINE_AA)
                cv2.line(out, (tx + d, ty + d), (tx + d, ty + d - s), accent, thick, cv2.LINE_AA)

        lines = {
            EDIT: ["Pinch thumb+index = draw (inside box)", "Open fingers = pen up",
                   "ENTER preview | u undo | l close loop | c clear | q quit"],
            PREVIEW: ["Green dot = start (place robot here, facing UP)",
                      "ENTER = send and GO | e = edit | c = clear | q = quit"],
            RUN: ["SPACE or ESC = EMERGENCY STOP"],
        }[self.state]
        pad = len(lines) * 22 + 10
        overlay = out.copy()
        cv2.rectangle(overlay, (0, h - pad), (560, h), DARK, -1)
        cv2.addWeighted(overlay, 0.6, out, 0.4, 0, out)
        for i, t in enumerate(lines):
            self._text(out, t, (8, h - pad + 20 + i * 22))

        head = self.state
        if self.state == EDIT and self.pen_down:
            head += "  (drawing)"
        if self.state == EDIT and hand is None:
            head += "  (no hand)"
        if self.plan is not None and self.state != EDIT:
            head += f"  |  {len(self.wp)} pts, {len(self.plan.steps)} steps, ~{self.plan.total_s:.0f} s"
        if self.state == RUN:
            head += f"  |  {time.time() - self.run_t0:.1f} s"
        self._text(out, head, (10, 26), CYAN, 0.65)
        if self.msg:
            self._text(out, self.msg, (10, 54), RED, 0.6)
        return out

    def _inside_pos(self) -> bool:
        return self.pos is not None and self._inside(self.pos)

    # ---- main loop -----------------------------------------------------------------
    def run(self) -> None:
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        try:
            while True:
                ok, frame = self.cap.read()
                if not ok:
                    print("Camera read failed.")
                    break
                frame = cv2.flip(frame, 1)                       # mirror view
                if self.box is None:
                    h, w = frame.shape[:2]
                    side = int(min(w, h) * BOX_FRACTION)
                    bx, by = (w - side) // 2, (h - side) // 2
                    self.box = (bx, by, bx + side, by + side)

                hand = None
                if self.state == EDIT:
                    hand = self.tracker.process(frame)
                    self._update_drawing(hand)
                elif self.state == RUN:
                    self._poll_run()

                cv2.imshow(WIN, self._render(frame, hand))
                key = cv2.waitKey(1) & 0xFF
                if self._handle_key(key):
                    break
                if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                    if self.state == RUN:
                        self._emergency_stop()
                    break
        finally:
            if self.state == RUN and self.robot is not None:
                self.robot.stop()
            if self.robot is not None:
                self.robot.close()
            self.tracker.close()
            self.cap.release()
            cv2.destroyAllWindows()


# =============================================================================
# CALIBRATION RUNS (no camera needed)
# =============================================================================
def calibration_plan(kind: str) -> Plan:
    if kind == "straight":                    # should travel exactly 1.00 m
        return plan_steps(np.array([[0.0, 0.0], [0.0, 1.0]]))
    if kind == "spin":                        # should turn exactly 360 degrees
        bld = _Builder()
        bld.steps.append((0, 0, 1))
        bld.pivot(2 * math.pi - 1e-6)
        bld.steps.pop(0)
        return Plan(bld.steps, sum(ms for _, _, ms in bld.steps) / 1000.0)
    if kind == "square":                      # 0.5 m square, ends where it started
        n = int(round(0.5 / SPACING_M))
        t = np.linspace(0, 0.5, n + 1)
        pts = ([(0, y) for y in t] + [(x, 0.5) for x in t[1:]] +
               [(0.5, 0.5 - y) for y in t[1:]] + [(0.5 - x, 0) for x in t[1:]])
        return plan_steps(np.array(pts, dtype=float))
    if kind == "circle":                      # perfect circle with 0.3m radius
        radius = 0.5
        n = int(round((2 * math.pi * radius) / SPACING_M))
        t = np.linspace(0, 2 * math.pi, n + 1)
        pts = [(radius * math.cos(th), radius * math.sin(th)) for th in t]
        return plan_steps(np.array(pts, dtype=float))
    raise ValueError(kind)


def pwm_test_plan(values: List[int], seconds: float) -> Plan:
    """Raw test: hold fixed PWM values (no speed model). 1 value = both wheels, 2 = left right."""
    if len(values) == 1:
        l = r = values[0]
    elif len(values) == 2:
        l, r = values
    else:
        raise ValueError("--test-pwm takes 1 value (both wheels) or 2 values (left right)")
    clamp = lambda v: max(-255, min(255, int(v)))
    l, r = clamp(l) * POLARITY_LEFT, clamp(r) * POLARITY_RIGHT
    ms = max(1, min(int(round(seconds * 1000)), MAX_STEP_MS))
    return Plan([(l, r, ms), (0, 0, 50)], (ms + 50) / 1000.0)


def ramp_plan(start: int = 100, stop: int = 255, step: int = 15,
              hold_s: float = 1.5, gap_s: float = 1.2
              ) -> Tuple[Plan, List[Tuple[float, str]]]:
    """Both wheels forward at rising PWM, with a stop between each level."""
    levels = list(range(start, stop, step)) + [stop]
    steps: List[Tuple[int, int, int]] = []
    marks: List[Tuple[float, str]] = []
    t = 0.0
    for pwm in levels:
        marks.append((t, f"  PWM {pwm:3d}"))
        steps.append((pwm * POLARITY_LEFT, pwm * POLARITY_RIGHT, int(hold_s * 1000)))
        t += hold_s
        steps.append((0, 0, int(gap_s * 1000)))
        t += gap_s
    return Plan(steps, t), marks


def main() -> None:
    ap = argparse.ArgumentParser(description="Finger-drawn path robot (laptop side)")
    ap.add_argument("--host", default=ESP_HOST)
    ap.add_argument("--port", type=int, default=ESP_PORT)
    ap.add_argument("--camera", type=int, default=CAMERA_INDEX)
    ap.add_argument("--model", default=MODEL_PATH)
    ap.add_argument("--dry-run", action="store_true", help="do everything except talk to the robot")
    ap.add_argument("--calibrate", choices=["straight", "spin", "square", "circle"],
                    help="run a tuning test on the robot instead of the drawing UI")
    ap.add_argument("--test-pwm", type=int, nargs="+", metavar="PWM",
                    help="hold raw PWM (1 value = both wheels, 2 = left right, negative = reverse)")
    ap.add_argument("--seconds", type=float, default=3.0, help="duration for --test-pwm")
    ap.add_argument("--ramp", action="store_true",
                    help="step both wheels through rising PWM to find where the robot really moves")
    args = ap.parse_args()

    if args.test_pwm or args.ramp:
        try:
            marks = None
            if args.ramp:
                plan, marks = ramp_plan()
                print("Robot on the floor, clear space ahead. Watch when BOTH wheels turn steadily:")
            else:
                plan = pwm_test_plan(args.test_pwm, args.seconds)
            if args.dry_run:
                print(f"{len(plan.steps)} steps, {plan.total_s:.1f} s:", plan.steps)
                return
            run_blocking(Robot(args.host, args.port), plan, marks)
        except Exception as e:
            sys.exit(f"Error: {e}")
        return

    if args.calibrate:
        plan = calibration_plan(args.calibrate)
        if args.dry_run:
            print(f"{len(plan.steps)} steps, {plan.total_s:.1f} s:", plan.steps[:12], "...")
            return
        try:
            run_blocking(Robot(args.host, args.port), plan)
        except Exception as e:
            sys.exit(f"Error: {e}")
        return

    if cv2 is None:
        sys.exit("OpenCV missing: pip install opencv-python")
    App(args).run()


if __name__ == "__main__":
    main()