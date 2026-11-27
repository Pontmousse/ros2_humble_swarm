"""Recent published poses, so a camera frame can be lifted with the pose at its capture stamp."""
from bisect import bisect_left
from collections import deque


class PoseHistory:
    def __init__(self, span=2.0):
        self.span, self.buf = span, deque()   # (t seconds, x, y, theta_deg)

    def add(self, t, pose):
        self.buf.append((t, *pose))
        while self.buf and t - self.buf[0][0] > self.span:
            self.buf.popleft()

    def at(self, t, fallback):
        """Pose interpolated at t; `fallback` when t is outside the buffer (another clock, too old, in the future)."""
        b = self.buf
        if len(b) < 2 or not b[0][0] <= t <= b[-1][0]:
            return fallback
        i = max(bisect_left([e[0] for e in b], t), 1)
        (t0, x0, y0, th0), (t1, x1, y1, th1) = b[i - 1], b[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        dth = (th1 - th0 + 180.0) % 360.0 - 180.0
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0), th0 + a * dth
