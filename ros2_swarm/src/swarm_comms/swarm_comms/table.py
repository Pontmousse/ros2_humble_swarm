"""Neighbour table with staleness marking and link statistics.

ROS-free so the same accounting runs in the node and in the sniffer.
"""
import time
from collections import deque

# One-way broadcast delay through the AP, measured 2026-10-03 over three 90 s
# runs on RM1/RM2/RM4: 77.3-79.9 ms with ~25 ms of jitter. It is AP DTIM
# buffering (dtim period 1, beacon int 100), so it is a property of the network,
# not of the payload, and it stays constant as the swarm grows.
TRANSPORT_LAG = 0.0786


class NeighbourEntry:
    def __init__(self, robot_id, window):
        self.robot_id = robot_id
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.omega = 0.0
        self.pose_valid = False
        self.last_rx = 0.0
        self.last_seq = -1
        self.valid = False
        self.total_received = 0
        self._window = deque(maxlen=window)  # (seq, delay)

    def age(self, now):
        return now - self.last_rx

    def update(self, pkt, t_recv, delay):
        # A restarted peer resets its counter; an old window would then report
        # a nonsensical expected count.
        if pkt["seq"] < self.last_seq - 1000:
            self._window.clear()
            self.total_received = 0

        self.x, self.y, self.yaw = pkt["x"], pkt["y"], pkt["yaw"]
        self.vx, self.vy, self.omega = pkt["vx"], pkt["vy"], pkt["omega"]
        self.pose_valid = pkt["pose_valid"]
        self.last_seq = pkt["seq"]
        self.last_rx = t_recv
        self.valid = True
        self.total_received += 1
        self._window.append((pkt["seq"], delay))

    def predicted(self, now, max_horizon, lag=TRANSPORT_LAG):
        """Dead-reckon the neighbour forward to `now`.

        `age()` is a local monotonic difference, so this needs no clock sync
        between robots; only the constant `lag` comes from the wire. Capped at
        `max_horizon` because velocity goes stale too — past that the neighbour
        has probably turned and extrapolating further makes things worse.

        Valid only because the packet carries map-frame velocity (see
        `packet.py`); adding a body-frame vx to a map-frame x is wrong by the
        sender's yaw, which spans the full circle in practice.
        """
        dt = min(self.age(now) + lag, max_horizon)
        return (self.x + self.vx * dt,
                self.y + self.vy * dt,
                self.yaw + self.omega * dt)

    def loss_fraction(self):
        if len(self._window) < 2:
            return 0.0
        expected = self._window[-1][0] - self._window[0][0] + 1
        if expected <= 0:
            return 0.0
        return max(0.0, 1.0 - len(self._window) / expected)

    def delay_stats(self):
        """(mean, max) one-way delay in seconds; only meaningful with synced clocks."""
        if not self._window:
            return 0.0, 0.0
        delays = [d for _, d in self._window]
        return sum(delays) / len(delays), max(delays)


class NeighbourTable:
    def __init__(self, own_id, stale_timeout, drop_timeout, window=200):
        self.own_id = own_id
        self.stale_timeout = stale_timeout
        self.drop_timeout = drop_timeout
        self.window = window
        self.entries = {}

    def ingest(self, pkt, t_recv=None, now_wall=None):
        """Returns False for own packets, which loop back on the same host."""
        if pkt["robot_id"] == self.own_id:
            return False
        t_recv = time.monotonic() if t_recv is None else t_recv
        now_wall = time.time() if now_wall is None else now_wall
        entry = self.entries.get(pkt["robot_id"])
        if entry is None:
            entry = NeighbourEntry(pkt["robot_id"], self.window)
            self.entries[pkt["robot_id"]] = entry
        entry.update(pkt, t_recv, now_wall - pkt["t_send"])
        return True

    def expire(self, now=None):
        """Mark stale entries invalid, delete long-gone ones."""
        now = time.monotonic() if now is None else now
        for robot_id in [k for k, e in self.entries.items()
                         if e.age(now) > self.drop_timeout]:
            del self.entries[robot_id]
        for entry in self.entries.values():
            if entry.age(now) > self.stale_timeout:
                entry.valid = False

    def valid_entries(self):
        return [e for e in self.entries.values() if e.valid and e.pose_valid]
