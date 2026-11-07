"""Pluggable landmark/target estimators -- the modelling behind landmark_estimator.

The node (`landmark_estimator.py`) owns all the plumbing: subscriptions, QoS,
frame transforms (body<->map), timing and publishing. An estimator in this file
owns ONLY the model: how a fresh observation updates the remembered state of each
landmark, and what the current best estimate is. Keeping the two apart means a
richer model (constant-velocity, Kalman, factor-graph SLAM, inter-agent sharing)
drops in behind the same interface without touching the node or any controller.

Stable interface (selected by the node's `estimator_type` param):
    observe(id, p_map, n_map, t)  -- fuse one fresh observation
                                     (map frame, metres; t = seconds, monotonic)
    predict(dt)                   -- advance the motion model (no-op when static)
    landmarks() -> [LandmarkEstimate]   -- current confident best estimates

All map-frame, plain numpy -- no ROS imports, so these are unit-testable offline.
Data association is by ArUco ID (the `id` key); the node guarantees IDs are
present and parallel to the observed markers before calling `observe`.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class LandmarkEstimate:
    """One remembered landmark's current best estimate, in the map frame."""

    id: int
    p: np.ndarray            # position [x, y] in metres (map frame)
    n: np.ndarray            # outward normal [nx, ny], unit-length (map frame)
    confidence: float        # 0..1, monotonic in the number of fusions
    count: int               # observations fused so far
    last_seen: float         # estimator time of the last fusion (seconds)


class Estimator:
    """Abstract base: fuse ID-tagged map-frame observations into a memory."""

    def observe(self, landmark_id, p_map, n_map, t):
        raise NotImplementedError

    def predict(self, dt):
        """Advance the motion model by dt seconds. Static models do nothing."""
        raise NotImplementedError

    def landmarks(self):
        """Return the list of confident LandmarkEstimate (map frame)."""
        raise NotImplementedError


@dataclass
class _Entry:
    p: np.ndarray
    n: np.ndarray
    count: int
    last_seen: float


class AveragingEstimator(Estimator):
    """Static-world memory: per-ID cumulative mean of position and normal.

    New ID -> insert (count=1). Known ID -> incremental mean
    ``p += (z - p) / (count + 1)`` (same for the normal, then renormalised),
    which is the exact running average with no decay -- correct for a world that
    does not move. `min_observations` gates out one-off false positives before a
    landmark is reported; `forget_timeout` (0 = never) drops a landmark that has
    not been re-seen, so a genuinely removed marker eventually clears.

    A single monotonic clock is advanced by both `observe` (to the observation
    time) and `predict` (by dt), so ageing works whether or not detections flow.
    """

    def __init__(self, min_observations=2, forget_timeout=0.0):
        self.min_observations = max(1, int(min_observations))
        self.forget_timeout = float(forget_timeout)   # 0 -> keep forever
        self._mem = {}                                # id -> _Entry
        self._clock = 0.0

    # -- strategy interface ----------------------------------------------
    def observe(self, landmark_id, p_map, n_map, t):
        self._clock = max(self._clock, float(t))
        key = int(landmark_id)
        z_p = np.asarray(p_map, dtype=float).reshape(2)
        z_n = _unit(np.asarray(n_map, dtype=float).reshape(2))

        e = self._mem.get(key)
        if e is None:
            self._mem[key] = _Entry(
                p=z_p.copy(), n=z_n.copy(), count=1, last_seen=self._clock)
            return

        # Incremental (cumulative) mean: exact average over all observations.
        w = 1.0 / (e.count + 1)
        e.p += (z_p - e.p) * w
        e.n = _unit(e.n + (z_n - e.n) * w)
        e.count += 1
        e.last_seen = self._clock

    def predict(self, dt):
        # Static world: geometry does not evolve. Advancing the clock lets the
        # forget_timeout age landmarks even during a detection starve.
        self._clock += max(0.0, float(dt))

    def landmarks(self):
        out = []
        for key, e in self._mem.items():
            if e.count < self.min_observations:
                continue
            if self.forget_timeout > 0.0 and \
                    (self._clock - e.last_seen) > self.forget_timeout:
                continue
            out.append(LandmarkEstimate(
                id=key,
                p=e.p.copy(),
                n=e.n.copy(),
                confidence=min(1.0, e.count / float(self.min_observations)),
                count=e.count,
                last_seen=e.last_seen,
            ))
        return out


def _unit(v):
    """Return v scaled to unit length, or v unchanged if (near) zero."""
    norm = float(np.hypot(v[0], v[1]))
    if norm < 1e-9:
        return v
    return v / norm


def build_estimator(estimator_type, **params):
    """Factory: map `estimator_type` -> estimator instance.

    v1 ships only "averaging"; richer models register here as they land, so the
    node and launch files select them purely by name.
    """
    kind = (estimator_type or "averaging").strip().lower()
    if kind == "averaging":
        return AveragingEstimator(
            min_observations=params.get("min_observations", 2),
            forget_timeout=params.get("forget_timeout", 0.0),
        )
    raise ValueError(f"unknown estimator_type: {estimator_type!r}")
