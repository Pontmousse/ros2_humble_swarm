"""Tests for the capture approach law's fixed point and its gain budget.

Until 2026-10-10 this law was a relay. The error was computed in centimetres
and written straight into `frc_cmd`, which `publish_cmd` clamps against
`max_translational_speed` in m/s, so the linear region was 0.5 mm wide in x and
1.5 mm in y -- two orders of magnitude under the 52 mm Marvelmind noise floor.
Past that the command was full scale, and crossing `buffer_radius` flipped its
sign with no taper, so there was no distance at which the robot was told to
stop. RM6 held the target but wandered the whole keep-out circle.

The tests below pin the two properties that failure lacked: a distance where
the command is exactly zero, and a gain small enough that a zero-order-hold
loop closed at the Marvelmind rate converges to it rather than oscillating
about it.

`target_callback` is called unbound against a stub rather than against a real
node: constructing `ControllerNode` joins the DDS graph, and the dev host
shares a domain with live robots.
"""

import collections
import math
import types

import numpy as np
import pytest

from swarm_controller.cp_proportional_controller import (
    TRACK_MAX_GAP,
    TRACK_MAX_JUMP,
    ControllerNode,
)

from swarm_interfaces.msg import CoordXY, Markers


# Node defaults. buffer_radius is metres and is now also the hold distance, not
# just a boundary to cross.
BUFFER_RADIUS = 0.60
APPROACH_GAIN = 0.9
ALIGN_GAIN = 0.05
HEADING_GAIN = 1.0

# publish_cmd's clamp, m/s, body x and y, and rad/s.
MAX_SPEED = [0.05, 0.15]
MAX_YAW_RATE = 0.2

# Marvelmind's measurement period with three beacons sharing the TDMA cycle:
# 2.87 Hz per beacon. This -- not timer_frequency -- is the period the loop is
# actually closed at, because frc_cmd only changes when target_callback fires.
# 0.6 s has been observed when fixes freeze, so the margin has to cover it.
TDMA_PERIOD = 0.35
FROZEN_PERIOD = 0.61

# Inlier-only Marvelmind noise, std over ~10 bags. The old law's linear region
# was 0.5 mm; anything narrower than this is saturated by sensor noise alone
# and only the sign of the error reaches the wheels.
NOISE_FLOOR = 0.052

# The same noise seen as a bearing. The target bearing is derived from
# Marvelmind positions by target_bridge, so at buffer_radius the 52 mm floor
# subtends 0.087 rad -- 5.0 deg, far coarser than an ArUco bearing would be,
# and it grows as the robot closes.
BEARING_NOISE = NOISE_FLOOR / BUFFER_RADIUS

# target_bridge's publish rate. This is what target_callback actually fires at,
# and it is deliberately NOT the fix rate: between fixes it republishes a
# zero-order hold, so three and a half samples in a row carry one measurement.
MARKERS_PERIOD = 0.1

# Feedforward defaults. The span is sized by the noise on a least-squares
# slope, sigma / (T * sqrt(M(M^2-1)/12)) over INDEPENDENT fixes: 15 fixes of
# 52 mm at 0.35 s is 5.25 s and gives 9 mm/s against a 30 mm/s target.
TRACK_SPAN = 5.25
FEEDFORWARD_GAIN = 1.0

# Size of the beacon-5 position outliers logged on 2026-10-10: the target
# apparently teleported ~2.1 m and came back within two fixes, and both robots
# saw it at the same instant, which is what identifies it as the beacon rather
# than either robot's pose.
BEACON_OUTLIER = 2.1


def stub(approach_gain=APPROACH_GAIN, align_gain=ALIGN_GAIN,
         heading_gain=HEADING_GAIN, span=TRACK_SPAN,
         feedforward_gain=FEEDFORWARD_GAIN):
    """Enough of ControllerNode for the laws, with a clock the test drives.

    The tracking methods are bound onto the namespace rather than reimplemented
    so the tests exercise the shipped arithmetic, and the clock is fake because
    the velocity estimate is a slope against time -- a real clock would make
    the window length depend on how fast the test machine runs.
    """
    node = types.SimpleNamespace(
        ref_offset_x=10.0,
        ref_offset_y=0.0,
        closest_target_index=None,
        frc_cmd=np.array([None, None]),
        trq_cmd=None,
        buffer_radius=BUFFER_RADIUS,
        approach_gain=approach_gain,
        align_gain=align_gain,
        heading_gain=heading_gain,
        target_velocity_span=span,
        feedforward_gain=feedforward_gain,
        target_track=collections.deque(),
        pose_x=0.0,
        pose_y=0.0,
        pose_theta=0.0,
        seconds=0.0,
    )
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=node.seconds * 1e9))
    for name in ("to_body", "track_target", "target_velocity_map"):
        setattr(node, name,
                types.MethodType(getattr(ControllerNode, name), node))
    return node


def markers(*positions_cm):
    """Markers message. Positions are body frame, centimetres, as published."""
    msg = Markers()
    for x, y in positions_cm:
        marker = CoordXY()
        marker.x, marker.y = float(x), float(y)
        msg.markers.append(marker)
        normal = CoordXY()
        bearing = math.atan2(y, x)
        normal.x, normal.y = -math.sin(bearing), math.cos(bearing)
        msg.markers_normals.append(normal)
    return msg


def command(distance_m, bearing_rad=0.0, **gains):
    """Velocity command for a single target at this range and bearing."""
    node = stub(**gains)
    x = 100.0 * distance_m * math.cos(bearing_rad)
    y = 100.0 * distance_m * math.sin(bearing_rad)
    ControllerNode.target_callback(node, markers((x, y)))
    return np.array([float(node.frc_cmd[0]), float(node.frc_cmd[1])])


def heading(bearing_rad, distance_m=BUFFER_RADIUS, **gains):
    """Yaw rate command, rad/s, for a single target at this bearing."""
    node = stub(**gains)
    x = 100.0 * distance_m * math.cos(bearing_rad)
    y = 100.0 * distance_m * math.sin(bearing_rad)
    ControllerNode.target_callback(node, markers((x, y)))
    return float(node.trq_cmd)


def radial_component(distance_m, bearing_rad):
    u_r = np.array([math.cos(bearing_rad), math.sin(bearing_rad)])
    return float(command(distance_m, bearing_rad) @ u_r)


def test_the_keep_out_circle_is_a_fixed_point():
    """The property the relay lacked: a distance commanding exactly zero."""
    assert command(BUFFER_RADIUS) == pytest.approx([0.0, 0.0], abs=1e-12)


def test_markers_are_read_as_centimetres():
    """A regression to metres would put the fixed point at 40 m."""
    assert command(100.0 * BUFFER_RADIUS)[0] > 0.0


def test_the_command_reverses_across_the_circle():
    assert command(BUFFER_RADIUS + 0.2)[0] > 0.0
    assert command(BUFFER_RADIUS - 0.2)[0] < 0.0


def test_the_command_tapers_toward_the_circle():
    """A relay holds full scale on both sides; this has to shrink."""
    far = abs(command(BUFFER_RADIUS + 0.20)[0])
    near = abs(command(BUFFER_RADIUS + 0.05)[0])
    assert near < far
    assert near == pytest.approx(APPROACH_GAIN * 0.05)


def test_the_linear_region_is_wider_than_the_sensor_noise():
    """What made the old law a relay in practice rather than in principle.

    The linear region is max_translational_speed / gain wide. Body x caps at
    0.05 m/s, so past gain 0.96 the noise floor alone saturates the command and
    its sign comes from noise -- a relay again, by a different route.
    """
    assert abs(command(BUFFER_RADIUS + NOISE_FLOOR)[0]) < MAX_SPEED[0]
    assert APPROACH_GAIN < MAX_SPEED[0] / NOISE_FLOOR


def test_the_hold_distance_does_not_depend_on_bearing():
    """Why the alignment term is tangential rather than along body y.

    A lateral term with a radial component trades bearing error for standoff
    error, so the keep-out stops being a circle -- the failure the
    buffer_radius comment rules out.
    """
    for bearing in (0.0, 0.4, math.pi / 2, 2.0, math.pi, -1.1):
        assert radial_component(BUFFER_RADIUS, bearing) == pytest.approx(0.0, abs=1e-12)


def test_the_tangential_term_opposes_the_bearing_error():
    """Orbiting along +u_t changes bearing by -delta/distance, so same sign."""
    for bearing in (0.3, 1.2, -0.8):
        u_t = np.array([-math.sin(bearing), math.cos(bearing)])
        tangential = command(BUFFER_RADIUS, bearing) @ u_t
        assert math.copysign(1.0, tangential) == math.copysign(1.0, bearing)
        assert tangential == pytest.approx(ALIGN_GAIN * bearing)


def test_a_detection_at_the_robot_centre_commands_nothing():
    """Both unit vectors are undefined there; no direction may be read off."""
    assert command(0.0) == pytest.approx([0.0, 0.0])


def test_the_closest_target_is_the_one_steered_to():
    node = stub()
    ControllerNode.target_callback(node, markers((200.0, 0.0), (60.0, 0.0)))
    assert node.closest_target_index == 1
    assert float(node.frc_cmd[0]) == pytest.approx(APPROACH_GAIN * (0.60 - BUFFER_RADIUS))


def settle(period, steps=400, start=1.20, target_speed=0.0, **gains):
    """Close the loop at the measurement period and return the distance trace.

    The robot is held facing the target and the target recedes straight ahead,
    so the approach is one-dimensional along body x and the chassis tracks the
    command within one period. That overstates the plant, which is the point:
    anything this simple loop cannot hold, the real one cannot either.
    """
    distance = start
    trace = [distance]
    for _ in range(steps):
        commanded = command(distance, **gains)[0]
        speed = min(max(commanded, -MAX_SPEED[0]), MAX_SPEED[0])
        distance += (target_speed - speed) * period
        trace.append(distance)
    return trace


def test_the_loop_settles_on_the_circle_at_the_marvelmind_rate():
    trace = settle(TDMA_PERIOD)
    assert trace[-1] == pytest.approx(BUFFER_RADIUS, abs=1e-3)


def test_the_loop_still_settles_on_a_frozen_fix():
    """1.64 Hz effective pose rate, measured on RM6 2026-10-10."""
    trace = settle(FROZEN_PERIOD)
    assert trace[-1] == pytest.approx(BUFFER_RADIUS, abs=1e-3)


def test_the_approach_is_monotone_rather_than_a_limit_cycle():
    """No overshoot means no boundary crossing, so nothing to oscillate about."""
    trace = settle(TDMA_PERIOD)
    assert all(b <= a + 1e-12 for a, b in zip(trace, trace[1:]))
    assert min(trace) >= BUFFER_RADIUS - 1e-9


def test_the_old_centimetre_gain_would_not_settle():
    """Guards the gain, not the code: this is what the units used to imply.

    frc_cmd in cm against a cap in m/s is 100 s^-1, and a zero-order-hold
    proportional loop needs gain * period < 2.
    """
    assert 100.0 * TDMA_PERIOD > 2.0
    trace = settle(TDMA_PERIOD, approach_gain=100.0)
    assert trace[-1] != pytest.approx(BUFFER_RADIUS, abs=1e-3)
    crossings = sum(1 for a, b in zip(trace, trace[1:])
                    if (a - BUFFER_RADIUS) * (b - BUFFER_RADIUS) < 0.0)
    assert crossings > 10


@pytest.mark.parametrize("period", [TDMA_PERIOD, FROZEN_PERIOD])
def test_the_default_gain_is_inside_the_zero_order_hold_budget(period):
    assert APPROACH_GAIN * period < 2.0


# RM5's commanded speed while serving as the moving target on 2026-10-10.
TARGET_SPEED = 0.03


def test_tracking_lag_is_the_target_speed_over_the_gain():
    """Proportional-only has no integrator, so a moving target costs standoff.

    This is the floor on approach_gain, the way the zero-order-hold budget is
    the ceiling. Closing it properly wants target-velocity feedforward.
    """
    for gain in (0.5, 1.5, 3.0):
        trace = settle(TDMA_PERIOD, target_speed=TARGET_SPEED,
                       approach_gain=gain)
        lag = trace[-1] - BUFFER_RADIUS
        assert lag == pytest.approx(TARGET_SPEED / gain, rel=1e-3)


def test_the_default_gain_keeps_the_lag_under_the_sensor_noise():
    """Below the noise floor the lag is not the dominant error term."""
    assert TARGET_SPEED / APPROACH_GAIN < NOISE_FLOOR


def test_a_moving_target_is_tracked_without_oscillating():
    """The RM6 run this law replaces: the swing, not the offset, was the fault."""
    trace = settle(FROZEN_PERIOD, target_speed=TARGET_SPEED)
    tail = trace[len(trace) // 2:]
    assert max(tail) - min(tail) < 1e-6


######################################################################
# Target velocity and feedforward
######################################################################
# settle() above rebuilds the node every step, so no track accumulates and it
# measures the pure proportional law. These drive one node through a run with
# its own clock, which is the only way the estimator is exercised at all.


def chase(period=MARKERS_PERIOD, steps=600, start=1.60, target_speed=0.0,
          **kw):
    """Run one node along body x and return (range trace, node).

    The robot and the target both move in the map frame, so the estimator sees
    exactly what it would on hardware: a body-frame bearing that changes for
    two different reasons at once. The default period is the MARKERS rate,
    which is what target_callback actually fires at.
    """
    node = stub(**kw)
    robot_x, target_x = 0.0, start
    trace = [start]
    for _ in range(steps):
        node.seconds += period
        node.pose_x = robot_x
        ControllerNode.target_callback(
            node, markers((100.0 * (target_x - robot_x), 0.0)))
        speed = min(max(float(node.frc_cmd[0]), -MAX_SPEED[0]), MAX_SPEED[0])
        robot_x += speed * period
        target_x += target_speed * period
        trace.append(target_x - robot_x)
    return trace, node


def fill_track(node, period=MARKERS_PERIOD, at=(1.0, 0.0)):
    """Run a standing target until the span is covered and the estimate up."""
    while node.target_velocity_map() is None:
        node.seconds += period
        node.track_target(np.array(at))
        assert node.seconds < 10 * TRACK_SPAN, "estimate never arrived"
    return node


def test_the_span_is_wall_time_not_sample_count():
    """The bug this replaced, and the reason the parameter is in seconds.

    target_callback fires at the 10 Hz markers rate, not the 0.35 s fix rate,
    so a 15-SAMPLE window spanned 1.5 s and held about four independent
    fixes -- the rest are zero-order holds carrying the same noise. That is
    59 mm/s against a 30 mm/s signal, fed into a 50 mm/s axis cap, which is
    the relay the approach law had just been fixed for.
    """
    node = stub()
    for _ in range(15):
        node.seconds += MARKERS_PERIOD
        node.track_target(np.array([1.0, 0.0]))
    assert len(node.target_track) == 15
    assert node.target_velocity_map() is None


def test_the_velocity_estimate_is_withheld_until_the_span_is_covered():
    """The span IS the noise budget; a short run is unusable."""
    node = fill_track(stub())
    covered = node.target_track[-1][0] - node.target_track[0][0]
    assert covered == pytest.approx(TRACK_SPAN, abs=2 * MARKERS_PERIOD)


def test_the_track_is_pruned_to_the_span_rather_than_growing():
    """An unbounded deque on a 40 minute run is a leak, not a longer window."""
    node = fill_track(stub())
    settled = len(node.target_track)
    for _ in range(int(10 * TRACK_SPAN / MARKERS_PERIOD)):
        node.seconds += MARKERS_PERIOD
        node.track_target(np.array([1.0, 0.0]))
    assert len(node.target_track) == settled
    covered = node.target_track[-1][0] - node.target_track[0][0]
    assert covered < TRACK_SPAN + 2 * MARKERS_PERIOD


def test_a_stationary_target_reads_as_stationary_while_we_move():
    """Why the track is lifted to the map frame before it is differenced.

    Differencing the body-frame position would see our own approach as target
    motion, feed it forward, and run away from the circle. Here the robot
    covers real ground and the estimate still has to be zero.
    """
    trace, node = chase()
    assert trace[-1] == pytest.approx(BUFFER_RADIUS, abs=1e-3)
    assert node.target_velocity_map() == pytest.approx([0.0, 0.0], abs=1e-9)


def test_the_estimate_recovers_a_constant_target_velocity():
    _, node = chase(target_speed=TARGET_SPEED)
    assert node.target_velocity_map()[0] == pytest.approx(TARGET_SPEED,
                                                          rel=1e-6)


def test_feedforward_closes_the_tracking_lag():
    """The lag is v / approach_gain without it and nothing with it."""
    without, _ = chase(steps=1200, target_speed=TARGET_SPEED,
                       feedforward_gain=0.0)
    with_ff, _ = chase(steps=1200, target_speed=TARGET_SPEED,
                       feedforward_gain=1.0)
    assert without[-1] - BUFFER_RADIUS == pytest.approx(
        TARGET_SPEED / APPROACH_GAIN, rel=1e-3)
    assert with_ff[-1] == pytest.approx(BUFFER_RADIUS, abs=1e-3)


def test_feedforward_cannot_beat_the_chassis_cap():
    """What actually opened RM2 up to 2.48 m on 2026-10-10.

    The feedforward is added before bound(), so a target faster than body x
    can hold the robot off at any distance and no gain or feedforward term
    changes that. The fix is a faster chassis or a shorter approach, not
    control law tuning -- which is worth a test so it is not retuned for.
    """
    runaway, _ = chase(target_speed=2 * MAX_SPEED[0])
    assert runaway[-1] > runaway[0]


def test_a_dropped_fix_restarts_the_track_rather_than_biasing_it():
    """A gap would read to the regression as a long steady run."""
    node = fill_track(stub())
    node.seconds += TRACK_MAX_GAP + 0.1
    node.track_target(np.array([1.0, 0.0]))
    assert len(node.target_track) == 1
    assert node.target_velocity_map() is None


@pytest.mark.parametrize("jump", [TRACK_MAX_JUMP + 0.1, BEACON_OUTLIER])
def test_a_glitched_fix_restarts_the_track(jump):
    """Least squares is not robust; one sample metres away would dominate.

    BEACON_OUTLIER is the magnitude actually seen on beacon 5, so this is a
    regression test against field data rather than against the threshold.
    """
    node = fill_track(stub())
    node.seconds += MARKERS_PERIOD
    node.track_target(np.array([1.0 + jump, 0.0]))
    assert len(node.target_track) == 1
    assert node.target_velocity_map() is None


def test_the_estimator_is_silent_without_a_pose():
    """Without our own position the body-frame track cannot be lifted."""
    node = stub()
    node.pose_x = None
    node.seconds += MARKERS_PERIOD
    node.track_target(np.array([1.0, 0.0]))
    assert len(node.target_track) == 0


######################################################################
# Heading
######################################################################
# The rotation channel had the same defect as the approach law and was fixed
# a run later: trq_cmd was a heading error in DEGREES bounded against
# max_rotational_speed in rad/s, so the linear region was 0.2 deg and an
# implicit gain of 57.3 saturated everything past it.
#
# The two unit errors downstream cancel in MAGNITUDE but not in GAIN. The
# driver does z=-deg(msg.angular.z) at chassis.py:366, so the saturated rate
# deg(0.2) = 11.46 deg/s is exactly the 0.2 rad/s intended; only the width of
# the linear region was ever wrong. That is why the fault showed up as a
# limit cycle rather than a wrong speed.
#
# Measured on RM2 and RM6 on 2026-10-10, independently and simultaneously:
# bearing held a +-13 deg swing at 1.23 s and 1.37 s. A relay closed around
# an integrator with one sample of delay oscillates at 4*T, and the beacon
# period in that run was 0.32 s -- 4*0.32 = 1.28 s, between the two.
OLD_DEGREE_GAIN = math.degrees(1.0)


def test_a_target_dead_ahead_commands_no_rotation():
    assert heading(0.0) == pytest.approx(0.0, abs=1e-12)


def test_the_heading_command_opposes_the_bearing_error():
    for bearing in (0.3, 1.2, -0.8):
        assert heading(bearing) == pytest.approx(HEADING_GAIN * bearing)


def test_the_heading_command_is_radians_not_degrees():
    """A 0.1 rad error is 5.7 in degrees, which the 0.2 rad/s cap saturates.

    So this single assertion separates the two laws: under the old one every
    bearing error above 0.2 DEGREES came out at full scale.
    """
    assert abs(heading(0.1)) < MAX_YAW_RATE
    old = bound_like(OLD_DEGREE_GAIN * 0.1)
    assert abs(old) == pytest.approx(MAX_YAW_RATE)


def bound_like(value):
    """publish_cmd's clamp on the yaw channel."""
    return min(max(value, -MAX_YAW_RATE), MAX_YAW_RATE)


def test_the_heading_linear_region_is_wider_than_the_bearing_noise():
    """The ceiling on heading_gain, and the tightest of the three.

    Marvelmind noise reaches the bearing through target_bridge over a lever
    arm of only buffer_radius, so it is 0.087 rad here rather than the ~2 deg
    an ArUco bearing would give. Above 0.2 / 0.087 the noise alone saturates
    the command and its sign comes from noise -- a relay again. Both sides of
    this scale as 1 / buffer_radius, so the ceiling moved 1.5 -> 2.3 when the
    radius went 0.40 -> 0.60.
    """
    assert abs(heading(BEARING_NOISE)) < MAX_YAW_RATE
    assert HEADING_GAIN < MAX_YAW_RATE / BEARING_NOISE


def test_the_heading_wraps_the_short_way_round():
    """A target just behind must not be chased 350 deg the long way."""
    for bearing in (math.pi - 0.05, -math.pi + 0.05):
        assert abs(heading(bearing)) <= HEADING_GAIN * math.pi
        assert (math.copysign(1.0, heading(bearing))
                == math.copysign(1.0, bearing))


@pytest.mark.parametrize("period", [TDMA_PERIOD, FROZEN_PERIOD])
def test_the_heading_gain_is_inside_the_zero_order_hold_budget(period):
    assert HEADING_GAIN * period < 2.0


def settle_heading(period, steps=400, start=1.0, **gains):
    """Close the heading loop at the measurement period.

    Positive trq_cmd drives the bearing toward zero -- that is the sign
    convention the law encodes, and chassis.py's negation is what makes it
    true on the hardware.
    """
    bearing = start
    trace = [bearing]
    for _ in range(steps):
        bearing -= bound_like(heading(bearing, **gains)) * period
        trace.append(bearing)
    return trace


def test_the_heading_loop_settles_rather_than_limit_cycling():
    assert settle_heading(FROZEN_PERIOD)[-1] == pytest.approx(0.0, abs=1e-3)


def test_the_heading_approach_is_monotone():
    trace = settle_heading(TDMA_PERIOD)
    assert all(b <= a + 1e-12 for a, b in zip(trace, trace[1:]))
    assert min(trace) >= -1e-9


def test_the_old_degree_law_limit_cycles():
    """Reproduces the RM2/RM6 swing: amplitude is saturated rate times T."""
    trace = settle_heading(TDMA_PERIOD, heading_gain=OLD_DEGREE_GAIN)
    crossings = sum(1 for a, b in zip(trace, trace[1:]) if a * b < 0.0)
    assert crossings > 10

    tail = trace[len(trace) // 2:]
    assert max(tail) - min(tail) == pytest.approx(MAX_YAW_RATE * TDMA_PERIOD,
                                                  rel=0.5)
