#!/usr/bin/env python3
"""Verify the virtual dynamics stay inside chassis capability.

Green = the RoboMaster can render what the simulator will produce.
Red   = the reference outruns the chassis; position error will grow without
        bound until the virtual body turns around.
Exit code is the number of failures, so a launch file can gate on it.
"""

import math
import sys
from pathlib import Path

import yaml

DEFAULT_PARAMETERS = (
    Path(__file__).resolve().parent.parent / "config" / "virtual_spacecraft.yaml"
)

RED = "\033[1;31m"
GREEN = "\033[1;32m"
YELLOW = "\033[1;33m"
DIM = "\033[2m"
OFF = "\033[0m"

FAIL = f"{RED}FAIL{OFF}"
WARN = f"{YELLOW}WARN{OFF}"
OK = f"{GREEN} OK {OFF}"


def block(doc, key):
    return (doc.get(key) or {}).get("ros__parameters", {}) or {}


class Report:
    def __init__(self):
        self.failures = 0
        self.warnings = 0

    def check(self, ok, label, detail, warn_only=False):
        if ok:
            tag = OK
        elif warn_only:
            tag = WARN
            self.warnings += 1
        else:
            tag = FAIL
            self.failures += 1
        print(f"  [{tag}] {label:<34} {DIM}{detail}{OFF}")


def main(path):
    doc = yaml.safe_load(open(path))
    vs = block(doc, "/**/virtual_spacecraft")
    bb = block(doc, "/**/bounding_box_search")
    sm = block(doc, "/**/velocity_smoother")
    viz = block(doc, "/**/bounding_box_visualizer")
    cap = block(doc, "/**/chassis_capability")

    hw_linear = cap.get("hardware_max_linear", 3.5)
    hw_yaw = cap.get("hardware_max_yaw", 10.47)
    axis = cap.get("mecanum_axis", 0.2)
    margin = cap.get("tracking_margin", 0.7)

    mass = vs["mass"]
    inertia = vs["yaw_inertia"]
    sim_force = vs["maximum_force"]
    sim_torque = vs["maximum_torque"]

    chassis_vx, chassis_vy, chassis_wz = sm["max_velocity"][:3]
    accel_x, _, accel_yaw = sm["max_accel"][:3]

    burn = bb.get("initial_burn_duration", 0.0)
    burn_force = math.hypot(
        bb.get("initial_force_x", 0.0), bb.get("initial_force_y", 0.0)
    )
    burn_torque = abs(bb.get("initial_torque", 0.0))

    # A raised-cosine ramp integrates to (D - r)/D of the equivalent step, so
    # the delivered impulse - and therefore the peak speed - is lower.
    ramp = min(bb.get("initial_burn_ramp", 0.0), 0.5 * burn)
    burn_impulse_time = burn - ramp

    # Guidance can never apply more than the simulator accepts.
    effective_force = min(bb.get("maximum_force", sim_force), sim_force)
    effective_torque = min(bb.get("maximum_torque", sim_torque), sim_torque)

    a_max = effective_force / mass
    alpha_max = effective_torque / inertia
    v_worst = a_max * burn_impulse_time
    w_worst = alpha_max * burn_impulse_time
    v_burn = (burn_force / mass) * burn_impulse_time
    w_burn = (burn_torque / inertia) * burn_impulse_time

    rep = Report()
    print(f"\n{DIM}--- virtual dynamics ---{OFF}")
    print(f"  a_max     = {effective_force:.2f} N / {mass:.2f} kg "
          f"= {a_max:.3f} m/s^2")
    print(f"  alpha_max = {effective_torque:.2f} Nm / {inertia:.2f} = "
          f"{alpha_max:.3f} rad/s^2")
    print(f"  burn {burn:.1f} s with a {ramp:.1f} s cosine ramp "
          f"= {burn_impulse_time:.1f} s of equivalent full thrust")
    print(f"  after the burn: v={v_burn:.2f} m/s (worst {v_worst:.2f}), "
          f"w={w_burn:.2f} rad/s (worst {w_worst:.2f})")

    print(f"\n{DIM}--- renderability ---{OFF}")
    budget_v = margin * chassis_vx
    budget_w = margin * chassis_wz
    rep.check(
        v_burn <= budget_v, "nominal burn speed",
        f"{v_burn:.2f} <= {budget_v:.2f} m/s  ({margin:.0%} of "
        f"{chassis_vx:.2f})",
    )
    rep.check(
        v_worst <= budget_v, "worst-case speed at force ceiling",
        f"{v_worst:.2f} <= {budget_v:.2f} m/s",
    )
    rep.check(
        w_burn <= budget_w, "nominal burn yaw rate",
        f"{w_burn:.2f} <= {budget_w:.2f} rad/s",
    )
    rep.check(
        a_max <= accel_x, "virtual accel is renderable",
        f"{a_max:.2f} <= {accel_x:.2f} m/s^2",
    )
    rep.check(
        alpha_max <= accel_yaw, "virtual yaw accel is renderable",
        f"{alpha_max:.2f} <= {accel_yaw:.2f} rad/s^2",
    )

    print(f"\n{DIM}--- consistency ---{OFF}")
    rep.check(
        bb.get("maximum_force", sim_force) <= sim_force,
        "guidance force <= sim force",
        f"{bb.get('maximum_force')} vs {sim_force} "
        f"(guidance is clamped if larger)",
    )
    rep.check(
        bb.get("boundary_force", 0.0) <= sim_force,
        "boundary force <= sim force",
        f"{bb.get('boundary_force')} vs {sim_force}",
    )
    rep.check(
        bb.get("maximum_torque", sim_torque) <= sim_torque,
        "guidance torque <= sim torque",
        f"{bb.get('maximum_torque')} vs {sim_torque}",
    )
    rep.check(
        viz.get("maximum_force") == sim_force,
        "visualizer force scale matches",
        f"{viz.get('maximum_force')} vs {sim_force} (arrow clips otherwise)",
        warn_only=True,
    )

    print(f"\n{DIM}--- chassis envelope ---{OFF}")
    wheel = abs(chassis_vx) + abs(chassis_vy) + axis * abs(chassis_wz)
    rep.check(
        wheel <= hw_linear, "mecanum wheel budget",
        f"{wheel:.2f} <= {hw_linear:.2f} m/s ({wheel / hw_linear:.0%} used)",
    )
    print(f"  {DIM}headroom: linear {chassis_vx:.2f}/{hw_linear:.2f} m/s "
          f"({chassis_vx / hw_linear:.0%}), yaw {chassis_wz:.2f}/{hw_yaw:.2f} "
          f"rad/s ({chassis_wz / hw_yaw:.0%}){OFF}")

    if rep.failures:
        head = f"{RED}{rep.failures} LIMIT VIOLATION(S)"
        tail = "virtual reference will outrun the chassis"
        print(f"\n{head} - {tail}{OFF}\n")
    elif rep.warnings:
        print(f"\n{YELLOW}{rep.warnings} warning(s); dynamics are "
              f"renderable{OFF}\n")
    else:
        print(f"\n{GREEN}All checks passed - virtual dynamics are inside "
              f"chassis capability{OFF}\n")
    return rep.failures


if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PARAMETERS
    if not path.is_file():
        print(f"{RED}no such parameter file: {path}{OFF}")
        sys.exit(1)
    print(f"{DIM}parameters: {path}{OFF}")
    sys.exit(main(path))
