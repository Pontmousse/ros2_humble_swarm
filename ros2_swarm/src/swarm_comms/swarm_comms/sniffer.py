#!/usr/bin/env python3
"""Passive listener that reports per-robot rate, loss and delay.

Runs standalone with no ROS sourced:
    python3 sniffer.py [--port 9870] [--duration 30]

Delay is wall-clock t_recv minus t_send, so it is only meaningful when the Pis
and this host share an NTP/chrony source. Rate and loss are always valid.
"""
import argparse
import socket
import sys
import time
from collections import deque
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_comms.packet import DEFAULT_PORT, decode  # noqa: E402
from swarm_comms.table import NeighbourTable  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = run until Ctrl-C")
    parser.add_argument("--interval", type=float, default=2.0, help="report period")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.settimeout(0.2)

    # own_id 0 is never a real robot, so nothing is filtered out as "self".
    table = NeighbourTable(own_id=0, stale_timeout=1.0, drop_timeout=1e9)
    arrivals = {}
    start = time.monotonic()
    next_report = start + args.interval

    print(f"listening on udp/*:{args.port}  (Ctrl-C to stop)")
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                pkt = decode(data)
                if pkt is not None:
                    table.ingest(pkt)
                    arrivals.setdefault(pkt["robot_id"], deque(maxlen=500)).append(
                        (time.monotonic(), addr[0])
                    )
            except socket.timeout:
                pass

            now = time.monotonic()
            if now >= next_report:
                report(table, arrivals, args.interval)
                next_report = now + args.interval
            if args.duration and (now - start) >= args.duration:
                break
    except KeyboardInterrupt:
        pass

    print("\n=== final ===")
    report(table, arrivals, args.interval)
    sock.close()


def report(table, arrivals, interval):
    now = time.monotonic()
    print(f"\n{'robot':>5} {'from':>15} {'rate':>7} {'loss':>7} "
          f"{'delay_ms':>9} {'max_ms':>8} {'rx':>7} {'age_s':>6}")
    if not table.entries:
        print("  (no packets)")
        return
    for robot_id in sorted(table.entries):
        entry = table.entries[robot_id]
        window = arrivals.get(robot_id, deque())
        recent = [t for t, _ in window if now - t <= interval]
        rate = len(recent) / interval if interval > 0 else 0.0
        src = window[-1][1] if window else "?"
        mean_delay, max_delay = entry.delay_stats()
        print(f"{robot_id:>5} {src:>15} {rate:>6.1f}H {entry.loss_fraction()*100:>6.1f}% "
              f"{mean_delay*1000:>9.1f} {max_delay*1000:>8.1f} "
              f"{entry.total_received:>7} {entry.age(now):>6.2f}")


if __name__ == "__main__":
    main()
