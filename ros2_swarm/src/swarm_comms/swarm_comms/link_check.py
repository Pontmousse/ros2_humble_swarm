#!/usr/bin/env python3
"""Standalone Pi-to-Pi link check: broadcasts dummy state and reports on peers.

No ROS, no container, no rebuild. Copy this directory to each Pi and run:

    python3 swarm_comms/link_check.py --robot-id 1

Pose fields carry a slow circle so the receiver can eyeball that payloads are
not corrupted; this measures the link, not localization.
"""
import argparse
import math
import socket
import sys
import time
from collections import deque
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_comms.packet import DEFAULT_BROADCAST, DEFAULT_PORT, decode, encode  # noqa: E402
from swarm_comms.table import NeighbourTable  # noqa: E402


def udp_drops(port):
    """Kernel-side overflow count for our port.

    Non-zero means the socket buffer overran, which we can fix with --rcvbuf.
    Zero alongside high loss means the frames never reached the host at all.
    """
    hexport = f"{port:04X}"
    try:
        for table in ("/proc/net/udp", "/proc/net/udp6"):
            try:
                lines = Path(table).read_text().splitlines()[1:]
            except OSError:
                continue
            for line in lines:
                f = line.split()
                if f[1].split(":")[1] == hexport:
                    return int(f[-1])
    except (OSError, IndexError, ValueError):
        pass
    return -1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--broadcast", default=DEFAULT_BROADCAST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--iface", default="", help="pin TX to this device, e.g. wlan0")
    parser.add_argument("--duration", type=float, default=0.0, help="0 = until Ctrl-C")
    parser.add_argument("--interval", type=float, default=5.0, help="report period")
    parser.add_argument("--rcvbuf", type=int, default=0, help="SO_RCVBUF bytes, 0 = default")
    args = parser.parse_args()

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    if args.iface:
        tx.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, args.iface.encode())

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if args.rcvbuf:
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.rcvbuf)
    rx.bind(("", args.port))
    rx.settimeout(0.05)
    print(f"  rcvbuf={rx.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)} B, "
          f"kernel udp drops at start={udp_drops(args.port)}")

    table = NeighbourTable(own_id=args.robot_id, stale_timeout=1.0, drop_timeout=1e9)
    arrivals = {}
    period = 1.0 / args.rate
    seq = 0
    sent = 0
    start = time.monotonic()
    next_tx = start
    next_report = start + args.interval

    print(f"robot {args.robot_id}: broadcasting to {args.broadcast}:{args.port} "
          f"at {args.rate} Hz"
          + (f" via {args.iface}" if args.iface else ""))

    try:
        while True:
            now = time.monotonic()

            if now >= next_tx:
                a = (now - start) * 0.5
                try:
                    tx.sendto(
                        encode(args.robot_id, seq, time.time(),
                               2.0 * math.cos(a), 2.0 * math.sin(a), a,
                               -math.sin(a), math.cos(a), 0.5),
                        (args.broadcast, args.port),
                    )
                    sent += 1
                except OSError as exc:
                    print(f"  send failed: {exc}")
                seq += 1
                next_tx += period

            try:
                data, addr = rx.recvfrom(2048)
                pkt = decode(data)
                if pkt is not None and table.ingest(pkt):
                    arrivals.setdefault(pkt["robot_id"], deque(maxlen=2000)).append(
                        time.monotonic()
                    )
                    table.entries[pkt["robot_id"]].source = addr[0]
            except socket.timeout:
                pass

            if now >= next_report:
                report(table, arrivals, sent, args.interval, args.port)
                next_report = now + args.interval
            if args.duration and (now - start) >= args.duration:
                break
    except KeyboardInterrupt:
        pass

    print("\n=== final ===")
    report(table, arrivals, sent, args.interval, args.port)


def report(table, arrivals, sent, interval, port):
    now = time.monotonic()
    print(f"\n[tx {sent}] [kdrops {udp_drops(port)}] "
          f"{'peer':>5} {'from':>15} {'rate':>7} {'loss':>7} "
          f"{'delay_ms':>9} {'max_ms':>8} {'rx':>7} {'age_s':>6}")
    if not table.entries:
        print("      (no peers heard)")
        return
    for robot_id in sorted(table.entries):
        e = table.entries[robot_id]
        recent = [t for t in arrivals.get(robot_id, ()) if now - t <= interval]
        rate = len(recent) / interval
        mean_delay, max_delay = e.delay_stats()
        print(f"       {robot_id:>5} {getattr(e, 'source', '?'):>15} {rate:>6.1f}H "
              f"{e.loss_fraction()*100:>6.1f}% {mean_delay*1000:>9.1f} "
              f"{max_delay*1000:>8.1f} {e.total_received:>7} {e.age(now):>6.2f}")


if __name__ == "__main__":
    main()
