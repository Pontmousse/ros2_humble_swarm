#!/usr/bin/env python3
"""Periodic UDP state broadcast plus neighbour table.

Transmit and receive share one node so there is a single socket lifetime and no
cross-thread table locking: the receive side is drained from a timer, not a
thread.
"""
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from swarm_interfaces.msg import CoordXY, NeighbourState, NeighbourStates, Neighbours
from transforms3d.euler import quat2euler

from swarm_comms.packet import DEFAULT_BROADCAST, DEFAULT_PORT, PACKET_SIZE, decode, encode
from swarm_comms.table import NeighbourTable


class StateBroadcaster(Node):
    def __init__(self):
        super().__init__("state_broadcaster")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("broadcast_address", DEFAULT_BROADCAST)
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("broadcast_rate", 10.0)
        self.declare_parameter("receive_rate", 50.0)
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("drop_timeout", 5.0)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("stats_rate", 1.0)
        self.declare_parameter("bind_device", "")
        self.declare_parameter("predict", True)

        self.robot_id = self.get_parameter("robot_id").value
        self.broadcast_address = self.get_parameter("broadcast_address").value
        self.port = self.get_parameter("port").value
        self.odom_timeout = self.get_parameter("odom_timeout").value
        bind_device = self.get_parameter("bind_device").value
        self.predict = self.get_parameter("predict").value

        self.stale_timeout = self.get_parameter("stale_timeout").value
        self.table = NeighbourTable(
            own_id=self.robot_id,
            stale_timeout=self.stale_timeout,
            drop_timeout=self.get_parameter("drop_timeout").value,
        )

        self.seq = 0
        self.last_odom = None
        self.last_odom_time = 0.0
        self.tx_count = 0
        self.rx_count = 0
        self.rx_dropped = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Odometry, "localization/odom", self.odom_callback, qos)

        self.states_pub = self.create_publisher(NeighbourStates, "comms/neighbours", qos)
        # Legacy shape, so the existing guidance law can be switched over with a
        # remap instead of a code change.
        self.legacy_pub = self.create_publisher(Neighbours, "comms/nbh_pos", qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, "comms/diagnostics", 10)

        self._open_sockets(bind_device)

        self.create_timer(1.0 / self.get_parameter("broadcast_rate").value, self.tx_callback)
        self.create_timer(1.0 / self.get_parameter("receive_rate").value, self.rx_callback)
        self.create_timer(1.0 / self.get_parameter("stats_rate").value, self.stats_callback)

        self.get_logger().info(
            f"robot_id={self.robot_id} broadcasting {PACKET_SIZE}B to "
            f"{self.broadcast_address}:{self.port}"
        )

    def _open_sockets(self, bind_device):
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if bind_device:
            # With RNDIS up, the route table may pick usb0 for a broadcast and
            # the packet never reaches the other Pis.
            self.tx_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_BINDTODEVICE, bind_device.encode()
            )

        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(("", self.port))  # INADDR_ANY; a pinned IP misses broadcasts
        self.rx_sock.setblocking(False)

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.last_odom_time = time.monotonic()

    def tx_callback(self):
        fresh = (
            self.last_odom is not None
            and (time.monotonic() - self.last_odom_time) < self.odom_timeout
        )

        if fresh:
            p = self.last_odom.pose.pose
            t = self.last_odom.twist.twist
            q = p.orientation
            yaw = quat2euler([q.w, q.x, q.y, q.z])[2]
            x, y = p.position.x, p.position.y
            # REP-105: Odometry.pose is in header.frame_id (swarm_map) but
            # Odometry.twist is in child_frame_id (<ns>/base_link). Checked
            # against finite-differenced pose on bag 20260910_200332 (857
            # samples, mean speed 0.262 m/s): residual is 0.067 m/s if the twist
            # is body-frame and rotated, 0.341 m/s if taken as map-frame —
            # larger than the speed itself. Rotate here so the packet is wholly
            # map-frame and receivers can dead-reckon by adding v*dt to (x, y)
            # without knowing the sender's yaw.
            c, s = math.cos(yaw), math.sin(yaw)
            vx = t.linear.x * c - t.linear.y * s
            vy = t.linear.x * s + t.linear.y * c
            omega = t.angular.z  # yaw rate is frame-invariant about a shared z
        else:
            x = y = yaw = vx = vy = omega = 0.0

        # Keep sending when localization is stale: peers need the liveness and
        # the sequence continuity to distinguish "lost" from "unlocalized".
        pkt = encode(self.robot_id, self.seq, time.time(),
                     x, y, yaw, vx, vy, omega, pose_valid=fresh)
        try:
            self.tx_sock.sendto(pkt, (self.broadcast_address, self.port))
            self.tx_count += 1
        except OSError as exc:
            self.get_logger().warning(f"broadcast failed: {exc}", throttle_duration_sec=5.0)

        self.seq += 1

    def rx_callback(self):
        # Bounded drain: a burst must not stall the executor.
        for _ in range(256):
            try:
                data, _addr = self.rx_sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError:
                break
            pkt = decode(data)
            if pkt is None:
                self.rx_dropped += 1
                continue
            if self.table.ingest(pkt):
                self.rx_count += 1

        self.table.expire()
        self.publish_table()

    def publish_table(self):
        now = time.monotonic()
        msg = NeighbourStates()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "swarm_map"

        legacy = Neighbours()
        for entry in self.table.entries.values():
            # Published pose is dead-reckoned to now, so consumers can treat it
            # as current rather than having to apply `age` themselves.
            if self.predict:
                x, y, yaw = entry.predicted(now, self.stale_timeout)
            else:
                x, y, yaw = entry.x, entry.y, entry.yaw

            state = NeighbourState()
            state.robot_id = entry.robot_id
            state.x, state.y, state.yaw = x, y, yaw
            state.vx, state.vy, state.omega = entry.vx, entry.vy, entry.omega
            state.age = entry.age(now)
            state.valid = entry.valid and entry.pose_valid
            msg.neighbours.append(state)

            if state.valid:
                coord = CoordXY()
                coord.x, coord.y = x, y
                legacy.neighbours.append(coord)

        self.states_pub.publish(msg)
        self.legacy_pub.publish(legacy)

    def stats_callback(self):
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()

        for entry in self.table.entries.values():
            mean_delay, max_delay = entry.delay_stats()
            status = DiagnosticStatus()
            status.name = f"swarm_comms/link/robot_{entry.robot_id}"
            status.hardware_id = str(entry.robot_id)
            loss = entry.loss_fraction()
            if not entry.valid:
                status.level = DiagnosticStatus.ERROR
                status.message = "stale"
            elif loss > 0.2:
                status.level = DiagnosticStatus.WARN
                status.message = "lossy"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "ok"
            status.values = [
                KeyValue(key="loss_fraction", value=f"{loss:.4f}"),
                KeyValue(key="received", value=str(entry.total_received)),
                KeyValue(key="last_seq", value=str(entry.last_seq)),
                KeyValue(key="age_s", value=f"{entry.age(now):.3f}"),
                KeyValue(key="delay_mean_s", value=f"{mean_delay:.4f}"),
                KeyValue(key="delay_max_s", value=f"{max_delay:.4f}"),
                KeyValue(key="pose_valid", value=str(entry.pose_valid)),
            ]
            diag.status.append(status)

        own = DiagnosticStatus()
        own.name = "swarm_comms/self"
        own.hardware_id = str(self.robot_id)
        own.level = DiagnosticStatus.OK
        own.values = [
            KeyValue(key="tx_count", value=str(self.tx_count)),
            KeyValue(key="rx_count", value=str(self.rx_count)),
            KeyValue(key="rx_malformed", value=str(self.rx_dropped)),
            KeyValue(key="neighbours", value=str(len(self.table.entries))),
        ]
        diag.status.append(own)

        self.diag_pub.publish(diag)

    def destroy_node(self):
        self.tx_sock.close()
        self.rx_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
