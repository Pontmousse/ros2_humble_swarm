# flake8: noqa: E402
import logging
import random
import time


import rclpy.logging
import rclpy.executors

# Disabled because it too expensive
# robomaster.logger = rclpy.logging.get_logger('sdk')

import robomaster
import robomaster.robot
import robomaster.protocol
import robomaster.conn
import robomaster.client
import robomaster.config


from robomaster_ros.modules import modules
from robomaster_ros.ftp import FtpConnection
from robomaster_ros import diagnostics

import rclpy
import rclpy.node
import rclpy.action
import rclpy.task
import rclpy.duration

import std_msgs.msg
import sensor_msgs.msg

import tf2_ros.transform_broadcaster

from typing import Any, Optional


SERIAL_NUMBER_LENGTH = 14

def pad_serial(value: str) -> str:
  value = value[:SERIAL_NUMBER_LENGTH]
  value += '*' * (SERIAL_NUMBER_LENGTH - len(value))
  return value


def add_unknown_protocol(cmdset: int, cmdid: int, hint: str = '?') -> None:
    def unpack_resp(self: Any, buf: bytes, offset: int = 0) -> None:
        robomaster.logger.debug(
            f'[{hint}] Received unknown response with cmd set {cmdset:#x} and id {cmdid:#x}, '
            f'with buffer {buf!r} ({len(buf)})')

    def unpack_req(self: Any, buf: bytes, offset: int = 0) -> None:
        robomaster.logger.debug(
            f'[{hint}] Received unknown request with cmd set {cmdset:#x} and id {cmdid:#x}, '
            f'with buffer {buf!r} ({len(buf)})')

    _ = type(f'UnknownProtocol_{cmdset}_{cmdid}',
             (robomaster.protocol.ProtoData, ),
             {'_cmdset': cmdset,
              '_cmdid': cmdid,
              'unpack_resp': unpack_resp,
              'unpack_req': unpack_req})


def add_unknown_protocols() -> None:
    # state change
    add_unknown_protocol(0x3f, 0x29, 'CHASSIS STATE')
    # related to uart
    add_unknown_protocol(0x3f, 0xf4, 'SENSOR ADC')
    # related to tof
    add_unknown_protocol(0x24, 0x21, 'TOF')


add_unknown_protocols()

# add_unknown_protocol(0x3f, 0xb3)


_orig_client_stop = robomaster.client.Client.stop


def _safe_client_stop(self: Any) -> None:
    # robomaster/client.py:132 does `if self._thread.is_alive():` with no
    # None-guard. A Client whose start() failed/returned False before
    # spawning the recv thread leaves _thread as None -- typically the
    # discarded ep_robot from a failed attempt in _connect_with_retries,
    # which stop() already closed once via the except handler there. When
    # Python later garbage-collects that orphaned object, Robot.__del__ /
    # Client.__del__ call stop() again, and self._thread.is_alive() raises
    # AttributeError. Since that happens inside __del__, Python can't
    # propagate it -- it just prints "Exception ignored in: ...__del__"
    # and the traceback, which is harmless noise but looks alarming.
    # Guard it so it no-ops (still closing the connection) instead.
    if getattr(self, '_thread', None) is None:
        conn = getattr(self, '_conn', None)
        if conn:
            try:
                conn.close()
            except Exception:
                pass
        return
    _orig_client_stop(self)


robomaster.client.Client.stop = _safe_client_stop


def _remote_ip(ep_robot: Any) -> Optional[str]:
    try:
        return ep_robot._client.remote_addr[0]
    except Exception:
        return robomaster.config.ROBOT_IP_STR or None


def _explain(e: Exception) -> str:
    # robomaster/client.py:95 does `raise print(...)`, so a client with no
    # connection object surfaces as `raise None` instead of the real reason.
    if isinstance(e, TypeError) and 'derive from BaseException' in str(e):
        return (" -- the SDK never built a connection: the handshake on UDP "
                f"{robomaster.config.ROBOT_PROXY_PORT} got no usable reply. "
                "Look for 'RECV TimeOut' (nothing answered) or "
                "'reject connection, service is busy' (another client holds "
                "the robot) above.")
    return ''


def wait_for_robot(serial_number: Optional[str], logger: Any = None) -> None:
    found = False
    attempts = 0
    while not found:
        try:
            found = robomaster.conn.scan_robot_ip(user_sn=serial_number)
        except OSError:
            pass
        if not found:
            attempts += 1
            if logger and attempts % 5 == 0:
                logger.warn(
                    f"No SN broadcast on UDP {robomaster.config.ROBOT_BROADCAST_PORT} "
                    f"after {attempts} scans. The robot must be powered on, in sta mode, "
                    f"and on a network that forwards broadcast to this host. "
                    f"Set the robot_ip parameter to skip discovery.")
            time.sleep(random.uniform(1.0, 2.0))


class RoboMasterROS(rclpy.node.Node):  # type: ignore

    initialized: bool = False
    diagnostics: Any = None

    def __init__(self, executor: Optional[rclpy.executors.Executor] = None) -> None:
        super(RoboMasterROS, self).__init__("robomaster_ros", start_parameter_services=True)
        # robomaster.logger.set_level(logging.ERROR)
        lib_log_level : str = self.declare_parameter("lib_log_level", "ERROR").value.upper()
        robomaster.logger.setLevel(lib_log_level)
        conn_type: str = self.declare_parameter("conn_type", "sta").value[:]
        self.reconnect: bool = self.declare_parameter("reconnect", True).value
        self.connection_attempts: int = self.declare_parameter("connection_attempts", 5).value
        self.connection_retry_delay: float = self.declare_parameter(
            "connection_retry_delay", 2.0).value
        robot_ip: str = self.declare_parameter("robot_ip", "").value
        local_ip: str = self.declare_parameter("local_ip", "").value
        if robot_ip:
            robomaster.config.ROBOT_IP_STR = robot_ip
        if local_ip:
            robomaster.config.LOCAL_IP_STR = local_ip
        sn: Optional[str] = self.declare_parameter("serial_number", "").value
        if sn:
            if len(sn) != SERIAL_NUMBER_LENGTH:
                sn = pad_serial(sn)
                self.get_logger().warn(
                    f"Serial number must have length {SERIAL_NUMBER_LENGTH}: "
                    f"trasformed to {sn}")
        else:
            sn = None

        # Installed before discovery so the handshake itself is instrumented.
        diagnostics_level: str = self.declare_parameter(
            "diagnostics", "off").value.lower()
        diagnostics_period: float = self.declare_parameter(
            "diagnostics_period", 10.0).value
        self.diagnostics: Optional[diagnostics.Diagnostics] = None
        if diagnostics_level != 'off':
            self.diagnostics = diagnostics.Diagnostics(
                self, diagnostics_level, diagnostics_period)
            self.diagnostics.install()
            self.diagnostics.startup_banner(sn, conn_type, robot_ip, local_ip)

        self.connected = False
        if conn_type == 'sta' and not robomaster.config.ROBOT_IP_STR:
            self.get_logger().info("Waiting for a robot")
            wait_for_robot(sn, self.get_logger())
            self.get_logger().info("Found a robot")
        robomaster.conn.FtpConnection = FtpConnection
        # robomaster.conn.FtpConnection = FakeFtpConnection
        self.disconnection = rclpy.task.Future(executor=executor or rclpy.get_global_executor())
        # For now, to handle simulations without FTP
        if not self._connect_with_retries(conn_type, sn):
            self.disconnection.set_result(False)
            return
        self.get_logger().info("Connected")
        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.connected_pub = self.create_publisher(std_msgs.msg.Bool, 'connected', qos)
        self.connected = True
        self._tf_name = self.declare_parameter('tf_prefix', '').value
        self.initialized = True
        self.heartbeat_check_timer = self.create_timer(5, self.heartbeat_check)
        self.heartbeat_handler = robomaster.client.MsgHandler(
            proto_data=robomaster.protocol.ProtoSdkHeartBeat(),
            ack_cb=lambda _, msg: self.got_heart_beat(msg))

        self.ep_robot._client.add_msg_handler(self.heartbeat_handler)
        if self.diagnostics:
            self.diagnostics.set_robot_addr(_remote_ip(self.ep_robot))
            self.diagnostics_timer = self.create_timer(
                diagnostics_period, self.diagnostics.report)
        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, 'joint_states_p', 1)
        self.tf_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
        self.modules = [module(self.ep_robot, self) for name, module in
                        modules.items() if self.enabled(name)]
        module_string = ', '.join(type(module).__name__ for module in self.modules)
        self.get_logger().info(f"Enabled modules: {module_string}")
        self.connected_pub.publish(std_msgs.msg.Bool(data=True))

    def _connect_with_retries(self, conn_type: str, sn: Optional[str]) -> bool:
        # conn.switch_remote_route sends the handshake as a single datagram with no
        # retry, so one lost packet on a marginal link fails the whole connection.
        attempts = max(1, self.connection_attempts)
        for attempt in range(1, attempts + 1):
            self.get_logger().info(
                f"Try to connect via {conn_type} to robot with sn {sn} "
                f"({attempt}/{attempts})")
            self.ep_robot = robomaster.robot.Robot()
            try:
                self.ep_robot.initialize(conn_type=conn_type, sn=sn)
                return True
            except Exception as e:
                self.get_logger().warn(
                    f"Connection attempt {attempt}/{attempts} failed: "
                    f"{type(e).__name__}: {e}{_explain(e)}")
                try:
                    self.ep_robot.close()
                except Exception:
                    pass
                if attempt < attempts:
                    time.sleep(self.connection_retry_delay)
        self.get_logger().error(f"Could not connect after {attempts} attempts")
        return False

    def __del__(self) -> None:
        self.stop()

    def abort(self) -> None:
        if self.initialized:
            self.get_logger().info("Will abort any action")
            for module in self.modules:
                module.abort()

    def stop(self) -> None:
        if self.diagnostics:
            self.diagnostics.report()
            self.diagnostics.uninstall()
            self.diagnostics = None
        if self.initialized:
            self.get_logger().info("Will stop client")
            self.heartbeat_check_timer.cancel()
            for module in self.modules:
                # self.get_logger().info(f"Will stop module {module}")
                module.stop()
                # self.get_logger().info(f"Stopped module {module}")
            time.sleep(0.5)
            if not self.connected:
                self.ep_robot._client.stop()
            self.ep_robot.close()
            self.connected = False
            self.connected_pub.publish(std_msgs.msg.Bool(data=False))
            self.initialized = False
            self.get_logger().info("Has stopped client")

    def tf_frame(self, name: str) -> str:
        if self._tf_name:
            return f'{self._tf_name}/{name}'
        return name

    def enabled(self, name: str) -> bool:
        return self.declare_parameter(f"{name}.enabled", False).value

    def heartbeat_check(self) -> None:
        self.connected = False
        self.disconnection.set_result(False)
        self.get_logger().warn("Disconnected")

    def got_heart_beat(self, msg: Any) -> None:
        if self.diagnostics:
            self.diagnostics.note_heartbeat()
        self.heartbeat_check_timer.reset()
