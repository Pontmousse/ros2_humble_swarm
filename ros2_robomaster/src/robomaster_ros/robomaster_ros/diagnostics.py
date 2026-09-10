"""Terminal-only diagnostics for the RoboMaster driver.

Every module in the vendored `robomaster` SDK logs through a single
`logging.getLogger("sdk")`, so one handler observes the whole SDK without
patching site-packages. Connection-lifecycle events are forwarded as they
happen; per-packet events are counted and summarised on a timer, because the
SDK emits them at 50 Hz+ and forwarding each one would itself cause drops.
"""

import logging
import re
import time
from typing import Any, Dict, List, Optional, Tuple

import robomaster
import robomaster.config


PREFIX = 'RM-DIAG'

# Surfaced immediately: one-shot events that explain why a connection did or
# did not come up.
_LIFECYCLE: Tuple[str, ...] = (
    'scan_robot_ip',
    'Cannot found robot based on the specified SN',
    'accept connection',
    'reject connection, service is busy',
    'RECV TimeOut',
    'got config ip',
    'request_connection',
    'initialized with',
    'Connection Failed',
    'unexpected connection param',
    'Start to Recving data',
    'TcpConnection, connect success',
    'UdpConnection, bind',
    'can not create client',
    'Connection Create Failed',
    'ack_register failed',
)

# Counted, never printed per-event. Order matters: first match wins.
_COUNTED: Tuple[Tuple[str, str], ...] = (
    ('recv msg is None, skip', 'recv_none'),
    ('is not in wait_ack_list', 'unmatched_ack'),
    ('is not define ack', 'no_ack_handler'),
    ('is not define req handler', 'no_req_handler'),
    ('send_sync_msg wait msg receiver', 'sync_timeout'),
    ('get resp msg failed', 'sync_no_resp'),
    ('recv_task is not running', 'recv_task_dead'),
    ('Client: send, exception', 'send_exception'),
    ('Connection: send, exception', 'send_exception'),
    ('Connection: recv, exception', 'recv_exception'),
    ('recv buff None', 'recv_buff_none'),
    ('decode_msg is None', 'decode_fail'),
    ('unpack_protocol failed', 'unpack_fail'),
    ('Received unknown res', 'unknown_proto'),
    ('Received unknown req', 'unknown_proto'),
)

_CMD_RE = re.compile(r'cmdset:\s*0?x?([0-9a-fA-F]+).*?cmdid:\s*0?x?([0-9a-fA-F]+)', re.S)
_SET_ID_RE = re.compile(r'cmd set 0x([0-9a-fA-F]+) and id 0x([0-9a-fA-F]+)')


class _SdkLogBridge(logging.Handler):
    """Routes SDK log records into the ROS logger, aggregating the noisy ones."""

    def __init__(self, diag: 'Diagnostics') -> None:
        super().__init__(level=logging.DEBUG)
        self._diag = diag

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = record.getMessage()
        except Exception:
            return
        self._diag.observe(record, msg)


class Diagnostics:
    """Owns the SDK log bridge, the counters and the periodic terminal report."""

    def __init__(self, node: Any, level: str, period: float) -> None:
        self.node = node
        self.level = level
        self.period = period
        self.verbose = (level == 'verbose')

        self._counts: Dict[str, int] = {}
        self._cmd_detail: Dict[str, Dict[str, int]] = {}
        self._unclassified: Dict[str, int] = {}
        self._hb_times: List[float] = []
        self._hb_late = 0
        self._robot_addr: Optional[str] = None
        self._bridge: Optional[_SdkLogBridge] = None
        self._saved_handlers: List[logging.Handler] = []
        self._saved_level: Optional[int] = None
        self._started = time.monotonic()

    # ---------------- install / teardown ----------------

    def install(self) -> None:
        sdk_logger = robomaster.logger
        self._saved_level = sdk_logger.level

        # The SDK ships its own StreamHandler. Leaving it attached while we raise
        # the level would double-print every record in a second format.
        for handler in list(sdk_logger.handlers):
            if isinstance(handler, logging.StreamHandler):
                sdk_logger.removeHandler(handler)
                self._saved_handlers.append(handler)

        self._bridge = _SdkLogBridge(self)
        sdk_logger.addHandler(self._bridge)
        sdk_logger.setLevel(logging.DEBUG if self.verbose else logging.INFO)
        sdk_logger.propagate = False

        self.log(f"diagnostics '{self.level}', reporting every {self.period:g}s. "
                 f"Set the diagnostics parameter to 'off' to disable.")

    def uninstall(self) -> None:
        sdk_logger = robomaster.logger
        if self._bridge is not None:
            sdk_logger.removeHandler(self._bridge)
            self._bridge = None
        for handler in self._saved_handlers:
            sdk_logger.addHandler(handler)
        self._saved_handlers = []
        if self._saved_level is not None:
            sdk_logger.setLevel(self._saved_level)

    # ---------------- ingestion ----------------

    def observe(self, record: logging.LogRecord, msg: str) -> None:
        for needle in _LIFECYCLE:
            if needle in msg:
                where = f'{record.filename}:{record.lineno}'
                self.log(f'[{where}] {msg}', record.levelno)
                return

        for needle, key in _COUNTED:
            if needle in msg:
                self._counts[key] = self._counts.get(key, 0) + 1
                self._note_cmd(key, msg)
                return

        if self.verbose:
            where = f'{record.filename}:{record.lineno}'
            self._unclassified[where] = self._unclassified.get(where, 0) + 1
        elif record.levelno >= logging.ERROR:
            self.log(msg, record.levelno)

    def _note_cmd(self, key: str, msg: str) -> None:
        match = _CMD_RE.search(msg) or _SET_ID_RE.search(msg)
        if not match:
            return
        label = f'0x{int(match.group(1), 16):02x}/0x{int(match.group(2), 16):02x}'
        bucket = self._cmd_detail.setdefault(key, {})
        bucket[label] = bucket.get(label, 0) + 1

    def note_heartbeat(self) -> None:
        self._hb_times.append(time.monotonic())

    def set_robot_addr(self, addr: Optional[str]) -> None:
        self._robot_addr = addr

    # ---------------- reporting ----------------

    def log(self, text: str, levelno: int = logging.INFO) -> None:
        line = f'[{PREFIX}] {text}'
        if levelno >= logging.ERROR:
            self.node.get_logger().error(line)
        elif levelno >= logging.WARNING:
            self.node.get_logger().warn(line)
        else:
            self.node.get_logger().info(line)

    def startup_banner(self, sn: Optional[str], conn_type: str,
                       robot_ip: str, local_ip: str) -> None:
        self.log('===== startup context =====')
        self.log(f'serial_number : {sn}')
        self.log(f'conn_type     : {conn_type}')
        self.log(f'robot_ip param: {robot_ip or "(empty -> broadcast discovery)"}')
        self.log(f'local_ip param: {local_ip or "(empty -> autodetect)"}')
        self.log(f'broadcast port: {robomaster.config.ROBOT_BROADCAST_PORT}')
        for line in _host_networks():
            self.log(f'host iface    : {line}')
        ssid = _wifi_ssid()
        if ssid:
            self.log(f'wifi ssid     : {ssid}')
        self.log('===========================')

    def report(self) -> None:
        """Timer callback: one compact, copy-pasteable block."""
        uptime = time.monotonic() - self._started
        self.log(f'===== {self.period:g}s report (up {uptime:.0f}s) =====')

        self.log(f'link      : {self._link_line()}')
        self.log(f'heartbeat : {self._heartbeat_line()}')

        if self._counts:
            for key in sorted(self._counts):
                detail = self._cmd_detail.get(key)
                suffix = ''
                if detail:
                    top = sorted(detail.items(), key=lambda kv: -kv[1])[:6]
                    suffix = '  {' + ', '.join(f'{k}:{v}' for k, v in top) + '}'
                self.log(f'sdk event : {key}={self._counts[key]}{suffix}')
        else:
            self.log('sdk event : none')

        if self._unclassified:
            top = sorted(self._unclassified.items(), key=lambda kv: -kv[1])[:8]
            self.log('other sdk : ' + ', '.join(f'{k}={v}' for k, v in top))

        self.log('===========================')

        self._counts.clear()
        self._cmd_detail.clear()
        self._unclassified.clear()
        self._hb_times = self._hb_times[-1:]

    def _link_line(self) -> str:
        parts = []
        if self._robot_addr:
            parts.append(f'robot={self._robot_addr}')
            parts.append(f'arp={_arp_state(self._robot_addr)}')
        else:
            parts.append('robot=(unknown)')
        rssi = _wifi_rssi()
        if rssi is not None:
            parts.append(f'rssi={rssi:.0f}dBm')
        ssid = _wifi_ssid()
        if ssid:
            parts.append(f'ssid={ssid}')
        return '  '.join(parts)

    def _heartbeat_line(self) -> str:
        times = self._hb_times
        if len(times) < 2:
            return f'n={len(times)} (no interval yet)'
        gaps = [b - a for a, b in zip(times, times[1:])]
        late = sum(1 for g in gaps if g > 1.0)
        self._hb_late += late
        return (f'n={len(gaps)} mean={sum(gaps)/len(gaps):.2f}s '
                f'max={max(gaps):.2f}s late(>1s)={late}')


# ---------------- host-side probes (no subprocesses, no root) ----------------

def _host_networks() -> List[str]:
    out = []
    try:
        import socket as _socket
        with open('/proc/net/route') as handle:
            handle.readline()
        # Addresses come from getifaddrs via socket; fall back quietly.
        import fcntl
        import struct
        for name in _iface_names():
            sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            try:
                packed = fcntl.ioctl(sock.fileno(), 0x8915,
                                     struct.pack('256s', name[:15].encode()))
                addr = _socket.inet_ntoa(packed[20:24])
                out.append(f'{name} {addr}')
            except OSError:
                pass
            finally:
                sock.close()
    except Exception:
        pass
    return out


def _iface_names() -> List[str]:
    try:
        with open('/proc/net/dev') as handle:
            lines = handle.readlines()[2:]
        return [line.split(':', 1)[0].strip() for line in lines
                if ':' in line and not line.split(':', 1)[0].strip().startswith('lo')]
    except Exception:
        return []


def _arp_state(ip: str) -> str:
    """Reads /proc/net/arp. Flags 0x2 means the entry is complete."""
    try:
        with open('/proc/net/arp') as handle:
            for line in handle.readlines()[1:]:
                fields = line.split()
                if len(fields) >= 4 and fields[0] == ip:
                    return 'PRESENT' if fields[2] == '0x2' else f'STALE({fields[2]})'
    except Exception:
        return 'unknown'
    return 'ABSENT(robot not on LAN)'


def _wifi_rssi() -> Optional[float]:
    try:
        with open('/proc/net/wireless') as handle:
            for line in handle.readlines()[2:]:
                fields = line.split()
                if len(fields) >= 4:
                    return float(fields[3].rstrip('.'))
    except Exception:
        return None
    return None


def _wifi_ssid() -> Optional[str]:
    """Reads the SSID via SIOCGIWESSID so no subprocess is needed."""
    try:
        import fcntl
        import socket as _socket
        import struct
        for name in _iface_names():
            sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            buf = bytearray(32)
            try:
                request = struct.pack('16sPHH', name[:15].encode(),
                                      *_buffer_args(buf))
                fcntl.ioctl(sock.fileno(), 0x8B1B, request)
                ssid = buf.split(b'\x00', 1)[0].decode(errors='replace')
                if ssid:
                    return ssid
            except OSError:
                pass
            finally:
                sock.close()
    except Exception:
        return None
    return None


def _buffer_args(buf: bytearray) -> Tuple[int, int, int]:
    import ctypes
    return (ctypes.addressof(ctypes.c_char.from_buffer(buf)), len(buf), 0)
