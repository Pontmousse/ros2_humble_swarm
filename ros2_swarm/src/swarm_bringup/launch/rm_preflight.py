#!/usr/bin/env python3
"""Pre-launch connectivity report for the RoboMaster driver.

Run by the swarm launch files as an ExecuteProcess before the drivers start, so
the terminal shows the state of the network *before* the SDK touches it. Every
check is read-only: nothing here opens an SDK session, because a half-open
session is one of the failure modes being diagnosed.

Usage: rm_preflight.py [--listen SECONDS] [robot_ip ...]
"""

import socket
import struct
import sys
import time

BROADCAST_PORT = 40927
PROXY_PORT = 30030
DEVICE_PORT = 20020
PREFIX = '[RM-PRE]'


def say(text=''):
    print(f'{PREFIX} {text}', flush=True)


def head(text):
    say()
    say(f'--- {text} ---')


# ---------------- host network state ----------------

def iface_names():
    try:
        with open('/proc/net/dev') as handle:
            lines = handle.readlines()[2:]
    except OSError:
        return []
    names = [line.split(':', 1)[0].strip() for line in lines if ':' in line]
    return [n for n in names if n and n != 'lo']


def iface_addr(name):
    import fcntl
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = fcntl.ioctl(sock.fileno(), 0x8915,
                             struct.pack('256s', name[:15].encode()))
        return socket.inet_ntoa(packed[20:24])
    except OSError:
        return None
    finally:
        sock.close()


def wifi_ssid():
    import ctypes
    import fcntl
    for name in iface_names():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        buf = bytearray(32)
        try:
            addr = ctypes.addressof(ctypes.c_char.from_buffer(buf))
            request = struct.pack('16sPHH', name[:15].encode(), addr, len(buf), 0)
            fcntl.ioctl(sock.fileno(), 0x8B1B, request)
            ssid = buf.split(b'\x00', 1)[0].decode(errors='replace')
            if ssid:
                return f'{ssid} (on {name})'
        except OSError:
            pass
        finally:
            sock.close()
    return None


def wifi_rssi():
    try:
        with open('/proc/net/wireless') as handle:
            for line in handle.readlines()[2:]:
                fields = line.split()
                if len(fields) >= 4:
                    return f'{fields[0].rstrip(":")} {fields[3].rstrip(".")} dBm'
    except OSError:
        pass
    return None


def routes():
    """Parses /proc/net/route into (iface, dest, mask, gateway) with dotted quads."""
    out = []
    try:
        with open('/proc/net/route') as handle:
            for line in handle.readlines()[1:]:
                fields = line.split()
                if len(fields) < 8:
                    continue
                out.append((fields[0],
                            _hex_ip(fields[1]),
                            _hex_ip(fields[7]),
                            _hex_ip(fields[2])))
    except OSError:
        pass
    return out


def _hex_ip(value):
    return socket.inet_ntoa(struct.pack('<L', int(value, 16)))


def route_for(ip):
    """Longest-prefix match against the kernel table, without shelling out."""
    try:
        target = struct.unpack('!L', socket.inet_aton(ip))[0]
    except OSError:
        return None
    best = None
    best_bits = -1
    for iface, dest, mask, gateway in routes():
        m = struct.unpack('!L', socket.inet_aton(mask))[0]
        d = struct.unpack('!L', socket.inet_aton(dest))[0]
        if (target & m) == d:
            bits = bin(m).count('1')
            if bits > best_bits:
                best_bits = bits
                via = 'direct' if gateway == '0.0.0.0' else f'via {gateway}'
                best = f'dev {iface} {via}'
    return best


def arp_state(ip):
    """Flags 0x2 in /proc/net/arp means the entry is complete."""
    try:
        with open('/proc/net/arp') as handle:
            for line in handle.readlines()[1:]:
                fields = line.split()
                if len(fields) >= 4 and fields[0] == ip:
                    if fields[2] == '0x2':
                        return f'PRESENT mac={fields[3]}'
                    return f'STALE(flags={fields[2]})'
    except OSError:
        return 'unreadable'
    return 'ABSENT (no recent L2 contact)'


# ---------------- port ownership ----------------

def udp_listeners(port):
    """Returns [(local_addr, inode)] from /proc/net/udp for the given port."""
    found = []
    for path in ('/proc/net/udp', '/proc/net/udp6'):
        try:
            with open(path) as handle:
                lines = handle.readlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 10:
                continue
            local = fields[1]
            if ':' not in local:
                continue
            hex_addr, hex_port = local.rsplit(':', 1)
            if int(hex_port, 16) != port:
                continue
            found.append((_pretty_addr(hex_addr, path), fields[9]))
    return found


def _pretty_addr(hex_addr, path):
    try:
        if path.endswith('udp6'):
            return f'[v6:{hex_addr}]'
        return socket.inet_ntoa(struct.pack('<L', int(hex_addr, 16)))
    except (OSError, ValueError):
        return hex_addr


def pid_for_inode(inode):
    """Scans /proc/*/fd for the socket inode. Only our own processes are visible."""
    import glob
    import os
    target = f'socket:[{inode}]'
    for fd_path in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            if os.readlink(fd_path) != target:
                continue
            pid = fd_path.split('/')[2]
            with open(f'/proc/{pid}/cmdline', 'rb') as handle:
                cmd = handle.read().replace(b'\x00', b' ').decode(errors='replace')
            return f'pid {pid} ({" ".join(cmd.split())[:90]})'
        except OSError:
            continue
    return None


# ---------------- reachability ----------------

def probe_udp(ip, port):
    """Sends one empty datagram on a connected socket to surface ICMP errors.

    A connected UDP socket reports ICMP port-unreachable as ECONNREFUSED on the
    next operation, which distinguishes 'host up, nothing listening' from
    'nothing answered at all'. No SDK payload is sent, so the robot cannot open
    a session as a side effect.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    try:
        sock.connect((ip, port))
        sock.send(b'')
        time.sleep(0.3)
        try:
            sock.recv(1024)
            return 'REPLIED (something is listening)'
        except socket.timeout:
            return 'no reply (open|filtered|ignoring empty datagrams)'
        except ConnectionRefusedError:
            return 'ICMP port-unreachable (host up, port CLOSED)'
    except ConnectionRefusedError:
        return 'ICMP port-unreachable (host up, port CLOSED)'
    except OSError as exc:
        return f'send failed: {exc}'
    finally:
        sock.close()


def listen_for_broadcast(seconds):
    """Passive SN-broadcast listener: the same signal scan_robot_ip waits for.

    Answers the one question the driver's 'scan_robot_ip: exception timed out'
    cannot: is the robot broadcasting at all, or is the broadcast not reaching
    this host?
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', BROADCAST_PORT))
    except OSError as exc:
        say(f'cannot bind {BROADCAST_PORT}: {exc}')
        say('  -> another process holds the discovery port; see the port section above')
        sock.close()
        return
    sock.settimeout(0.5)
    deadline = time.monotonic() + seconds
    seen = {}
    while time.monotonic() < deadline:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError as exc:
            say(f'recv failed: {exc}')
            break
        text = data.decode(errors='replace').strip()
        key = (addr[0], text)
        if key not in seen:
            seen[key] = 0
            say(f'broadcast from {addr[0]}: {text!r}')
        seen[key] += 1
    sock.close()
    if seen:
        for (ip, text), count in seen.items():
            say(f'total: {ip} sn={text!r} x{count}')
        say('-> discovery should work; set robot_ip to the address above to skip it')
    else:
        say(f'NOTHING received on UDP {BROADCAST_PORT} in {seconds:g}s')
        say('-> robot off, not in sta mode, on another subnet, or broadcast is filtered')


# ---------------- main ----------------

def main(argv):
    listen = 0.0
    ips = []
    index = 1
    while index < len(argv):
        if argv[index] == '--listen':
            listen = float(argv[index + 1])
            index += 2
            continue
        if argv[index]:
            ips.append(argv[index])
        index += 1

    say('===== preflight =====')

    head('host')
    ssid = wifi_ssid()
    say(f'ssid      : {ssid or "(not wifi / unavailable)"}')
    rssi = wifi_rssi()
    say(f'rssi      : {rssi or "(unavailable)"}')
    for name in iface_names():
        addr = iface_addr(name)
        if addr:
            say(f'iface     : {name} {addr}')
    for iface, dest, mask, gateway in routes():
        if dest == '0.0.0.0':
            say(f'default   : dev {iface} via {gateway}')

    head(f'discovery port {BROADCAST_PORT}')
    holders = udp_listeners(BROADCAST_PORT)
    if not holders:
        say('free')
    for local, inode in holders:
        owner = pid_for_inode(inode) or 'owner not visible (another user/container)'
        say(f'HELD by {owner} bound {local}')
        say('  -> a leftover driver here would silently eat the SN broadcast')

    if ips:
        for ip in ips:
            head(f'robot {ip}')
            say(f'route     : {route_for(ip) or "NO ROUTE (wrong subnet)"}')
            say(f'arp       : {arp_state(ip)}')
            say(f'udp {PROXY_PORT:<5}: {probe_udp(ip, PROXY_PORT)}   (SDK handshake)')
            say(f'udp {DEVICE_PORT:<5}: {probe_udp(ip, DEVICE_PORT)}   (SDK data)')
    else:
        head('robot ip')
        say('no robot_ip pinned; the driver will use SN broadcast discovery')

    if listen > 0:
        head(f'passive SN broadcast, {listen:g}s')
        listen_for_broadcast(listen)

    say()
    say('===== preflight done =====')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
