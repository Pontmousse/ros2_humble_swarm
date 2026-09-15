
Yes — that is exactly what you should do, but I would not hardcode static IPs directly on the Pis unless you have a special reason.
The cleaner setup is DHCP reservations on the Netgear router. Each robot still uses DHCP normally, but the router permanently maps that Pi’s Wi-Fi MAC address to the same IP every time. Netgear explicitly supports this under Address Reservation / Static DHCP Leases, depending on model. Netgear KB
I would organize the fleet something like:

Router           192.168.1.1

Registry/server  192.168.1.10

robot01          192.168.1.101
robot02          192.168.1.102
robot03          192.168.1.103
robot04          192.168.1.104
...
robot10          192.168.1.110

Then your Ansible inventory becomes permanently predictable:

[robots]
robot01 ansible_host=192.168.1.101
robot02 ansible_host=192.168.1.102
robot03 ansible_host=192.168.1.103
robot04 ansible_host=192.168.1.104

[registry]
registry ansible_host=192.168.1.10

On most Netgear routers
Connect to the router and go to:

routerlogin.net
    ↓
ADVANCED
    ↓
Setup
    ↓
LAN Setup
    ↓
Address Reservation

Then for each robot:

Device name: robot01
MAC address: AA:BB:CC:DD:EE:01
IP address:  192.168.1.101

Click Add / Apply.
Repeat for each Pi. Netgear notes that the new reserved address takes effect when the client next contacts DHCP, so usually rebooting or reconnecting the Pi is sufficient. Netgear KB
Some newer/business Netgear models call this:

LAN → Static DHCP Leases

instead, but conceptually it is exactly the same thing. Netgear KB
I would also separate your address ranges
For example:

192.168.1.1       router

192.168.1.10-19   infrastructure
                  registry
                  development laptop
                  other servers

192.168.1.100-149 robots
                  robot01 = .101
                  robot02 = .102
                  ...
                  
192.168.1.200-240 ordinary DHCP clients
                  phones
                  laptops
                  temporary devices

Netgear allows you to configure the DHCP pool range under the same LAN setup area. Netgear KB
So you could make the automatic DHCP pool:

Start: 192.168.1.200
End:   192.168.1.240

and keep:

.10-.19
.100-.149

for your controlled infrastructure and robot addresses.
That removes a whole class of accidental IP conflicts.
One important Raspberry Pi detail
Make sure you're reserving the MAC address of the interface you're actually using.
If the robots connect over Wi-Fi:

ip link show wlan0

or:

cat /sys/class/net/wlan0/address

You'll get something like:

dc:a6:32:12:34:56

That is the MAC you put into the Netgear reservation.
For Ethernet it would instead be:

cat /sys/class/net/eth0/address

Also, don't enable randomized/private MAC addresses on the Pi Wi-Fi connection. If the MAC changed, the router would no longer recognize that Pi as robot01.
Then you can make the whole fleet deterministic
You end up with:

robot01
 MAC dc:a6:32:xx:xx:01
 IP  192.168.1.101

robot02
 MAC dc:a6:32:xx:xx:02
 IP  192.168.1.102

robot03
 MAC dc:a6:32:xx:xx:03
 IP  192.168.1.103

and preferably also set the OS hostname:

sudo hostnamectl set-hostname robot01

Then your infrastructure starts looking very clean:

Wi-Fi LAN
│
├── registry   192.168.1.10
│
├── robot01    192.168.1.101
├── robot02    192.168.1.102
├── robot03    192.168.1.103
├── robot04    192.168.1.104
└── ...

For your use case, Netgear DHCP reservations + predictable hostnames + Ansible inventory is the approach I'd choose. It gives you effectively static networking without having to manually configure DNS, gateway, subnet mask, and static networking on every Pi.



Yes. For the RoboMaster, you can identify both the current IP and its Wi-Fi MAC address without SSH.
The easiest method is actually through the Netgear router. Power on one RoboMaster at a time, connect it to the router in networking/router mode, then open:

Netgear → BASIC → Attached Devices

Netgear shows the connected device's IP address and MAC address there. Netgear KB
So a very practical fleet setup procedure is:

Turn on RoboMaster #1 only
        ↓
Connect it to the Netgear Wi-Fi
        ↓
Refresh Attached Devices
        ↓
Find the newly appearing device
        ↓
Record its MAC address
        ↓
Reserve:
robot01-rm → 192.168.1.151
        ↓
Turn it off
        ↓
Repeat for RoboMaster #2

I would give the Pis and RoboMasters separate ranges, for example:

Infrastructure
192.168.1.10      registry

Raspberry Pis
192.168.1.101     robot01-pi
192.168.1.102     robot02-pi
192.168.1.103     robot03-pi

RoboMasters
192.168.1.151     robot01-rm
192.168.1.152     robot02-rm
192.168.1.153     robot03-rm

That becomes extremely convenient later.
There is also an official RoboMaster way to get its IP
DJI specifically designed the RoboMaster SDK for this situation.
In Wi-Fi networking mode, DJI says the router assigns the RoboMaster's IP dynamically through DHCP. The robot then broadcasts its IP address on UDP port 40926. GitHub
So from your laptop or Pi on the same LAN, this tiny Python script can discover it:

import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 40926))

print("Waiting for RoboMaster...")

while True:
    data, addr = sock.recvfrom(1024)
    print("RoboMaster:", addr[0], data)

DJI's documentation actually recommends this exact mechanism for discovering a RoboMaster in router/networking mode. RoboMaster Developer Guide
The RoboMaster App can also show the LAN IP under:

Settings
   ↓
Connection
   ↓
Robot IP

when it is connected in networking mode. RoboMaster Developer Guide
Once you know the IP, finding the MAC from Linux is easy
Suppose discovery gives:

RoboMaster = 192.168.1.173

From your Pi/laptop:

ping -c 1 192.168.1.173
ip neigh show 192.168.1.173

You may get:

192.168.1.173 dev wlan0 lladdr 60:60:1f:ab:cd:ef REACHABLE

So:

IP  = 192.168.1.173
MAC = 60:60:1f:ab:cd:ef

You can also use:

arp -n 192.168.1.173

or scan the subnet:

sudo nmap -sn 192.168.1.0/24

An interesting detail about the RoboMaster
The robot has different addresses depending on connection mode.
DJI documents:

Wi-Fi direct mode:
192.168.2.1

USB/RNDIS:
192.168.42.2

Router/networking mode:
DHCP address assigned by your router
``` citeturn860719search0turn860719search1


So **192.168.2.1 is not the address you want to reserve**. That's its direct Wi-Fi hotspot address.

For your setup, you care about its **station-mode Wi-Fi interface MAC**, the one visible to the Netgear router.

### I would definitely reserve the RoboMasters too

Then your experiment infrastructure becomes deterministic:

```text
                   Netgear
                      │
          ┌───────────┼────────────┐
          │           │            │
       robot01     robot02      robot03
       │    │      │    │       │    │
       │    │      │    │       │    │
      Pi    RM     Pi    RM      Pi    RM
     .101  .151   .102  .152    .103  .153

Then software can know ahead of time:

robot01:
  computer: 192.168.1.101
  robomaster: 192.168.1.151

robot02:
  computer: 192.168.1.102
  robomaster: 192.168.1.152

And that should help a lot with the connection/discovery problems you were seeing in the ROS2 RoboMaster setup: instead of relying entirely on UDP auto-discovery every run, you know exactly which RoboMaster IP belongs to which Pi/robot pair.
One caution: I would still keep the SDK's normal discovery mechanism available. The reservation makes DHCP deterministic, but it is useful to be able to scan/broadcast-discover the robots as a diagnostic check if one fails to appear.

