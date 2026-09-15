For your RoboMaster/Pi LAN, I would set it up like this:

                     Wi-Fi Router
                  192.168.10.1
                        │
          ┌─────────────┼─────────────┐
          │             │             │
 Registry Server      Pi #1         Pi #2 ...
 192.168.10.10      .101           .102
   :5000
          ▲             ▲             ▲
          │             │             │
          └──── Docker images ────────┘

The registry is just another Docker container. Docker's current official examples use registry:3. Docker Documentation
1. Pick one machine as the registry
It could be your laptop, a dedicated Pi, or some computer permanently connected to the experiment Wi-Fi.
Give it a fixed IP/DHCP reservation, for example:

192.168.10.10

First, while you still have Internet access, get the registry image:

docker pull registry:3

If that machine will already be offline, transfer registry:3 to it with docker save / docker load, just like we discussed.
Then create permanent storage:

mkdir -p ~/docker-registry/data

and run:

docker run -d \
  --name registry \
  --restart unless-stopped \
  -p 5000:5000 \
  -v ~/docker-registry/data:/var/lib/registry \
  registry:3

Now you have a registry at:

192.168.10.10:5000

2. Test it from another machine
From one of the Pis:

curl http://192.168.10.10:5000/v2/

You should get something equivalent to:

{}

If that doesn't work, check that the Wi-Fi router doesn't have client/AP isolation enabled. Your Pis must be allowed to communicate with each other.
Also make sure port 5000 isn't blocked by the registry machine's firewall.
3. Tell Docker on the Pis that this LAN registry is allowed
The easiest initial setup is plain HTTP because this is a private isolated robot LAN.
On every Pi, edit:

sudo nano /etc/docker/daemon.json

and add:

{
  "insecure-registries": [
    "192.168.10.10:5000"
  ]
}

Then:

sudo systemctl restart docker

Docker normally expects private registries to use TLS, so an HTTP registry has to be explicitly declared as insecure. Docker documents insecure-registries for exactly this scenario. Docker Documentation
If /etc/docker/daemon.json already contains settings, merge this property into the existing JSON rather than replacing the file.
Do this on the computer that will docker push as well.
4. Push your RoboMaster image
Suppose locally you have:

robomaster-control:v1

Tag it with the registry address:

docker tag robomaster-control:v1 \
  192.168.10.10:5000/robomaster-control:v1

Docker's private-registry image naming convention is exactly this host:port/image:tag form. Docker Documentation
Then:

docker push 192.168.10.10:5000/robomaster-control:v1

Now the image physically lives on your local registry server.
No Docker Hub involved.
5. Pull it from any Raspberry Pi
On Pi 1:

docker pull 192.168.10.10:5000/robomaster-control:v1

Pi 2:

docker pull 192.168.10.10:5000/robomaster-control:v1

etc.
Then:

docker run \
  192.168.10.10:5000/robomaster-control:v1

This continues working even if the entire Wi-Fi network has zero Internet access.
This becomes especially nice with Ansible
Your inventory could be:

[robots]
robot01 ansible_host=192.168.10.101
robot02 ansible_host=192.168.10.102
robot03 ansible_host=192.168.10.103
robot04 ansible_host=192.168.10.104

And deployment becomes essentially:

- hosts: robots
  become: true

  tasks:
    - name: Pull RoboMaster image
      community.docker.docker_image:
        name: "192.168.10.10:5000/robomaster-control"
        tag: "v1"
        source: pull

    - name: Start RoboMaster container
      community.docker.docker_container:
        name: robomaster
        image: "192.168.10.10:5000/robomaster-control:v1"
        state: started
        restart_policy: unless-stopped

Then you'd effectively do:

docker build ...
docker tag ...
docker push 192.168.10.10:5000/robomaster-control:v2

ansible-playbook deploy.yml

and every robot gets v2.
The major advantage versus copying .tar.gz files with SSH is Docker's layer system. If your image is:

1.8 GB total
├── Ubuntu/ROS base       1.2 GB   unchanged
├── dependencies           500 MB   unchanged
└── your code              100 MB   changed

after you've deployed once, a new version might require transferring only roughly the changed layers rather than the entire 1.8 GB image.
For your fleet, I would therefore use local registry + Ansible + fixed image tags, rather than repeatedly doing docker save over SSH.
One thing I'd change eventually is replacing the "insecure-registries" setup with a private TLS CA. Docker recommends TLS rather than insecure registries for anything security-sensitive. Docker Documentation For an isolated experimental RoboMaster Wi-Fi network, though, HTTP on port 5000 is a very convenient first version.

