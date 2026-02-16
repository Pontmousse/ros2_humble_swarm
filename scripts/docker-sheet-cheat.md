Here’s the **same concise Markdown**, extended with **removal / cleanup commands**, still minimal and README-ready.

---

## 🐳 Docker Basics — Robot Swarm Cheat Sheet

### Show containers

```bash
sudo docker ps
```

Shows **running** containers only.

```bash
sudo docker ps -a
```

Shows **all** containers (running + stopped).

---

### Show images

```bash
sudo docker images
```

Lists Docker images on the system.

---

### Container lifecycle

```bash
sudo docker start swarm
```

Start the container.

```bash
sudo docker stop swarm
```

Graceful stop (SIGTERM, ROS shuts down cleanly).

```bash
sudo docker kill swarm
```

Force stop (SIGKILL, emergency only).

```bash
sudo docker restart swarm
```

Stop + start in one command.

---

### Exec into container

```bash
sudo docker exec -it swarm bash
```

Open an interactive shell inside the running container.

---

### Logs

```bash
sudo docker logs swarm
```

Show container logs.

```bash
sudo docker logs -f swarm
```

Follow logs live (useful for ROS launch output).

---

## 🧹 Remove / Cleanup

### Remove a container

```bash
sudo docker stop swarm
sudo docker rm swarm
```

Stops and removes the container.

---

### Remove a Docker image

```bash
sudo docker rmi elghaliasri/ros2-humble-swarm:robot-arm64-v1
```

Removes the image (must not be used by any container).

---

### Remove all stopped containers

```bash
sudo docker container prune
```

Deletes **stopped** containers only.

---

### Remove unused images

```bash
sudo docker image prune
```

Deletes dangling (unused) images.

---

### Full cleanup (use with care)

```bash
sudo docker system prune
```

Removes:

* stopped containers
* unused images
* unused networks
* build cache

⚠️ **Do not run casually**.

---

That’s the **complete, minimal operational set** for Docker on your swarm robots.
