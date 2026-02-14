Here’s a **clean, concise Markdown version** you can drop straight into a `README.md`.

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

That’s the **minimal daily-use set** — nothing extra, nothing missing.
