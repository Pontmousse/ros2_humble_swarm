"""Self-describing-bag metadata: snapshot the configs a run was launched with.

Nothing in the stack used to record *which configs produced a bag*. The YAMLs that
parameterize a run (virtual_spacecraft.yaml, the scene file, swarm_config.yaml) and the
env flags that reshape it (VIRTUAL_*, DIRECT_CMD_VEL, SCENE_FILE, ...)
lived only on disk / in the shell at launch time, so a bag reviewed later could not be
tied back to the configuration that generated it.

The swarm node publishes one latched std_msgs/String whose payload is a JSON document
bundling the raw text of each config file (so it round-trips exactly) plus the run's env
flags and, best-effort, the git HEAD of the install tree. The swarm launches
(centralized + decentralized) start it and tell it which files/env to capture, because
they are the processes that actually own the run's environment; the recorder then just
records /run_config (global) and /<ns>/run_config (per robot). Read it back offline with
`json.loads(msg.data)` -- no custom deserializer needed.

snapshot() is deliberately generic: file-selection logic stays in the launches (which know
the share dirs and read the env), this just reads whatever paths/names it is handed.
"""

import hashlib
import json
import os
import socket
import subprocess
from datetime import datetime, timezone

from std_msgs.msg import String

SCHEMA_VERSION = "run_config/v1"


def _read_file(path):
    """Return {sha256, text, bytes} for a file, or {error} if unreadable."""
    try:
        with open(path, "rb") as handle:
            raw = handle.read()
    except OSError as exc:
        return {"error": str(exc)}
    return {
        "sha256": hashlib.sha256(raw).hexdigest(),
        "bytes": len(raw),
        # errors="replace" so a stray non-UTF8 byte degrades one char rather than
        # dropping the whole (otherwise-valid) config from the snapshot.
        "text": raw.decode("utf-8", errors="replace"),
    }


def _git_info(repo_path):
    """Best-effort {commit, branch, dirty} for repo_path, or None if unavailable.

    On the Pis the repo is often absent (docker-cp deploy, not a bind-mounted work
    tree), so this resolving to None is expected -- the file contents still come from
    the Pi's own share dir, which is the part that matters.
    """
    if not repo_path or not os.path.isdir(repo_path):
        return None

    def _git(*args):
        return subprocess.run(
            ["git", "-C", repo_path, *args],
            capture_output=True,
            text=True,
            timeout=5,
        )

    try:
        head = _git("rev-parse", "HEAD")
        if head.returncode != 0:
            return None
        branch = _git("rev-parse", "--abbrev-ref", "HEAD")
        status = _git("status", "--porcelain")
        return {
            "commit": head.stdout.strip(),
            "branch": branch.stdout.strip() if branch.returncode == 0 else None,
            "dirty": bool(status.stdout.strip()) if status.returncode == 0 else None,
        }
    except (OSError, subprocess.SubprocessError):
        return None


def snapshot(config_files, env_vars, repo_path, source_launch, robot_names):
    """Return (String with the JSON document, [unreadable files])."""
    document = {
        "schema": SCHEMA_VERSION,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "hostname": socket.gethostname(),
        "source_launch": source_launch,
        "robot_names": robot_names,
        "git": _git_info(repo_path),
        "env": {name: os.environ.get(name) for name in env_vars},
        "files": {path: _read_file(path) for path in config_files},
    }
    missing = [p for p, m in document["files"].items() if "error" in m]
    return String(data=json.dumps(document, indent=2, sort_keys=True)), missing
