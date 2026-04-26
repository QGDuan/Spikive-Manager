#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
astro_manager / node_manager.py
--------------------------------

Per-drone supervisor for the Spikive ROS1 stack.

Responsibilities
================
- Start, stop, restart six logical "launches": MavRos, Lidar_Driver,
  Camera_Diver, LIO, Planner, Ctrl.
- Publish a 1 Hz AutoManager status that the Lichtblick frontend uses to
  drive the Robot card's Start / Restart button.
- Refuse start/restart/shutdown while the drone is armed (mavros/state.armed).
- Keep its own log stream isolated from rosout so SLAM/Planner spam does
  not drown out manager events.
- Manage child processes via Popen + setsid, allowing reliable group kill.

Topics (all prefixed with /drone_{drone_id}_)
=============================================
- /drone_{id}_command_topic         (sub)  astro_manager/Command
- /drone_{id}_auto_manager_status   (pub)  astro_manager/AutoManager
- /drone_{id}_mavros/state          (sub)  mavros_msgs/State

Logs (per-drone)
================
~/spikive_logs/drone_{id}/
  manager.log               -- this node's events (one file, no rosout)
  drivers/{LaunchName}.log  -- stdout+stderr of each driver child
  algo/{LaunchName}.log     -- stdout+stderr of each algorithm child
"""

from __future__ import annotations

import logging
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Set

import rospkg
import rospy
import yaml

from astro_manager.msg import AutoManager, Command

try:
    from mavros_msgs.msg import State as MavrosState
    _MAVROS_AVAILABLE = True
except ImportError:  # mavros_msgs may not be installed at build time
    MavrosState = None
    _MAVROS_AVAILABLE = False


# ----- constants -------------------------------------------------------------

STATUS_PUBLISH_HZ = 1.0
ROSNODE_LIST_INTERVAL = 1.0          # seconds between rosnode list polls during readiness wait
HEALTH_POLL_INTERVAL = 2.0           # background health update period
GRACE_SHUTDOWN_SEC = 5.0             # SIGINT -> SIGKILL grace period for child groups
INTER_LAUNCH_DELAY_SEC = 0.5         # small breathing room between sequential launches

# Status integers for AutoManager subsystem fields.
ST_NOT_RUNNING = 0
ST_PARTIAL = 1
ST_READY = 2

# Mode strings broadcast in AutoManager.mode.
MODE_IDLE = "idle"
MODE_STARTING = "starting"
MODE_READY = "ready"
MODE_STOPPING = "stopping"
MODE_ERROR = "error"


# ----- color terminal formatter (manager's own console only) -----------------


class _ColorFormatter(logging.Formatter):
    _COLORS = {
        "DEBUG": "\033[36m",
        "INFO": "\033[32m",
        "WARNING": "\033[33m",
        "ERROR": "\033[31m",
        "CRITICAL": "\033[1;41m",
    }
    _RESET = "\033[0m"

    def format(self, record: logging.LogRecord) -> str:
        color = self._COLORS.get(record.levelname, "")
        msg = super().format(record)
        return f"{color}{msg}{self._RESET}"


def _setup_logger(drone_id: str, log_root: str) -> logging.Logger:
    """Manager logger that does NOT propagate to rosout, so SLAM/Planner spam
    via /rosout cannot drown out manager events. Writes a dedicated file plus
    color-coded stderr."""
    log_dir = os.path.join(os.path.expanduser(log_root), f"drone_{drone_id}")
    os.makedirs(log_dir, exist_ok=True)

    logger = logging.getLogger(f"astro_manager.drone_{drone_id}")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False  # critical: stay out of rosout

    # File handler -- everything.
    fh = logging.FileHandler(os.path.join(log_dir, "manager.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(
        logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    )
    logger.addHandler(fh)

    # Console handler -- INFO+, colored.
    ch = logging.StreamHandler(sys.stderr)
    ch.setLevel(logging.INFO)
    ch.setFormatter(_ColorFormatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.addHandler(ch)

    return logger


# ----- ManagedProcess --------------------------------------------------------


class ManagedProcess:
    """Wraps a single subprocess.Popen that runs a roslaunch (or any shell)
    command. Each instance owns its log fd so output never mixes with
    other launches or with the manager's own log."""

    def __init__(self, name: str, cmd: str, log_path: str, logger: logging.Logger):
        self.name = name
        self.cmd = cmd
        self.log_path = log_path
        self.log = logger
        self._popen: Optional[subprocess.Popen] = None
        self._log_fd = None
        self.started_at: Optional[float] = None

    def is_alive(self) -> bool:
        return self._popen is not None and self._popen.poll() is None

    def start(self) -> None:
        if self.is_alive():
            self.log.warning("ManagedProcess[%s] already running, skip start", self.name)
            return
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self._log_fd = open(self.log_path, "ab", buffering=0)
        self._log_fd.write(
            f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} starting: {self.cmd} =====\n".encode()
        )
        self._popen = subprocess.Popen(
            self.cmd,
            shell=True,
            executable="/bin/bash",
            stdout=self._log_fd,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,  # own process group -> killpg works cleanly
            close_fds=True,
        )
        self.started_at = time.time()
        self.log.info("Started [%s] pid=%d (log=%s)", self.name, self._popen.pid, self.log_path)

    def stop(self, grace_sec: float = GRACE_SHUTDOWN_SEC) -> None:
        if self._popen is None:
            return
        if self._popen.poll() is not None:
            self._cleanup_fd()
            self._popen = None
            return
        try:
            pgid = os.getpgid(self._popen.pid)
        except ProcessLookupError:
            self._cleanup_fd()
            self._popen = None
            return
        self.log.info("Stopping [%s] pgid=%d (SIGINT, grace %.1fs)", self.name, pgid, grace_sec)
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        deadline = time.time() + grace_sec
        while time.time() < deadline:
            if self._popen.poll() is not None:
                break
            time.sleep(0.2)
        if self._popen.poll() is None:
            self.log.warning("Force-killing [%s] pgid=%d (SIGKILL)", self.name, pgid)
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self._popen.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.log.error("[%s] did not exit after SIGKILL", self.name)
        self._cleanup_fd()
        self._popen = None
        self.started_at = None

    def _cleanup_fd(self) -> None:
        if self._log_fd is not None:
            try:
                self._log_fd.write(
                    f"===== {time.strftime('%Y-%m-%d %H:%M:%S')} stopped =====\n\n".encode()
                )
                self._log_fd.close()
            except Exception:
                pass
            self._log_fd = None


# ----- NodeManager -----------------------------------------------------------


class NodeManager:
    def __init__(self) -> None:
        rospy.init_node("node_manager", anonymous=False)

        self.drone_id: str = str(rospy.get_param("~drone_id", "")).strip()
        if not self.drone_id:
            rospy.logfatal("astro_manager: ~drone_id is required")
            raise SystemExit(2)

        log_root = rospy.get_param("~log_root", "~/spikive_logs")
        self.log_root = os.path.expanduser(str(log_root))
        self.log = _setup_logger(self.drone_id, self.log_root)

        # Topic names follow the existing /drone_{id}_<base> convention.
        self.cmd_topic = f"/drone_{self.drone_id}_command_topic"
        self.status_topic = f"/drone_{self.drone_id}_auto_manager_status"
        self.armed_topic = f"/drone_{self.drone_id}_mavros/state"

        self.log.info(
            "astro_manager booting drone_id=%s cmd=%s status=%s armed=%s",
            self.drone_id, self.cmd_topic, self.status_topic, self.armed_topic,
        )

        # ---- config ---------------------------------------------------------
        config_path = rospy.get_param(
            "~config_path",
            os.path.join(rospkg.RosPack().get_path("astro_manager"), "config", "launch_config.yaml"),
        )
        self.launches: Dict[str, dict] = self._load_config(config_path)
        # Topological order of launch names (raises if a cycle is detected).
        self.launch_order: List[str] = self._topo_sort(self.launches)
        self.log.info("Launch order: %s", " -> ".join(self.launch_order))

        # ---- process registry ----------------------------------------------
        self.processes: Dict[str, ManagedProcess] = {}
        for name, cfg in self.launches.items():
            log_path = os.path.join(
                self.log_root, f"drone_{self.drone_id}",
                cfg.get("group", "misc"), f"{name}.log",
            )
            cmd = cfg["launch_cmd"].replace("${DRONE_ID}", self.drone_id)
            cfg["nodes"] = [n.replace("${DRONE_ID}", self.drone_id) for n in cfg.get("nodes", [])]
            self.processes[name] = ManagedProcess(
                name=name, cmd=cmd, log_path=log_path, logger=self.log,
            )

        # ---- runtime state --------------------------------------------------
        self._lock = threading.RLock()
        self.mode: str = MODE_IDLE
        self.is_active: bool = False
        self.starting: bool = False
        self.stopping: bool = False
        self.armed: bool = False
        self.last_error: str = ""
        self.last_error_seq: int = 0

        # Cache of the most recent rosnode list output (refreshed by health thread).
        self._running_nodes: Set[str] = set()
        self._running_nodes_lock = threading.Lock()

        # Single-consumer command queue: ROS callback enqueues, worker thread executes.
        # This prevents Popen + sleep loops from blocking the rospy callback queue.
        self.cmd_queue: "queue.Queue[Command]" = queue.Queue(maxsize=16)
        self._last_command: Command = Command()

        # ---- ROS I/O --------------------------------------------------------
        self._cmd_sub = rospy.Subscriber(self.cmd_topic, Command, self._on_command, queue_size=4)
        self._status_pub = rospy.Publisher(self.status_topic, AutoManager, queue_size=10)

        if _MAVROS_AVAILABLE:
            self._armed_sub = rospy.Subscriber(
                self.armed_topic, MavrosState, self._on_armed, queue_size=4,
            )
        else:
            self.log.warning(
                "mavros_msgs not importable; armed-interlock disabled (assuming armed=False)"
            )
            self._armed_sub = None

        # ---- threads --------------------------------------------------------
        self._stop_evt = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, name="cmd_worker", daemon=True)
        self._worker.start()
        self._health = threading.Thread(target=self._health_loop, name="health_loop", daemon=True)
        self._health.start()
        self._status_timer = rospy.Timer(
            rospy.Duration(1.0 / STATUS_PUBLISH_HZ), self._publish_status,
        )
        rospy.on_shutdown(self._shutdown_hook)

        # ---- recover state from currently running nodes ---------------------
        self._refresh_running_nodes()
        self._recover_state()

        self.log.info(
            "astro_manager ready (mode=%s is_active=%s)", self.mode, self.is_active,
        )

    # ------------------------------------------------------------------ config

    def _load_config(self, path: str) -> Dict[str, dict]:
        if not os.path.exists(path):
            self.log.error("Config file not found: %s", path)
            raise SystemExit(3)
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        launches = data.get("launches", {}) or {}
        if not launches:
            self.log.error("Config has no 'launches' section: %s", path)
            raise SystemExit(3)
        for name, cfg in launches.items():
            if "launch_cmd" not in cfg:
                self.log.error("Launch '%s' missing 'launch_cmd'", name)
                raise SystemExit(3)
            cfg.setdefault("nodes", [])
            cfg.setdefault("group", "misc")
            cfg.setdefault("depends_on", [])
            cfg.setdefault("start_timeout", 8.0)
        self.log.info("Loaded %d launches from %s", len(launches), path)
        return launches

    @staticmethod
    def _topo_sort(launches: Dict[str, dict]) -> List[str]:
        """Kahn's algorithm. Stable order for deterministic startup."""
        in_deg: Dict[str, int] = {n: 0 for n in launches}
        edges: Dict[str, List[str]] = {n: [] for n in launches}
        for name, cfg in launches.items():
            for dep in cfg.get("depends_on", []):
                if dep not in launches:
                    raise ValueError(f"Launch '{name}' depends on unknown '{dep}'")
                edges[dep].append(name)
                in_deg[name] += 1
        ordered: List[str] = []
        ready = deque(sorted(n for n, d in in_deg.items() if d == 0))
        while ready:
            n = ready.popleft()
            ordered.append(n)
            for m in sorted(edges[n]):
                in_deg[m] -= 1
                if in_deg[m] == 0:
                    ready.append(m)
        if len(ordered) != len(launches):
            raise ValueError("Dependency cycle detected in launch_config.yaml")
        return ordered

    # ------------------------------------------------------------------ ROS callbacks

    def _on_command(self, msg: Command) -> None:
        # Stay non-blocking: the worker thread does the real work.
        try:
            self.cmd_queue.put_nowait(msg)
            self.log.info(
                "Queued command: type=%s targets=%s qsize=%d",
                msg.command_type, list(msg.target_launches), self.cmd_queue.qsize(),
            )
        except queue.Full:
            self._set_error("Command queue full, dropping command", msg.command_type)

    def _on_armed(self, msg) -> None:  # MavrosState (or None at type-check time)
        new_armed = bool(getattr(msg, "armed", False))
        if new_armed != self.armed:
            self.log.warning("armed state changed: %s -> %s", self.armed, new_armed)
        self.armed = new_armed

    # ------------------------------------------------------------------ worker

    def _worker_loop(self) -> None:
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            try:
                msg = self.cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._handle_command(msg)
            except Exception as e:  # pylint: disable=broad-except
                self.log.exception("Command handler crashed: %s", e)
                self._set_error(f"Internal error: {e!r}", msg.command_type)
            finally:
                self._last_command = msg

    def _handle_command(self, msg: Command) -> None:
        ctype = (msg.command_type or "").strip()

        # Hard interlock: any lifecycle change while armed is rejected.
        if ctype in ("start_all", "shutdown_all",
                     "start_node", "restart_node", "shutdown_node"):
            if self.armed:
                self._set_error(f"REJECTED: drone armed, aborting {ctype}", ctype)
                return

        if ctype == "start_all":
            # Reject double-start: only allowed from IDLE / ERROR.
            if self.starting or self.stopping:
                self._set_error(
                    f"REJECTED: cannot start_all while mode={self.mode}", ctype,
                )
                return
            if self.is_active:
                self._set_error(
                    "REJECTED: already running; send shutdown_all first", ctype,
                )
                return
            self._start_all()
        elif ctype == "shutdown_all":
            if self.starting or self.stopping:
                self._set_error(
                    f"REJECTED: cannot shutdown_all while mode={self.mode}", ctype,
                )
                return
            if not self.is_active:
                self._set_error(
                    "REJECTED: nothing to stop; not active", ctype,
                )
                return
            self._shutdown_all()
        elif ctype == "start_node":
            self._start_named(list(msg.target_launches), restart=False)
        elif ctype == "restart_node":
            self._start_named(list(msg.target_launches), restart=True)
        elif ctype == "shutdown_node":
            self._shutdown_named(list(msg.target_launches))
        else:
            self._set_error(f"Unknown command_type: {ctype!r}", ctype)

    # ------------------------------------------------------------------ lifecycle

    def _start_all(self) -> None:
        """Start every launch in topological order with readiness checks."""
        with self._lock:
            self.starting = True
            self.mode = MODE_STARTING
        self._publish_status_immediate()
        self.log.info("Begin start_all")

        ok = True
        for name in self.launch_order:
            if not self._start_launch(name):
                ok = False
                break
            time.sleep(INTER_LAUNCH_DELAY_SEC)

        with self._lock:
            self.starting = False
            if ok:
                self.is_active = True
                self.mode = MODE_READY
                self.log.info("All launches ready (is_active=true)")
            else:
                self.mode = MODE_ERROR
                self.log.error("Start sequence aborted: %s", self.last_error or "unknown")
        self._publish_status_immediate()

    def _stop_all_processes(self) -> None:
        """Stop every launch in reverse topological order. Used by both
        restart_all (as phase 1) and shutdown_all."""
        for name in reversed(self.launch_order):
            self.processes[name].stop()

    def _shutdown_all(self) -> None:
        with self._lock:
            self.stopping = True
            self.mode = MODE_STOPPING
        self._publish_status_immediate()
        self.log.info("Begin shutdown_all")
        self._stop_all_processes()
        with self._lock:
            self.stopping = False
            self.is_active = False
            self.mode = MODE_IDLE
        self._publish_status_immediate()
        self.log.info("shutdown_all complete")

    def _start_named(self, names: List[str], restart: bool) -> None:
        for n in names:
            if n not in self.processes:
                self._set_error(f"Unknown launch '{n}'", "start_node")
                continue
            if restart:
                self.processes[n].stop()
            self._start_launch(n)

    def _shutdown_named(self, names: List[str]) -> None:
        for n in names:
            if n not in self.processes:
                self._set_error(f"Unknown launch '{n}'", "shutdown_node")
                continue
            self.processes[n].stop()
        with self._lock:
            if not any(self.processes[n].is_alive() for n in self.processes):
                self.is_active = False
                self.mode = MODE_IDLE

    def _start_launch(self, name: str) -> bool:
        cfg = self.launches[name]
        timeout = float(cfg.get("start_timeout", 8.0))
        nodes: List[str] = list(cfg.get("nodes", []))

        try:
            self.processes[name].start()
        except Exception as e:  # pylint: disable=broad-except
            self._set_error(f"Failed to spawn '{name}': {e!r}", "start")
            return False

        if not nodes:
            # No node-readiness criteria; trust the Popen and move on.
            self.log.warning("No 'nodes' configured for '%s', skipping readiness check", name)
            return True

        return self._wait_until_ready(name, nodes, timeout)

    def _wait_until_ready(self, launch_name: str, nodes: List[str], timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.processes[launch_name].is_alive():
                self._set_error(f"'{launch_name}' exited during startup", "start")
                return False
            self._refresh_running_nodes()
            with self._running_nodes_lock:
                if all(n in self._running_nodes for n in nodes):
                    self.log.info("'%s' READY (nodes=%s)", launch_name, nodes)
                    return True
            time.sleep(ROSNODE_LIST_INTERVAL)
        missing = [n for n in nodes if n not in self._running_nodes]
        self._set_error(
            f"'{launch_name}' not READY after {timeout:.1f}s (missing={missing})", "start",
        )
        # Tear it down so we don't leak a half-started launch.
        self.processes[launch_name].stop()
        return False

    # ------------------------------------------------------------------ health

    def _health_loop(self) -> None:
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            self._refresh_running_nodes()
            time.sleep(HEALTH_POLL_INTERVAL)

    def _refresh_running_nodes(self) -> None:
        try:
            out = subprocess.check_output(
                ["rosnode", "list"], stderr=subprocess.DEVNULL, timeout=3.0,
            ).decode("utf-8", errors="replace")
            running = set(line.strip() for line in out.splitlines() if line.strip())
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
            running = set()
        with self._running_nodes_lock:
            self._running_nodes = running

    def _recover_state(self) -> None:
        """If manager itself was restarted while children are still running,
        figure out current state from rosnode list."""
        all_ready = True
        any_running = False
        for name in self.launch_order:
            nodes = self.launches[name].get("nodes", [])
            if not nodes:
                continue
            present = sum(1 for n in nodes if n in self._running_nodes)
            if present == 0:
                all_ready = False
            elif present < len(nodes):
                all_ready = False
                any_running = True
            else:
                any_running = True
        with self._lock:
            if all_ready and any_running:
                self.is_active = True
                self.mode = MODE_READY
            elif any_running:
                self.is_active = False
                self.mode = MODE_ERROR
                self.last_error = "Recovered with partial nodes running (manager did not own them)"
                self.last_error_seq += 1
            else:
                self.is_active = False
                self.mode = MODE_IDLE

    # ------------------------------------------------------------------ status

    def _check_subsystem(self, launch_name: str) -> int:
        nodes = self.launches[launch_name].get("nodes", [])
        if not nodes:
            return ST_NOT_RUNNING
        with self._running_nodes_lock:
            present = sum(1 for n in nodes if n in self._running_nodes)
        if present == 0:
            return ST_NOT_RUNNING
        if present < len(nodes):
            return ST_PARTIAL
        return ST_READY

    def _build_status_msg(self) -> AutoManager:
        m = AutoManager()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = f"drone_{self.drone_id}"
        with self._lock:
            m.mode = self.mode
            m.is_active = self.is_active
            m.starting = self.starting
            m.armed = self.armed
            m.last_error = self.last_error
            m.last_error_seq = self.last_error_seq
            m.restart_status = 1 if self.starting else 0
        m.odometry_status = 0  # not tracked yet
        m.mavros_status = self._check_subsystem("MavRos") if "MavRos" in self.launches else 0
        m.cam_driver_status = (
            self._check_subsystem("Camera_Driver") if "Camera_Driver" in self.launches else 0
        )
        m.lidar_driver_status = (
            self._check_subsystem("Lidar_Driver") if "Lidar_Driver" in self.launches else 0
        )
        m.slam_status = self._check_subsystem("LIO") if "LIO" in self.launches else 0
        m.planner_status = self._check_subsystem("Planner") if "Planner" in self.launches else 0
        m.ctrl_status = self._check_subsystem("Ctrl") if "Ctrl" in self.launches else 0
        m.command = self._last_command
        return m

    def _publish_status(self, _evt) -> None:
        try:
            self._status_pub.publish(self._build_status_msg())
        except Exception as e:  # pylint: disable=broad-except
            self.log.error("status publish failed: %s", e)

    def _publish_status_immediate(self) -> None:
        try:
            self._status_pub.publish(self._build_status_msg())
        except Exception as e:  # pylint: disable=broad-except
            self.log.error("status publish (immediate) failed: %s", e)

    # ------------------------------------------------------------------ utility

    def _set_error(self, message: str, context: str = "") -> None:
        with self._lock:
            self.last_error_seq += 1
            self.last_error = message
            self.mode = MODE_ERROR if self.starting is False else self.mode
        self.log.error("[%s] %s (seq=%d)", context, message, self.last_error_seq)
        self._publish_status_immediate()

    def _shutdown_hook(self) -> None:
        self.log.info("ROS shutdown received; tearing down children")
        self._stop_evt.set()
        for name in reversed(self.launch_order):
            try:
                self.processes[name].stop(grace_sec=2.0)
            except Exception:
                pass


# ----- entrypoint ------------------------------------------------------------


def main() -> int:
    try:
        NodeManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except SystemExit:
        raise
    except Exception as e:  # pylint: disable=broad-except
        rospy.logfatal(f"astro_manager crashed: {e!r}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
