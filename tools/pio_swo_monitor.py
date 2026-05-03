"""PlatformIO hook: make Upload and Monitor work with SWO.

When the user runs:

    pio run -e bluepill_flight -t upload -t monitor

PlatformIO's built-in monitor opens ``monitor_port``.  We set that port to a
local TCP socket and start ``tools/swo_gateway.py`` automatically after upload
so the built-in monitor receives decoded SWO text.
"""

from __future__ import annotations

import json
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

Import("env")  # type: ignore[name-defined]


HOST = "127.0.0.1"
PORT = 34430


def _project_dir() -> Path:
    return Path(env.subst("$PROJECT_DIR"))  # type: ignore[name-defined]


def _python() -> str:
    return env.subst("$PYTHONEXE") or sys.executable  # type: ignore[name-defined]


def _openocd() -> Path:
    project = _project_dir()
    candidate = project.parent / ".platformio" / "packages" / "tool-openocd" / "bin" / "openocd.exe"
    if candidate.exists():
        return candidate
    userprofile = Path(os.environ.get("USERPROFILE", ""))
    return userprofile / ".platformio" / "packages" / "tool-openocd" / "bin" / "openocd.exe"


def _port_is_open() -> bool:
    try:
        with socket.create_connection((HOST, PORT), timeout=0.2):
            return True
    except OSError:
        return False


def _taskkill(pid: int) -> None:
    if pid <= 0:
        return
    if sys.platform.startswith("win"):
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        try:
            os.kill(pid, 15)
        except OSError:
            pass


def _kill_port_listener() -> None:
    """Recover from an orphaned SWO bridge when the pid file is missing."""
    if not sys.platform.startswith("win"):
        return
    try:
        result = subprocess.run(
            ["netstat", "-ano", "-p", "tcp"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
    except OSError:
        return

    for line in result.stdout.splitlines():
        if f":{PORT}" not in line or "LISTENING" not in line:
            continue
        match = re.search(r"\s(\d+)\s*$", line)
        if match:
            _taskkill(int(match.group(1)))
    time.sleep(0.5)


def _kill_previous_gateway(pid_file: Path) -> None:
    if not pid_file.exists():
        return
    try:
        data = json.loads(pid_file.read_text(encoding="utf-8"))
        gateway_pid = int(data.get("gateway_pid", 0))
        openocd_pid = int(data.get("openocd_pid", 0))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        gateway_pid = 0
        openocd_pid = 0

    if gateway_pid <= 0 and openocd_pid <= 0:
        try:
            pid_file.unlink()
        except OSError:
            pass
        return

    _taskkill(gateway_pid)
    _taskkill(openocd_pid)
    time.sleep(0.5)
    try:
        pid_file.unlink()
    except OSError:
        pass


def _start_gateway(source=None, target=None, env_arg=None, **_kwargs) -> None:
    # PlatformIO handles "-t monitor" outside the SCons target list.  During
    # `pio run -t upload -t monitor`, SCons only sees the "upload" target, then
    # PlatformIO opens `monitor_port` after SCons exits.  Therefore the bridge
    # must start after every SWO upload; otherwise the socket monitor races a
    # port that does not exist yet.
    project = _project_dir()
    tools = project / "tools"
    gateway = tools / "swo_gateway.py"
    cfg = tools / "swo_capture.cfg"
    raw_log = tools / "swo.log"
    gateway_log = tools / "swo_gateway.log"
    pid_file = tools / "swo_gateway.pid"

    _kill_previous_gateway(pid_file)
    if _port_is_open():
        _kill_port_listener()

    if _port_is_open():
        print(f"[SWO] WARNING: socket://{HOST}:{PORT} is already in use.")
        return

    args = [
        _python(),
        str(gateway),
        "--host",
        HOST,
        "--port",
        str(PORT),
        "--openocd",
        str(_openocd()),
        "--cfg",
        str(cfg),
        "--raw-log",
        str(raw_log),
        "--gateway-log",
        str(gateway_log),
        "--pid-file",
        str(pid_file),
    ]

    flags = 0
    if sys.platform.startswith("win"):
        flags = subprocess.CREATE_NO_WINDOW | subprocess.DETACHED_PROCESS  # type: ignore[attr-defined]

    subprocess.Popen(args, cwd=str(project), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags)

    deadline = time.time() + 8.0
    while time.time() < deadline:
        if _port_is_open():
            print(f"[SWO] PlatformIO monitor bridge listening on socket://{HOST}:{PORT}")
            return
        time.sleep(0.1)

    print("[SWO] WARNING: monitor bridge did not start in time.")
    print(f"[SWO] Check log: {gateway_log}")


def _stop_gateway_before_upload(source=None, target=None, env_arg=None, **_kwargs) -> None:
    # A previous PlatformIO monitor session leaves the SWO bridge/OpenOCD alive
    # until the user closes it.  Kill our own bridge before flashing so ST-Link
    # is free for PlatformIO's upload OpenOCD instance.
    pid_file = _project_dir() / "tools" / "swo_gateway.pid"
    _kill_previous_gateway(pid_file)
    if _port_is_open():
        _kill_port_listener()


env.AddPreAction("upload", _stop_gateway_before_upload)  # type: ignore[name-defined]
env.AddPostAction("upload", _start_gateway)  # type: ignore[name-defined]
env.AddPreAction("monitor", _start_gateway)  # type: ignore[name-defined]
