#!/usr/bin/env python3
"""SWO-to-TCP bridge for PlatformIO's serial monitor.

PlatformIO's built-in monitor can read serial ports and pySerial URLs such as
``socket://127.0.0.1:34430``, but it cannot directly consume ARM CoreSight SWO.
This helper starts OpenOCD, captures the raw SWO stream into ``tools/swo.log``,
decodes ITM stimulus port 0, and serves the clean text over a local TCP socket.
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path


BACKLOG_LIMIT = 64 * 1024


class SwoBridge:
    def __init__(
        self,
        host: str,
        port: int,
        openocd: Path,
        cfg: Path,
        raw_log: Path,
        gateway_log: Path,
        pid_file: Path,
    ) -> None:
        self.host = host
        self.port = port
        self.openocd = openocd
        self.cfg = cfg
        self.raw_log = raw_log
        self.gateway_log = gateway_log
        self.pid_file = pid_file
        self.stop_event = threading.Event()
        self.clients: list[socket.socket] = []
        self.clients_lock = threading.Lock()
        self.backlog: collections.deque[int] = collections.deque(maxlen=BACKLOG_LIMIT)
        self.openocd_proc: subprocess.Popen[str] | None = None

    def run(self) -> int:
        self.raw_log.parent.mkdir(parents=True, exist_ok=True)
        self.gateway_log.parent.mkdir(parents=True, exist_ok=True)
        self.raw_log.write_bytes(b"")

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(4)
        server.settimeout(0.25)

        self.openocd_proc = self._start_openocd()
        self._write_pid_file()

        threading.Thread(target=self._accept_clients, args=(server,), daemon=True).start()
        threading.Thread(target=self._tail_and_decode, daemon=True).start()

        try:
            while not self.stop_event.is_set():
                if self.openocd_proc.poll() is not None:
                    self._broadcast(
                        f"\r\n[SWO BRIDGE] OpenOCD exited with code "
                        f"{self.openocd_proc.returncode}\r\n".encode("ascii")
                    )
                    return int(self.openocd_proc.returncode or 0)
                time.sleep(0.2)
        finally:
            self.stop_event.set()
            server.close()
            self._close_clients()
            self._stop_openocd()
            try:
                self.pid_file.unlink()
            except FileNotFoundError:
                pass
        return 0

    def _start_openocd(self) -> subprocess.Popen[str]:
        log = open(self.gateway_log, "a", encoding="utf-8", errors="replace")
        log.write("\n=== starting OpenOCD for PlatformIO SWO bridge ===\n")
        log.flush()
        return subprocess.Popen(
            [str(self.openocd), "-f", str(self.cfg)],
            cwd=str(self.cfg.parent.parent),
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
        )

    def _write_pid_file(self) -> None:
        data = {
            "gateway_pid": os.getpid(),
            "openocd_pid": self.openocd_proc.pid if self.openocd_proc else None,
            "host": self.host,
            "port": self.port,
            "raw_log": str(self.raw_log),
        }
        self.pid_file.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def _accept_clients(self, server: socket.socket) -> None:
        while not self.stop_event.is_set():
            try:
                client, _addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                return

            client.settimeout(0.25)
            with self.clients_lock:
                self.clients.append(client)
                if self.backlog:
                    try:
                        client.sendall(bytes(self.backlog))
                    except OSError:
                        self.clients.remove(client)
                        client.close()
                        continue
            threading.Thread(target=self._drain_client_input, args=(client,), daemon=True).start()

    def _drain_client_input(self, client: socket.socket) -> None:
        while not self.stop_event.is_set():
            try:
                data = client.recv(256)
                if not data:
                    break
            except socket.timeout:
                continue
            except OSError:
                break
        with self.clients_lock:
            if client in self.clients:
                self.clients.remove(client)
        try:
            client.close()
        except OSError:
            pass

    def _tail_and_decode(self) -> None:
        while not self.stop_event.is_set() and not self.raw_log.exists():
            time.sleep(0.05)

        position = 0
        state = "HEADER"
        payload_port = 0
        payload_remaining = 0

        while not self.stop_event.is_set():
            try:
                with self.raw_log.open("rb") as f:
                    f.seek(position)
                    chunk = f.read()
                    position = f.tell()
            except OSError:
                time.sleep(0.05)
                continue

            if not chunk:
                time.sleep(0.05)
                continue

            for byte in chunk:
                if state == "HEADER":
                    size_bits = byte & 0x03
                    continuation = byte & 0x04
                    if size_bits == 0 or continuation != 0:
                        continue
                    payload_port = (byte >> 3) & 0x1F
                    payload_remaining = 1 if size_bits == 1 else (2 if size_bits == 2 else 4)
                    state = "PAYLOAD"
                else:
                    if payload_port == 0:
                        self._broadcast(bytes((byte,)))
                    payload_remaining -= 1
                    if payload_remaining <= 0:
                        state = "HEADER"

    def _broadcast(self, data: bytes) -> None:
        self.backlog.extend(data)
        with self.clients_lock:
            dead: list[socket.socket] = []
            for client in self.clients:
                try:
                    client.sendall(data)
                except OSError:
                    dead.append(client)
            for client in dead:
                self.clients.remove(client)
                try:
                    client.close()
                except OSError:
                    pass

    def _close_clients(self) -> None:
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except OSError:
                    pass
            self.clients.clear()

    def _stop_openocd(self) -> None:
        if self.openocd_proc and self.openocd_proc.poll() is None:
            self.openocd_proc.terminate()
            try:
                self.openocd_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.openocd_proc.kill()


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=34430)
    parser.add_argument("--openocd", required=True)
    parser.add_argument("--cfg", required=True)
    parser.add_argument("--raw-log", required=True)
    parser.add_argument("--gateway-log", required=True)
    parser.add_argument("--pid-file", required=True)
    args = parser.parse_args(argv)

    bridge = SwoBridge(
        host=args.host,
        port=args.port,
        openocd=Path(args.openocd),
        cfg=Path(args.cfg),
        raw_log=Path(args.raw_log),
        gateway_log=Path(args.gateway_log),
        pid_file=Path(args.pid_file),
    )

    def _signal_handler(_signum: int, _frame: object) -> None:
        bridge.stop_event.set()

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)
    return bridge.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
