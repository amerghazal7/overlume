#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
"""Minimal HTTP CONNECT proxy (stdlib only) used as a network kill switch.

Point the node at it with `https_proxy=http://127.0.0.1:8899`; while the
proxy runs, every Cesium ion / tile request tunnels through it unchanged.
Killing this process is the "cable pull": libcurl gets connection refused
on every request, which is exactly the consecutive-failure signal the
streaming source's baked fallback keys on. Plain forwarding, no logging of
URLs or headers (they carry session tokens).
"""
import select
import socket
import sys
import threading

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8899


def pipe(a, b):
    try:
        while True:
            r, _, _ = select.select([a, b], [], [], 60)
            if not r:
                break
            for s in r:
                data = s.recv(65536)
                if not data:
                    return
                (b if s is a else a).sendall(data)
    except OSError:
        pass
    finally:
        for s in (a, b):
            try:
                s.close()
            except OSError:
                pass


def handle(client):
    try:
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = client.recv(4096)
            if not chunk:
                client.close()
                return
            head += chunk
        line = head.split(b"\r\n", 1)[0].decode(errors="replace")
        method, target, _ = line.split(" ", 2)
        if method != "CONNECT":
            client.sendall(b"HTTP/1.1 405 Method Not Allowed\r\n\r\n")
            client.close()
            return
        host, port = target.rsplit(":", 1)
        upstream = socket.create_connection((host, int(port)), timeout=15)
        client.sendall(b"HTTP/1.1 200 Connection Established\r\n\r\n")
        pipe(client, upstream)
    except Exception:
        try:
            client.close()
        except OSError:
            pass


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(64)
    print(f"kill_switch_proxy: listening on 127.0.0.1:{PORT} (kill me to cut the network)", flush=True)
    while True:
        c, _ = srv.accept()
        threading.Thread(target=handle, args=(c,), daemon=True).start()


if __name__ == "__main__":
    main()
