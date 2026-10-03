#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
"""Local stand-in for the Sonatype Central Portal publisher API endpoints publish_maven_central.sh uses.

  stub_central_portal.py PORT_FILE LOG_FILE      (env STUB_USER, STUB_PASSWORD, STUB_SCENARIO)

POST /api/v1/publisher/upload?publishingType=..&name=..   multipart `bundle` -> 201, deployment id
POST /api/v1/publisher/status?id=..                        PENDING, VALIDATING, then VALIDATED or FAILED
                                                           (AUTOMATIC: ... PUBLISHING, PUBLISHED)
DELETE /api/v1/publisher/deployment/<id>                   204 in VALIDATED/FAILED, else 400
The bundle is checked like Central does it: every artifact needs .asc/.md5/.sha1 and a .pom must
exist; STUB_SCENARIO=reject forces FAILED. The log holds `METHOD path HTTP code` lines only.
"""
import base64, email, email.policy, http.server, io, json, os, sys, threading, uuid, zipfile
from urllib.parse import urlparse, parse_qs

EXPECT = "Bearer " + base64.b64encode(
    f"{os.environ['STUB_USER']}:{os.environ['STUB_PASSWORD']}".encode()).decode()
SCENARIO = os.environ.get("STUB_SCENARIO", "ok")
LOG = sys.argv[2]
deployments = {}
lock = threading.Lock()


def check_bundle(data):
    errors = {}
    try:
        names = set(zipfile.ZipFile(io.BytesIO(data)).namelist())
    except zipfile.BadZipFile:
        return {"bundle": ["not a zip file"]}
    arts = [n for n in names if not n.endswith(("/", ".asc", ".md5", ".sha1"))]
    if not any(n.endswith(".pom") for n in arts):
        errors["pom"] = ["no .pom in bundle"]
    for n in arts:
        miss = [e for e in (".asc", ".md5", ".sha1") if n + e not in names]
        if miss:
            errors[n] = [f"missing {e}" for e in miss]
    return errors


class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def reply(self, code, body=b"", ctype="text/plain"):
        with lock, open(LOG, "a") as f:
            f.write(f"{self.command} {urlparse(self.path).path} HTTP {code}\n")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def authed(self):
        if self.headers.get("Authorization") != EXPECT:
            self.reply(401, b"unauthorized")
            return False
        return True

    def do_POST(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n)
        if not self.authed():
            return
        if u.path == "/api/v1/publisher/upload":
            msg = email.message_from_bytes(
                b"Content-Type: " + self.headers["Content-Type"].encode() + b"\r\n\r\n" + raw,
                policy=email.policy.HTTP)
            parts = [p for p in msg.iter_parts() if p.get_param("name", header="content-disposition") == "bundle"]
            if not parts:
                return self.reply(400, b"no bundle part")
            errors = check_bundle(parts[0].get_payload(decode=True))
            if SCENARIO == "reject":
                errors["signature"] = ["Invalid signature for file overlume.aar (key not found on keyservers)"]
            did = str(uuid.uuid4())
            deployments[did] = {"polls": 0, "errors": errors, "type": q.get("publishingType", [""])[0],
                                "name": q.get("name", [""])[0]}
            return self.reply(201, did.encode())
        if u.path == "/api/v1/publisher/status":
            d = deployments.get(q.get("id", [""])[0])
            if d is None:
                return self.reply(404, b"unknown deployment")
            d["polls"] += 1
            seq = ["PENDING", "VALIDATING", "FAILED" if d["errors"] else "VALIDATED"]
            if not d["errors"] and d["type"] == "AUTOMATIC":
                seq += ["PUBLISHING", "PUBLISHED"]
            d["state"] = seq[min(d["polls"], len(seq)) - 1]
            body = {"deploymentId": q["id"][0], "deploymentName": d["name"], "deploymentState": d["state"],
                    "purls": [], "errors": d["errors"] if d["state"] == "FAILED" else {}}
            return self.reply(200, json.dumps(body).encode(), "application/json")
        self.reply(404)

    def do_DELETE(self):
        if not self.authed():
            return
        did = urlparse(self.path).path.rsplit("/", 1)[-1]
        d = deployments.get(did)
        if d is None:
            return self.reply(404)
        if d.get("state") not in ("VALIDATED", "FAILED"):
            return self.reply(400, b"deployment can only be dropped when VALIDATED or FAILED")
        del deployments[did]
        self.reply(204)


srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), H)
open(sys.argv[1], "w").write(str(srv.server_address[1]))
srv.serve_forever()
