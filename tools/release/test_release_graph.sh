#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Static check of release.yml's draft-until-green graph (dry runs skip create/publish/finalize, so only
# this fails when it is reverted). Usage: test_release_graph.sh [WORKFLOW.yml]. Needs python3 + PyYAML.
set -euo pipefail
wf="${1:-$(cd "$(dirname "$0")/../.." && pwd)/.github/workflows/release.yml}"
python3 - "$wf" <<'PY'
import re, sys, yaml
J = yaml.safe_load(open(sys.argv[1]))["jobs"]
errs = []
def need(n): v = J[n].get("needs", []); return [v] if isinstance(v, str) else list(v)
def anc(n, seen=None):
    seen = set() if seen is None else seen
    for m in need(n):
        if m not in seen: seen.add(m); anc(m, seen)
    return seen
def runs(n): return "\n".join(s.get("run", "") for s in J[n].get("steps", []))
def ok(c, m):
    if not c: errs.append(m)

# (a) the release is created as a draft
ok(re.search(r"gh release create\b[^\n]*--draft|gh release create\b(?:[^\n]*\\\n)*[^\n]*--draft", runs("create")), "create: gh release create lacks --draft")

# (b) every build/package/sign job is an ancestor of sums; sums.if demands success of each direct need
gates = {"create", "sums", "channels", "finalize", "pages"}
for n in J:
    if n not in gates and not n.startswith("publish-"):
        ok(n in anc("sums"), f"{n} is not an ancestor of sums")
for m in need("sums"):
    ok(f"needs.{m}.result == 'success'" in J["sums"]["if"], f"sums.if does not require {m} success")

# (c) publishers wait for sums + channels; Maven (irreversible) last
for p in ("publish-homebrew", "publish-swiftpm"):
    ok({"sums", "channels"} <= anc(p), f"{p} must (transitively) need sums and channels")
ok({"sums", "channels", "publish-homebrew", "publish-swiftpm"} <= set(need("publish-maven")), "publish-maven must need sums, channels and both other publishers")

# The `needs` edge alone does not stop a publisher (every gate uses !cancelled()): the if-terms do.
for p in ("publish-homebrew", "publish-swiftpm", "publish-maven"):
    for t in ("needs.sums.result == 'success'", "needs.channels.result == 'success'"):
        ok(t in J[p]["if"], f"{p}.if lacks {t}")
for p in ("publish-homebrew", "publish-swiftpm"):
    ok("github.ref_type == 'tag'" in J[p]["if"], f"{p}.if lacks the tag-only term")

# (d) finalize needs everything and requires success of each; pages follows finalize
pubs = ["publish-homebrew", "publish-swiftpm", "publish-maven"]
for m in ["sums", "channels"] + pubs:
    ok(m in need("finalize"), f"finalize does not need {m}")
    ok(f"needs.{m}.result == 'success'" in J["finalize"]["if"], f"finalize.if does not require {m} success")
ok("finalize" in need("pages"), "pages must need finalize")
ok("needs.finalize.result == 'success'" in J["pages"]["if"], "pages.if does not require finalize success")

# (e) release mutation and contents: write confined to create, sums, finalize
allowed = {"create", "sums", "finalize"}
for n, j in J.items():
    ok(n in allowed or not re.search(r"gh release (upload|create|edit)", runs(n)), f"{n} mutates the release")
    w = (j.get("permissions") or {}).get("contents") == "write"
    ok(not w or n in allowed, f"{n} has contents: write")

# (f) pinned actions, timeouts -- timeouts are checked in EVERY workflow file (the plan says "every job"),
# not only release.yml; reusable-workflow calls (`uses:` jobs) inherit the callee's.
import glob, os
for wf in sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), "*.yml"))):
    for n, j in (yaml.safe_load(open(wf)).get("jobs") or {}).items():
        ok("timeout-minutes" in j or "uses" in j, f"{os.path.basename(wf)}: job {n} has no timeout-minutes")
txt = open(sys.argv[1]).read()
for m in re.finditer(r"^\s*-?\s*uses:\s*(\S+)(.*)$", txt, re.M):
    ok(re.fullmatch(r".+@[0-9a-f]{40}", m.group(1)) and re.search(r"#\s*v\d", m.group(2)), f"unpinned action: {m.group(1)}")

# (g) the sums upload checks isDraft first
up = next((s.get("run", "") for s in J["sums"]["steps"] if "gh release upload" in s.get("run", "")), "")
upif = next((s.get("if", "") for s in J["sums"]["steps"] if "gh release upload" in s.get("run", "")), "")
ok("env.DRY_RUN != 'true'" in upif and "github.ref_type == 'tag'" in upif, "sums upload step if lacks the dry-run/tag guards")
ok("isDraft" in up and up.index("isDraft") < up.index("gh release upload"), "sums upload is not preceded by the isDraft check")

if errs: print("\n".join("FAIL: " + e for e in errs)); sys.exit(1)
print("PASS: release.yml graph")
PY
