#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Verify relative Markdown links/image refs in the docs set resolve on disk.

Stdlib only. Scans README.md, AGENTS.md, CONTRIBUTING.md (if present),
docs/README.md, docs/status.md, docs/runbooks/**, docs/design/**, docs/adr/**,
and docs/plans/2026-09-17-overlume-restructure.md -- the LIVE docs set. Skips
http(s)/mailto links and anchor-only (`#foo`) links. LICENSE, CONTRIBUTING.md,
CODE_OF_CONDUCT.md, SECURITY.md, and CHANGELOG.md now exist (open-source
restructure Task 6), so a dangling link to any of them is a hard failure like
any other missing target -- there is no more allowlist.

--all additionally scans every *.md under docs/plans/ (including
docs/plans/archive/), i.e. the historical plans/specs -- these are EXPECTED
to report broken links to their own pre-restructure paths (the path map
covers them, their content is intentionally left unedited). Not run by CI.

Exit 0 if the scanned set has no broken link, 1 otherwise.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

LIVE_FILES = [
    "README.md",
    "AGENTS.md",
    "CONTRIBUTING.md",
    "CODE_OF_CONDUCT.md",
    "SECURITY.md",
    "CHANGELOG.md",
    "docs/README.md",
    "docs/status.md",
    "docs/plans/archive/README.md",
    "docs/plans/2026-09-17-overlume-restructure.md",
]
LIVE_DIRS = ["docs/runbooks", "docs/design", "docs/adr"]

LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
# Strip fenced/inline code spans before scanning: C++/Python snippets can
# contain `[capture](args)` lambda syntax that would otherwise false-positive
# as a Markdown link.
FENCED_CODE_RE = re.compile(r"```.*?```", re.DOTALL)
INLINE_CODE_RE = re.compile(r"`[^`\n]+`")


def collect_files(scan_all: bool) -> list[Path]:
    files: list[Path] = []
    for rel in LIVE_FILES:
        p = REPO_ROOT / rel
        if p.is_file():
            files.append(p)
    for reldir in LIVE_DIRS:
        d = REPO_ROOT / reldir
        if d.is_dir():
            files.extend(sorted(d.rglob("*.md")))
    if scan_all:
        plans_dir = REPO_ROOT / "docs" / "plans"
        if plans_dir.is_dir():
            files.extend(sorted(plans_dir.rglob("*.md")))
    # de-dupe, keep order
    seen = set()
    ordered = []
    for f in files:
        if f not in seen:
            seen.add(f)
            ordered.append(f)
    return ordered


def extract_targets(text: str) -> list[str]:
    return LINK_RE.findall(text)


def is_skippable(target: str) -> bool:
    target = target.strip()
    if not target:
        return True
    if target.startswith("#"):
        return True
    if "://" in target:
        return True
    if target.startswith("mailto:"):
        return True
    return False


def normalize(target: str) -> str:
    # Drop an optional trailing ` "title"` and any #fragment.
    target = target.strip()
    if " " in target:
        target = target.split(" ", 1)[0]
    target = target.split("#", 1)[0]
    return target


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--all",
        action="store_true",
        help="also scan docs/plans/** (historical plans; expected to report old paths)",
    )
    args = ap.parse_args()

    broken = []

    for f in collect_files(args.all):
        text = f.read_text(encoding="utf-8", errors="replace")
        text = FENCED_CODE_RE.sub("", text)
        text = INLINE_CODE_RE.sub("", text)
        for raw_target in extract_targets(text):
            if is_skippable(raw_target):
                continue
            target = normalize(raw_target)
            if not target or target.startswith("/"):
                # Root-relative links aren't resolvable against a file's own
                # directory the way this repo's docs use links; skip rather
                # than guess.
                continue
            resolved = (f.parent / target).resolve()
            if resolved.exists():
                continue
            rel_f = f.relative_to(REPO_ROOT)
            broken.append(f"{rel_f}: -> {raw_target}")

    if broken:
        print(f"Broken links ({len(broken)}):")
        for b in broken:
            print(f"  {b}")
        return 1

    print("OK: no broken links")
    return 0


if __name__ == "__main__":
    sys.exit(main())
