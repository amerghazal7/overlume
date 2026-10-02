#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Usage: check_package_elf.sh <package.tar.gz> <ELF machine, e.g. AArch64> [max-glibc]
# Unpacks the tarball and requires every shipped ELF file to be of that machine
# and to need no newer glibc than max (default 2.28; tools/release/check_glibc_floor.sh).
# Every static archive member must be of that machine too. READELF / OBJDUMP
# override the tools (cross builds: llvm-readelf / llvm-objdump).
set -euo pipefail
tgz="${1:?usage: $0 <tar.gz> <machine> [max-glibc]}"
machine="${2:?usage: $0 <tar.gz> <machine> [max-glibc]}"
max="${3:-2.28}"
here="$(cd "$(dirname "$0")" && pwd)"
readelf="${READELF:-readelf}"
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
tar -xzf "$tgz" -C "$tmp"
elfs=0 archives=0
while IFS= read -r -d '' f; do
    case "$f" in
    *.a)
        bad="$("$readelf" -h "$f" | awk '/Machine:/' | grep -vc "$machine" || true)"
        if [ "$bad" != 0 ]; then echo "FAIL machine: $f has $bad members that are not $machine"; exit 1; fi
        archives=$((archives + 1)) ;;
    *)
        [ "$(head -c4 "$f" | od -An -c | tr -d ' ')" = '177ELF' ] || continue
        if ! "$readelf" -h "$f" | grep -q "Machine:.*$machine"; then echo "FAIL machine: $f is not $machine"; exit 1; fi
        "$here/check_glibc_floor.sh" "$f" "$max"
        elfs=$((elfs + 1)) ;;
    esac
done < <(find "$tmp" -type f -print0)
if [ "$elfs" -eq 0 ]; then echo "FAIL: no ELF file in $tgz"; exit 1; fi
echo "PASS package ELF: $elfs ELF files and $archives archives, all $machine, glibc <= $max"
