#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Usage: check_package_elf.sh <package-dir | package...> <ELF machine, e.g. AArch64> [max-glibc]
# Unpacks every .tar.gz, .rpm (rpm2cpio) and .deb (ar) given (or found in the directory) and
# requires every shipped ELF file AND every member of every static archive to be of that
# machine (rpmbuild's brp-strip once rewrote a foreign-arch member to e_machine 0), and every
# ELF file to need no newer glibc than max (default 2.28; tools/release/check_glibc_floor.sh).
# READELF / OBJDUMP override the tools (cross builds: llvm-readelf / llvm-objdump).
set -euo pipefail
usage="usage: $0 <dir|package>... <machine> [max-glibc]"
[ "$#" -ge 2 ] || { echo "$usage" >&2; exit 2; }
args=("$@")
# The machine is the first argument that is not an existing path (the last one or two).
machine="" max=2.28 inputs=()
for a in "${args[@]}"; do
    if [ -e "$a" ]; then inputs+=("$a")
    elif [ -z "$machine" ]; then machine="$a"
    else max="$a"; fi
done
[ -n "$machine" ] && [ "${#inputs[@]}" -gt 0 ] || { echo "$usage" >&2; exit 2; }
here="$(cd "$(dirname "$0")" && pwd)"
readelf="${READELF:-readelf}"
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT

pkgs=()
for i in "${inputs[@]}"; do
    if [ -d "$i" ]; then
        while IFS= read -r -d '' p; do pkgs+=("$p"); done \
            < <(find "$i" -maxdepth 1 -type f \( -name '*.tar.gz' -o -name '*.rpm' -o -name '*.deb' \) -print0)
    else pkgs+=("$i"); fi
done
[ "${#pkgs[@]}" -gt 0 ] || { echo "FAIL: no package found in ${inputs[*]}"; exit 1; }

unpack() { # <package> <dir>
    case "$1" in
    *.tar.gz) tar -xzf "$1" -C "$2" ;;
    *.rpm) (cd "$2" && rpm2cpio "$1" | cpio -idm --quiet) ;;
    *.deb) (cd "$2" && d="$(ar t "$1" | grep '^data\.tar')" && ar x "$1" "$d" && tar -xf "$d") ;;
    *) echo "FAIL: unknown package type $1"; exit 1 ;;
    esac
}

elfs=0 archives=0 members=0
for pkg in "${pkgs[@]}"; do
    pkg="$(realpath "$pkg")"
    d="$tmp/$(basename "$pkg")"; mkdir "$d"
    unpack "$pkg" "$d"
    while IFS= read -r -d '' f; do
        rel="${pkg##*/}:${f#"$d"/}"
        case "$f" in
        *.a)
            out="$("$readelf" -h "$f")"
            n="$(grep -c 'Machine:' <<<"$out" || true)"
            bad="$(grep 'Machine:' <<<"$out" | grep -vc "$machine" || true)"
            if [ "$n" -eq 0 ] || [ "$bad" != 0 ]; then
                echo "FAIL machine: $rel has $bad of $n members that are not $machine"; exit 1
            fi
            members=$((members + n)); archives=$((archives + 1)) ;;
        *)
            [ "$(head -c4 "$f" | od -An -c | tr -d ' ')" = '177ELF' ] || continue
            if ! "$readelf" -h "$f" | grep -q "Machine:.*$machine"; then echo "FAIL machine: $rel is not $machine"; exit 1; fi
            "$here/check_glibc_floor.sh" "$f" "$max"
            elfs=$((elfs + 1)) ;;
        esac
    done < <(find "$d" -type f -print0)
done
if [ "$elfs" -eq 0 ]; then echo "FAIL: no ELF file in ${pkgs[*]}"; exit 1; fi
echo "PASS package ELF: ${#pkgs[@]} packages, $elfs ELF files and $archives archives ($members members), all $machine, glibc <= $max"
