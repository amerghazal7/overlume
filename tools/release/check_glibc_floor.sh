#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Usage: check_glibc_floor.sh <elf> [max=2.28]
# PASS iff the highest GLIBC_x.y symbol version the ELF needs is <= max.
# OBJDUMP overrides the tool (cross builds: llvm-objdump reads foreign ELF).
set -euo pipefail
elf="${1:?usage: $0 <elf> [max-glibc]}"
max="${2:-2.28}"
top="$("${OBJDUMP:-objdump}" -T "$elf" 2>/dev/null | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1)"
top="${top#GLIBC_}"
if [[ -n "$top" && "$(printf '%s\n%s\n' "$top" "$max" | sort -V | tail -1)" != "$max" ]]; then
    echo "FAIL glibc floor: $elf needs GLIBC_$top > $max"
    exit 1
fi
echo "PASS glibc floor: $elf needs GLIBC_${top:-none} <= $max"
