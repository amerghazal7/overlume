#!/usr/bin/env bash
# Usage: check_notice_strings.sh LIB [NOTICE] — each Filament-internal third-party string must be named in NOTICE.
set -u
lib="${1:?usage: $0 LIB [NOTICE]}"; notice="${2:-$(dirname "$0")/../../NOTICE}"; rc=0
[ -f "$lib" ] || { echo "FAIL: no such library: $lib"; exit 1; }
# One needle per Filament-internal notice (FSR1/AMD, FXAA/NVIDIA + G3D, SSR/Mara, Oklab/Ottosson, Drobot).
for s in 'Advanced Micro Devices' 'NVIDIA CORPORATION' 'Morgan McGuire' 'Michael Mara' 'Ottosson' 'Drobot'; do
  strings -a "$lib" | grep -qiF "$s" && f=found || f=absent
  if tr -s " \n" "  " < "$notice" | grep -qiF "$s"; then echo "ok   $s (in $lib: $f; in NOTICE)"; else echo "FAIL $s not in NOTICE (in $lib: $f)"; rc=1; fi
done
[ $rc = 0 ] && echo PASS || echo FAIL; exit $rc
