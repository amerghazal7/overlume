#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Self-test of check_shared_exports.sh's Mach-O branch with fake nm/otool (runs on any host):
# a clean library must PASS, a leaked export and a leaked dependency must each FAIL.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
w="$(mktemp -d)"; trap 'rm -rf "$w"' EXIT
printf '\xcf\xfa\xed\xfe fake mach-o\n' > "$w/lib.dylib"
cat > "$w/nm" <<'N'
#!/usr/bin/env bash
printf '0000 T __ZN8overlume15create_rendererEi\n'
[ -z "${LEAK_SYM:-}" ] || printf '0000 T __ZN4YAML4LoadERKNSt3__112basic_stringIcNS0_11char_traitsIcEENS0_9allocatorIcEEEE\n'
N
cat > "$w/otool" <<'N'
#!/usr/bin/env bash
printf '%s:\n\t@rpath/liboverlume.1.dylib (compatibility version 1.0.0)\n\t/usr/lib/libc++.1.dylib (c)\n\t/System/Library/Frameworks/Metal.framework/Metal (c)\n' "$2"
[ -z "${LEAK_DEP:-}" ] || printf '\t/opt/homebrew/lib/libyaml-cpp.0.8.dylib (c)\n'
N
chmod +x "$w/nm" "$w/otool"
check() { "$here/check_shared_exports.sh" "$w/nm" "$w/otool" "$w/lib.dylib" >/dev/null 2>&1; }
check || { echo "FAIL: clean Mach-O library rejected"; exit 1; }
LEAK_SYM=1 check && { echo "FAIL: leaked export accepted"; exit 1; }
LEAK_DEP=1 check && { echo "FAIL: leaked dependency accepted"; exit 1; }
echo "PASS: check_shared_exports.sh Mach-O branch"
