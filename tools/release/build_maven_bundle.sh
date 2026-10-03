#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_maven_bundle.sh AAR OUT_ZIP -- Central Portal bundle for io.github.amerghazal7:overlume:
#   io/github/amerghazal7/overlume/<ver>/overlume-<ver>{.aar,.pom,-sources.jar,-javadoc.jar}
# each with .asc (GPG detached, armored), .md5, .sha1 (and the same checksums for the .asc).
# <ver> comes from the AAR name (overlume-<ver>-android.aar). GPG material comes from env
# OVERLUME_GPG_PRIVATE_KEY (armored) and OVERLUME_GPG_PASSPHRASE, imported into a temporary
# GNUPGHOME that is deleted on exit; secret values are never printed.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
aar="$(cd "$(dirname "${1:?usage: $0 AAR OUT_ZIP}")" && pwd)/$(basename "$1")"
out_arg="${2:?usage: $0 AAR OUT_ZIP}"
: "${OVERLUME_GPG_PRIVATE_KEY:?OVERLUME_GPG_PRIVATE_KEY is not set}"
: "${OVERLUME_GPG_PASSPHRASE:?OVERLUME_GPG_PASSPHRASE is not set}"
ver="$(basename "$aar" .aar)"; ver="${ver#overlume-}"; ver="${ver%-android}"

work="$(mktemp -d)"
GNUPGHOME="$work/gnupg"; export GNUPGHOME
trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$work"' EXIT
mkdir -m 700 "$GNUPGHOME"
printf '%s\n' "$OVERLUME_GPG_PRIVATE_KEY" | gpg --batch --quiet --import
fpr="$(gpg --batch --list-secret-keys --with-colons | awk -F: '/^fpr:/ {print $10; exit}')"

d="$work/bundle/io/github/amerghazal7/overlume/$ver"; mkdir -p "$d"
cp "$aar" "$d/overlume-$ver.aar"
sed "s/@VERSION@/$ver/" "$repo/packaging/maven/overlume.pom.in" > "$d/overlume-$ver.pom"

src="$work/src"; mkdir -p "$src"
cp -r "$repo/overlume/include/overlume" "$src/"; cp "$repo/overlume/README.md" "$src/"
(cd "$src" && zip -qr -X "$d/overlume-$ver-sources.jar" .)
jd="$work/javadoc"; mkdir -p "$jd"
cat > "$jd/README.txt" <<DOC
Overlume $ver is a native (C++) Prefab package; there is no Java API.
API reference: https://amerghazal7.github.io/overlume/
DOC
(cd "$jd" && zip -qr -X "$d/overlume-$ver-javadoc.jar" .)

for f in "$d"/overlume-"$ver"*; do
    printf '%s' "$OVERLUME_GPG_PASSPHRASE" | gpg --batch --yes --quiet --pinentry-mode loopback \
        --passphrase-fd 0 --armor --detach-sign -u "$fpr" -o "$f.asc" "$f"
    gpg --batch --quiet --verify "$f.asc" "$f" 2>/dev/null
done
for f in "$d"/*; do
    md5sum "$f" | cut -d' ' -f1 | tr -d '\n' > "$f.md5"
    sha1sum "$f" | cut -d' ' -f1 | tr -d '\n' > "$f.sha1"
done
mkdir -p "$(dirname "$out_arg")"
out="$(cd "$(dirname "$out_arg")" && pwd)/$(basename "$out_arg")"
rm -f "$out"
(cd "$work/bundle" && zip -qr -X "$out" io)
echo "PASS: $(basename "$out") ($(find "$d" -type f | wc -l) files, key ${fpr: -16})"
