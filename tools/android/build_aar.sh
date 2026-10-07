#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_aar.sh ZIP_DIR OUT_DIR -- merge the per-ABI `cpack -G ZIP` outputs
# (ZIP_DIR/overlume-<ver>-android-<abi>.zip, any subset of the four ABIs) into
#   OUT_DIR/overlume-<ver>-android.zip   <abi>/{lib,include,share,...} for every ABI, both components
#   OUT_DIR/overlume-<ver>-android.aar   Prefab v2 package (schema 2); no Gradle involved:
#       prefab/modules/overlume         shared liboverlume.so (+ jni/<abi>/ copy so APKs package it);
#                                       stl "none": the STL is baked in and private, the API is POD-only
#                                       (Prefab rejects "c++_static" shared libs for every consumer)
#       prefab/modules/overlume_static  one merged liboverlume.a (liboverlume.a + its dep archives)
# Needs unzip, zip and the NDK's llvm-ar (ANDROID_NDK_HOME).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
zips="$(cd "${1:?usage: $0 ZIP_DIR OUT_DIR}" && pwd)"
mkdir -p "${2:?usage: $0 ZIP_DIR OUT_DIR}"; out="$(cd "$2" && pwd)"
: "${ANDROID_NDK_HOME:?ANDROID_NDK_HOME is not set}"
llvm="$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin"
ar="$llvm/llvm-ar"; strip="$llvm/llvm-strip"
stage="$(mktemp -d)"; trap 'rm -rf "$stage"' EXIT

ver=""
for z in "$zips"/overlume-*-android-*.zip; do
    base="$(basename "$z" .zip)"                    # overlume-1.0.0-android-arm64-v8a
    ver="${base#overlume-}"; ver="${ver%%-android-*}"
    abi="${base#*-android-}"
    mkdir -p "$stage/abis/$abi"
    unzip -q "$z" -d "$stage/abis/$abi"
done
[ -n "$ver" ] || { echo "FAIL: no overlume-*-android-<abi>.zip in $zips" >&2; exit 1; }

rm -f "$out/overlume-$ver-android.zip" "$out/overlume-$ver-android.aar"  # zip -r would update a stale archive in place
(cd "$stage/abis" && zip -qr -X "$out/overlume-$ver-android.zip" .)

p="$stage/aar/prefab"
mkdir -p "$p/modules/overlume/libs" "$p/modules/overlume_static/libs"
sed "s/@VERSION@/$ver/" "$here/prefab/prefab.json.in" > "$p/prefab.json"
cp "$here/prefab/module.json" "$p/modules/overlume/module.json"
cp "$here/prefab/module_static.json" "$p/modules/overlume_static/module.json"
abis=(); for d in "$stage/abis"/*/; do abis+=("$(basename "$d")"); done
first_abi="${abis[0]}"
for m in overlume overlume_static; do
    mkdir -p "$p/modules/$m/include"
    cp -r "$stage/abis/$first_abi/include/overlume" "$p/modules/$m/include/"
done
for abi in "${abis[@]}"; do
    t="$stage/abis/$abi"
    d="$p/modules/overlume/libs/android.$abi"; mkdir -p "$d" "$stage/aar/jni/$abi"
    cp "$t/lib/liboverlume.so" "$d/liboverlume.so"; cp "$t/lib/liboverlume.so" "$stage/aar/jni/$abi/"
    sed "s/@ABI@/$abi/; s/@STL@/none/; s/@STATIC@/false/" "$here/prefab/abi.json.in" > "$d/abi.json"
    d="$p/modules/overlume_static/libs/android.$abi"; mkdir -p "$d"
    {   echo "CREATE $d/liboverlume.a"
        echo "ADDLIB $t/lib/liboverlume.a"
        for a in "$t"/lib/overlume/deps/*.a; do echo "ADDLIB $a"; done
        echo SAVE; echo END
    } | "$ar" -M
    "$strip" --strip-debug "$d/liboverlume.a"   # keeps the AAR (Maven Central bundle limit) small
    sed "s/@ABI@/$abi/; s/@STL@/c++_static/; s/@STATIC@/true/" "$here/prefab/abi.json.in" > "$d/abi.json"
done
cp "$here/prefab/AndroidManifest.xml" "$stage/aar/AndroidManifest.xml"
: > "$stage/aar/R.txt"
(cd "$stage" && mkdir -p jar/META-INF && printf 'Manifest-Version: 1.0\r\n\r\n' > jar/META-INF/MANIFEST.MF \
    && (cd jar && zip -qr -X ../aar/classes.jar META-INF))   # AGP expects a classes.jar, even an empty one
# AAR = zip, AndroidManifest.xml first.
(cd "$stage/aar" && zip -qr -X "$out/overlume-$ver-android.aar" AndroidManifest.xml . )
echo "PASS: ${abis[*]} -> overlume-$ver-android.{zip,aar}"
