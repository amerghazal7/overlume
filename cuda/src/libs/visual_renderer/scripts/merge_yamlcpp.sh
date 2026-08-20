#!/usr/bin/env bash
# POST_BUILD step on the `visual_renderer` target (Epic 2 Task 1 / VM-020,
# Step 0.2). Goal: the node links a SECOND, independently-built
# gcc/libstdc++ yaml-cpp (yaml_cpp_vendor) in the same process as this
# archive's bundled clang/libc++ one, and this archive's copy must never be
# resolvable from, or resolve, the node's copy -- ANY shared symbol name
# between the two is a latent ABI hazard (see below), regardless of which
# archive happens to "win" it on a given link line.
#
# THREE recipes were tried, in order, and this comment records why the
# first two are not what's wired in below (do not reintroduce either
# without re-reading this):
#
# 1. Plan's literal recipe: compile yaml-cpp with `-fvisibility=hidden
#    -fvisibility-inlines-hidden`, `ld -r` everything into one relocatable
#    object, `objcopy --localize-hidden` it. FAILED to link at all: hiding
#    the WHOLE yaml-cpp target also hides incidental libc++ template
#    instantiations it merely happens to pull in (std::vector<std::string>,
#    __clang_call_terminate, ...) that other, unrelated code in the same
#    final link needs as ordinary global/weak COMDAT symbols -- localizing
#    those breaks COMDAT folding program-wide ("... defined in discarded
#    section", "undefined hidden symbol ... PIE object").
#
# 2. Narrowed to `objcopy --localize-symbols` on exactly the symbols whose
#    mangled name contains the `YAML` namespace token (this script's
#    previous version). Passed every check it was written against --
#    including the plan's own two `nm` checks and the Step 0.3 coexistence
#    test -- then FAILED the first time real node-side code (this task's
#    profile.cpp) called `YAML::Node::operator[]("somekey")`: GNU ld died
#    with "`YAML::Node::operator[]<char [5]>(...)' referenced in section
#    `.text': defined in discarded section". Root cause: that accessor (and
#    three others -- EnsureNodeExists/Mark/mark_defined) has no std:: type
#    in its signature, so it mangles IDENTICALLY whether instantiated by
#    clang/libc++ (in this archive) or gcc/libstdc++ (in profile.cpp.o) --
#    a bigger collision set than "Exception's ctor/dtor" alone, first
#    exercised by this task's actual yaml usage. Once localized, those
#    symbols still sit in ELF COMDAT groups sharing a group SIGNATURE with
#    profile.cpp.o's own same-named instantiation; the final link's ordinary
#    comdat folding picks ONE survivor per signature across the WHOLE link
#    (independent of symbol binding), and a LOCALIZED reference inside this
#    archive cannot be redirected to whichever copy wins that fold.
#
# 3. Tried the plan's own named fallback next ("keep yaml-cpp
#    hidden-but-GLOBAL, no localize, no merge, keep the node's _yamlcpp_a
#    import, accept that link order decides"). This LINKS -- and then
#    silently corrupts data: with two same-named, differently-ABI'd copies
#    of `YAML::Node` on one link line, `load_profile()` in the actual node
#    test binary returned a `Profile` with the right error count (0) and
#    the WRONG row count (0 instead of 16), then the very next gtest
#    segfaulted. A `YAML::Node` returned across that boundary carries
#    std::shared_ptr-based internals whose layout is not guaranteed
#    identical between libc++ and libstdc++ -- an empirically reproduced
#    ABI mismatch, not a theoretical one.
#
# What's actually wired in (this file, current version): RENAME every
# `YAML::`-namespaced defined symbol in the merged object (`objcopy
# --redefine-syms`), rather than merely changing its binding. A rename
# means no other object on any link line -- the node's own yaml_cpp_vendor
# archive included -- can ever produce a matching symbol/COMDAT-group name
# for the final linker to arbitrate between; recipe 2's failure mode (a
# localized reference orphaned by comdat folding) and recipe 3's failure
# mode (an ABI-incompatible fold silently "winning") both require a SHARED
# name to occur at all. Internal cross-references within the merged object
# are relocations keyed by symbol-table INDEX, not by name string, so
# renaming every entry cannot break anything the preceding `ld -r` already
# resolved -- it only changes what an EXTERNAL object could address it as.
# No -fvisibility flag is needed on yaml-cpp for this recipe.
#
# Verified: `nm libvisual_renderer.a | grep ' U .*YAML'` prints nothing (no
# undefined YAML:: refs survive the merge); `nm --defined-only
# libvisual_renderer.a | awk '{print $3}' | grep -E '^_Z.*4YAML'` prints
# nothing (no DEFINED symbol whose mangled name still starts with the YAML::
# token survives the rename, so nothing can collide with it) -- note this is
# NOT the same as a plain `grep '_ZN4YAML'` over the whole `nm` line: the
# rename prefixes the mangled name (`mpviz_vendored_yaml__ZN4YAML...`), so an
# unanchored substring grep still matches all 655 renamed symbols and would
# wrongly look broken on a CORRECT build; anchoring on column 3 (the name)
# with `^_Z.*4YAML` is what actually discriminates. Also: the Step 0.3
# coexistence test passes; and -- the check the first two recipes could not
# pass --
# micropilot_visualization_node's own `test_profile` gtest, which actually
# calls both yaml-cpps' `Node::operator[]` in one process via
# `load_profile()` + `mpviz::theme_parses()`, gets the CORRECT row count.
#
# The final archive is built FRESH from the merged object alone. Do NOT
# `ar r`/`ar rcs` the merged .o into the archive libvisual_renderer.a
# already has: `ar r` replaces members by NAME, and visual_renderer_merged.o
# shares no name with renderer.cpp.o/theme.cpp.o, so those stale members
# would survive untouched in the archive index; the node link would then
# resolve mpviz::create_renderer out of the STALE renderer.cpp.o/theme.cpp.o
# (earlier in the index) instead of the merged/renamed one, and die on
# `undefined reference to YAML::LoadFile` -- the yaml symbols exist only
# under their renamed names inside merged.o, which those stale members
# never reference. Deleting the old archive file and `ar crs`-ing only
# merged.o into a brand-new one is what actually replaces every member.
set -euo pipefail

ar_tool="$1"; ld_tool="$2"; objcopy_tool="$3"; nm_tool="$4"
work_dir="$5"; merged_o="$6"; yamlcpp_archive="$7"; target_archive="$8"
shift 8
# Remaining args: visual_renderer's own .o files ($<TARGET_OBJECTS:visual_renderer>).
own_objs=("$@")

rm -rf "$work_dir"
mkdir -p "$work_dir"
( cd "$work_dir" && "$ar_tool" x "$yamlcpp_archive" )

shopt -s nullglob
yaml_objs=("$work_dir"/*.o)
if [ ${#yaml_objs[@]} -eq 0 ]; then
  echo "merge_yamlcpp.sh: no .o files extracted from $yamlcpp_archive" >&2
  exit 1
fi

"$ld_tool" -r -o "$merged_o" "${own_objs[@]}" "${yaml_objs[@]}"

# Rename every defined YAML::-namespaced symbol (found by the Itanium
# mangled-name token for the `YAML` namespace, `4YAML`) to a name nothing
# else on any link line will ever produce. `--redefine-syms` takes
# "old new" pairs, one per line.
rename_map="${merged_o}.yaml_rename.txt"
"$nm_tool" --defined-only "$merged_o" | awk '{print $3}' | grep '4YAML' | sort -u \
  | awk '{print $1, "mpviz_vendored_yaml_" $1}' > "$rename_map"
if [ ! -s "$rename_map" ]; then
  echo "merge_yamlcpp.sh: found zero YAML:: symbols in $merged_o -- something upstream broke" >&2
  exit 1
fi
"$objcopy_tool" --redefine-syms="$rename_map" "$merged_o"

rm -f "$target_archive"
"$ar_tool" crs "$target_archive" "$merged_o"
