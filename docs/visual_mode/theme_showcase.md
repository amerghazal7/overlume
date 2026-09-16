# Theme showcase — palette-iteration harness (2026-09-16)

A fast, repeatable single-frame capture that exercises the **whole** theme
palette at once, for judging "which candidate theme looks best" by eye. Built
for the ref-2 palette-iteration loop: edit a theme YAML, re-run, look at the
PNG — no rebuild of the renderer itself needed (YAML is read at runtime), no
golden comparison to keep in sync (this test never asserts pixel content, it
only reports — same convention as `EnvSourceCapture.*` in
`test_environment_stream.cpp`).

Opt-in and env-gated, same shape as every other manual capture in this repo:
never runs under a plain `ctest`/`ci_visual_mode.sh` invocation, never needs
network or a token.

## Re-render command

From the repo root, after building `test_theme_showcase` once
(`cmake --build cuda/src/libs/visual_renderer/build --target test_theme_showcase`):

```bash
cd cuda/src/libs/visual_renderer/build

# A shipped theme (assets/themes/*.yaml) -- MPVIZ_SHOWCASE_THEME_DIR defaults
# to the shipped dir, so it can be omitted here:
MPVIZ_SHOWCASE=1 MPVIZ_SHOWCASE_THEME=dark_adas \
MPVIZ_SHOWCASE_OUT=/tmp/showcase_dark_adas.png \
    ./test_theme_showcase --gtest_filter='ThemeShowcase.Capture'

# A candidate variant (assets/theme_variants/*.yaml) -- point
# MPVIZ_SHOWCASE_THEME_DIR at the variants directory:
MPVIZ_SHOWCASE=1 MPVIZ_SHOWCASE_THEME=dark_ref2 \
MPVIZ_SHOWCASE_THEME_DIR=$(pwd)/../assets/theme_variants \
MPVIZ_SHOWCASE_OUT=/tmp/showcase_dark_ref2.png \
    ./test_theme_showcase --gtest_filter='ThemeShowcase.Capture'
```

Iterating on a candidate: edit the variant's YAML in place, re-run the same
command (no rebuild) — `theme.cpp`'s loader reads it fresh every run. stderr
logs `mean_luminance`/`non_background_fraction` for the render as a sanity
check that the capture actually produced content.

Env contract (also documented in the test file's own header comment):

| Var | Meaning | Default |
|---|---|---|
| `MPVIZ_SHOWCASE` | `1` enables the capture; unset -> `GTEST_SKIP()` | unset |
| `MPVIZ_SHOWCASE_THEME` | theme name (yaml stem) to load | `dark_adas` |
| `MPVIZ_SHOWCASE_THEME_DIR` | directory to load `<theme>.yaml` from | shipped `assets/themes/` |
| `MPVIZ_SHOWCASE_OUT` | output PNG path | `/tmp/theme_showcase_<theme>.png` |

## What the scene contains

One 1280×960 frame, elevated 3/4 chase-cam pose (behind/above the ego,
looking forward-and-across along the road toward the buildings ahead —
echoing `assets/visualization-reference-2.jpg`'s own framing), containing:

- **Baked environment buildings** — `tests/fixtures/environment_test_town_0`,
  the same fixture every other environment test in this suite uses.
- **Road** — surface polygon, left/right lane boundaries, centerline,
  a crosswalk, and both road edges, hand-built the same way
  `test_map_elements.cpp` builds individual `MapElement`s (no existing helper
  assembles a whole cross-section at once).
- **Ego** — the shipped clay-box fallback (`palette.ego`); no dedicated ego
  glTF ships in this repo, so this IS the real default appearance, not a
  test-only stand-in.
- **TrackedObjects** — one of every class (`CAR`/`TRUCK_VAN`/`BUS`/
  `PEDESTRIAN`/`CYCLIST`/`UNKNOWN`), reused verbatim from `golden.hpp`'s
  `make_mixed_class_objects()`, translated into this scene's frame. One
  `CRITICAL`-severity `AlertPolygon` (hand-built — no existing helper
  produces that severity) straddles the `CAR`, so the hazard/coral accent
  appears.
- **All three ribbon roles** (`BEHAVIOR`/`GLOBAL`/`LOCAL`) — reused verbatim
  from `make_three_role_ribbons()`.
- **The ground grid** — reused verbatim from `make_two_layer_grids()` (two
  OGM layers, dynamic + gradient).

Every reused helper's own coordinates are defined relative to a local
`(0,0,0)` origin; this test translates all of them by one shared offset so
they land together with the (un-movable, already baked-in-map-frame)
buildings in a single frame — see the test file's own `kSceneOrigin` comment.

## The two candidates are NOT shipped

`assets/theme_variants/light_ref2.yaml` and `dark_ref2.yaml` are candidates
under iteration, derived from the ref-2-measured palette the user approved.
They live in a **separate** directory from `assets/themes/` on purpose:
`theme.cpp`'s loader has no per-theme code branches, so any directory of
schema-complete YAMLs works via `MPVIZ_SHOWCASE_THEME_DIR` — the shipped
themes stay byte-identical until the user picks a winner and one gets
promoted (copied into `assets/themes/`, at which point it also needs a
schema-conformance pass through `test_theme_parses.cpp`-style checks).

**Flagged deliberate change:** the shipped `dark_adas.yaml`'s hero ribbon is
green (`[0.12, 0.55, 0.42]`) — `dark_adas`/`light_clay` were intentionally
split (dark green / light blue) per that file's own comment. `dark_ref2.yaml`
makes both hero ribbons azure, per the user's "same vibes" direction for a
light/dark pair drawn from one reference. If the user picks this direction,
reverting the *shipped* `dark_adas.yaml` to match is a **one-token change**
(`palette.ribbon_core`/`ribbon_glow` only) — nothing else in that file's
schema is touched by this decision. `dark_adas.yaml` itself is untouched by
this pass.

## Named gap: vegetation

ref-2 shows mint-green park/vegetation patches. There is **no** `vegetation`
palette token in the schema, and none was added in this pass — a token needs
a rendering consumer (park polygons), and it isn't clear the map data this
codebase renders from has that geometry at all. Out of scope here; recorded
so it isn't silently missing from the palette conversation.
