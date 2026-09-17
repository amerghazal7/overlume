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
(`cmake --build overlume/build --target test_theme_showcase`):

```bash
cd overlume/build

# A shipped theme (assets/themes/*.yaml) -- OVERLUME_SHOWCASE_THEME_DIR defaults
# to the shipped dir, so it can be omitted here:
OVERLUME_SHOWCASE=1 OVERLUME_SHOWCASE_THEME=dark_adas \
OVERLUME_SHOWCASE_OUT=/tmp/showcase_dark_adas.png \
    ./test_theme_showcase --gtest_filter='ThemeShowcase.Capture'

# A future candidate variant, kept in a scratch directory of your own
# (there is no assets/theme_variants/ in the repo right now -- the ref-2
# candidates that lived there were promoted into assets/themes/ on
# 2026-09-16 and the directory was deleted so the promoted content
# couldn't silently drift from a leftover copy). OVERLUME_SHOWCASE_THEME_DIR
# still works for this the same way -- point it at wherever you keep the
# next round of candidate YAMLs:
OVERLUME_SHOWCASE=1 OVERLUME_SHOWCASE_THEME=my_candidate \
OVERLUME_SHOWCASE_THEME_DIR=/path/to/your/candidate/dir \
OVERLUME_SHOWCASE_OUT=/tmp/showcase_my_candidate.png \
    ./test_theme_showcase --gtest_filter='ThemeShowcase.Capture'
```

Iterating on a candidate: edit the variant's YAML in place, re-run the same
command (no rebuild) — `theme.cpp`'s loader reads it fresh every run. stderr
logs `mean_luminance`/`non_background_fraction` for the render as a sanity
check that the capture actually produced content.

Env contract (also documented in the test file's own header comment):

| Var | Meaning | Default |
|---|---|---|
| `OVERLUME_SHOWCASE` | `1` enables the capture; unset -> `GTEST_SKIP()` | unset |
| `OVERLUME_SHOWCASE_THEME` | theme name (yaml stem) to load | `dark_adas` |
| `OVERLUME_SHOWCASE_THEME_DIR` | directory to load `<theme>.yaml` from | shipped `assets/themes/` |
| `OVERLUME_SHOWCASE_OUT` | output PNG path | `/tmp/theme_showcase_<theme>.png` |

## What the scene contains

One 1280×960 frame, elevated 3/4 chase-cam pose (behind/above the ego,
looking forward-and-across along the road toward the buildings ahead —
echoing `docs/assets/visualization-reference-2.jpg`'s own framing), containing:

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
- **All three ribbon roles** (`BEHAVIOR`/`GLOBAL`/`LOCAL`) — hand-built,
  NOT `golden.hpp`'s `make_three_role_ribbons()` (round-2 gate finding,
  blocking): that helper stacks all three roles into one shared corridor
  with the ego sitting exactly on it, so the ego-proximity clip
  (`polyline.cpp`'s `compute_polyline_clip()`, which runs per-ribbon)
  collapsed BEHAVIOR — the shortest, narrowest role — down to a stub
  sitting entirely under the ego's own clay box, 0 visible pixels.
  This test instead lays the three roles out as separate lateral lanes,
  each starting just behind the ego and running well past it, so every
  role keeps a long, unoccluded run to judge.
- **The ground grid** — reused verbatim from `make_two_layer_grids()` (two
  OGM layers, dynamic + gradient).

Every reused helper's own coordinates are defined relative to a local
`(0,0,0)` origin; this test translates all of them by one shared offset so
they land together with the (un-movable, already baked-in-map-frame)
buildings in a single frame — see the test file's own `kSceneOrigin` comment.

## The ref-2 candidates: promoted 2026-09-16

The `light_ref2`/`dark_ref2` candidates this harness was built to judge are
no longer candidates — the user approved both by eye and they are now the
shipped `assets/themes/light_clay.yaml` and `dark_adas.yaml` themselves
(this pass also re-shot every golden the re-palette legitimately changed and
fixed the handful of theme-test guards it tripped; see the promotion
commit). `assets/theme_variants/` was deleted with the promotion — the
showcase now renders the shipped themes directly (the `OVERLUME_SHOWCASE_THEME_DIR`
example above still applies to whatever directory holds the *next* round of
candidates).

One decision from that round is worth restating since it's easy to miss by
reading the YAML alone: an earlier revision of this same re-palette made
BOTH hero ribbons azure, on the reading that "same vibes" meant one shared
accent across light/dark. The user looked at it rendered and decided
otherwise — `dark_adas.yaml` keeps its green hero
(`palette.ribbon_core`/`ribbon_glow` = `[0.12, 0.55, 0.42]`,
`hud.accent_color` tracks it), `light_clay.yaml` keeps its blue. Don't
unify these two without seeing that decision again.

## Named gap: object_tint judging at opacity 0.25

`objects: { opacity: 0.25 }` is the shipped repo-wide default (VM-078,
unchanged by this pass) — every `TrackedObject` renders at 25% strength
against its background, so the six per-class `object_tints` are also
diluted to ~25% strength in this capture. A sentinel render (each tint set
to a pure primary) confirms hue carries through the render path correctly;
the dilution is a real, expected side effect of the shipped opacity value,
not a bug in this harness. If a future palette review needs to judge
`object_tints` at full strength, capture a second frame with
`OVERLUME_SHOWCASE_THEME_DIR` pointed at a scratch copy of the candidate theme
with `objects.opacity` set to `1.0` — do **not** change the shipped/
candidate YAML's own opacity value for this.

## Named gap: vegetation

ref-2 shows mint-green park/vegetation patches. There is **no** `vegetation`
palette token in the schema, and none was added in this pass — a token needs
a rendering consumer (park polygons), and it isn't clear the map data this
codebase renders from has that geometry at all. Out of scope here; recorded
so it isn't silently missing from the palette conversation.
