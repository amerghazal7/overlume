# Model pack attribution (Epic 2 Task 4 / VM-022)

All models below are CC0 (Creative Commons Zero, public domain — no
attribution legally required; credited here anyway per each pack's own
"support by crediting" note). Normalized with `scripts/normalize_models.py`
(recentered on footprint, +X-forward/+Z-up, unit 1m x 1m footprint) from the
following sources.

## car.glb, truck_van.glb — Kenney Car Kit

- Source: `sedan.glb` (car), `van.glb` (truck_van)
- Pack: Kenney Car Kit, version 3.1, downloaded 2026-08-20
- URL: `https://kenney.nl/media/pages/assets/car-kit/1a312ec241-1775131960/kenney_car-kit.zip`
- License: CC0 1.0 Universal — http://creativecommons.org/publicdomain/zero/1.0/
- Credit: Kenney (www.kenney.nl)

## pedestrian.glb — Kenney Blocky Characters

- Source: `character-a.glb`
- Pack: Kenney Blocky Characters, version 2.0, downloaded 2026-08-20
- URL: `https://kenney.nl/media/pages/assets/blocky-characters/8369c0cf30-1749547469/kenney_blocky-characters_20.zip`
- License: CC0 1.0 Universal — http://creativecommons.org/publicdomain/zero/1.0/
- Credit: Kenney (www.kenney.nl)

## bus.glb, cyclist.glb — NOT SHIPPED

No CC0 bus model was found in Kenney's Car Kit (its truck/van/box-truck
variants cover `truck_van`, but no dedicated bus), and no usable CC0
bike+rider model was found in a quick Kenney/Quaternius pass (both sites'
asset listings are largely JS-rendered, and no direct-download zip could be
located for a bicycle/cyclist pack in the time budget for this task).

Per the plan's Step 0 default: these two stems are simply absent from this
directory. `set_object_model_dir()` treats a missing stem as non-fatal —
`BUS` and `CYCLIST` objects render as the procedural rounded clay box, scaled
to their measured bbox, same as every other unloadable class. This is a
cosmetic gap only; instancing, tinting, arrows, and predicted ribbons are
all fully exercised by the three loaded classes plus the box fallback for
the other three (`BUS`, `CYCLIST`, `UNKNOWN`).
