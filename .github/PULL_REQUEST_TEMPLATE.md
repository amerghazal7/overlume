## What / why

<!-- What changed, and why. Link the issue/task this addresses, if any. -->

## Gate

<!-- Paste the relevant tools/ci_visual_mode.sh output (or the failing/passing
     stage), and note which hosted CI checks (lint/build/docs) ran. -->

- [ ] `tools/ci_visual_mode.sh` run locally and pasted above (requires a GPU/EGL box)
- [ ] Hosted CI (lint/build) green — `docs.yml`'s Pages deploy only runs on push to `main`

## Docs

- [ ] Docs updated alongside the change (or N/A)
- [ ] `docs/status.md` updated, if this changes project status (or N/A)

## Goldens

- [ ] No goldens touched
- [ ] Goldens promoted by a human — which ones, and what was reviewed (per-pixel drift, not just SSIM):
