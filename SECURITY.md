# Security policy

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | Yes |

Overlume is pre-1.0; only the latest `0.1.x` release line receives fixes.
There is no long-term-support branch yet.

## Reporting a vulnerability

Please use GitHub's private vulnerability reporting rather than a public
issue: go to the repository's **Security** tab → **Report a vulnerability**.
(Private vulnerability reporting is enabled for this repository as of
2026-09-17; if the button is missing, the setting was turned off under
Settings → Code security.) This opens a private advisory only the maintainer can see until it's
resolved.

Do not report a suspected vulnerability in a public issue, discussion, or
pull request.

## Token handling

This project integrates with Cesium ion and Mapbox, both gated by an
environment variable:

- `CESIUM_ION_TOKEN`
- `MAPBOX_TOKEN`

Rules that apply everywhere in this repository, including CI:

- Tokens are referenced **by name only** — never printed, logged, echoed, or
  committed, in any script, test, or CI log. Scripts that check a token
  report PASS/FAIL and an HTTP status code, never a token value or a full
  response body.
- No test suite or CI workflow needs a live token; anything that would
  require one skips cleanly instead of failing.
- If a token is ever accidentally exposed (committed, pasted into a log, a
  transcript, an issue, etc.), the remedy is **rotation** — revoke and
  reissue the token at the provider (Cesium ion account settings / Mapbox
  account settings) — not an attempt to scrub git history, which does not
  reliably remove already-exposed secrets from clones, forks, or CI caches
  that may have already fetched them.

## What is *not* a vulnerability

The following are functional or performance concerns, not security issues —
please file them as a regular issue instead:

- A stale or mismatched golden/fixture image (see `CONTRIBUTING.md`'s golden
  promotion convention).
- Rendering artifacts, visual regressions, or theme/lighting differences.
- Performance regressions (frame time, `render_ms`, tile-load latency) —
  see `docs/runbooks/ci_gate.md`'s benchmark section.
- Missing GPU/EGL on a given box causing gated tests to fail (documented,
  expected behavior — see `docs/runbooks/ci_gate.md`).
