# Attribution assets

## mapbox-logo.png: the Mapbox logo

- **What:** the logo Mapbox GL JS draws in the corner of a map (a white mark and wordmark with a
  dark outline, on a transparent background), rasterised to 176 x 46 px, twice GL JS's 88 x 23 CSS
  size. RGBA8 PNG, stored in Git LFS through the repository's `*.png` rule (`.gitattributes`).
- **Why it is here:** Mapbox Product Terms 1.4 require the Mapbox logo, "© Mapbox",
  "© OpenStreetMap" and "Improve this map" on any map that shows Mapbox content. While a
  `mapbox://` environment source is streaming, the node draws this file at the bottom left and
  that line at the bottom right (parameters `environment_attribution` and
  `environment_attribution_logo_path`).
- **Source:** the npm package `mapbox-gl` 3.33.0 from the public registry, no token needed:
  `https://registry.npmjs.org/mapbox-gl/-/mapbox-gl-3.33.0.tgz`, npm integrity
  `sha512-2HpIrVedccHedolZyMVG/7FXdpgmsLTENgEDsX71x3/gKDGfYMf1FCRY0prntknkn/JXcraUKXlBTv7vkr0EXg==`.
  The SVG is the percent-encoded `data:` URI of the top-level `a.mapboxgl-ctrl-logo` rule in
  `package/dist/mapbox-gl.css`; the two `@media (-ms-high-contrast: ...)` variants are not used.
- **SHA-256:**
  - the extracted SVG (3012 bytes, not committed):
    `0b72e543243c47f8202f6dc7d4b45062939e85f863df2f8feb08bc90a8e99aa0`
  - `mapbox-logo.png`: `ae67ebad76fb0d383d971455b1ddfab8d564822eb77eb6fc54e5a98e50fa0c31`
- **Rasteriser:** librsvg 2.52.5 with cairo 1.16.0 through PyGObject, as Ubuntu 22.04 ships them
  (`gir1.2-rsvg-2.0`, `python3-gi-cairo`; the same librsvg that `rsvg-convert` from
  `librsvg2-bin` wraps). Other versions can write different PNG bytes; record the new hash.
- **Licence and trademark:** the Mapbox logo is a trademark of Mapbox, Inc. and is not covered by
  this repository's Apache-2.0 licence. `mapbox-gl` 3.x is licensed under the Mapbox Terms of
  Service (`package/LICENSE.txt`). Whether this file may stay in the tree is part of the release
  check (decision D5 of `docs/design/2026-10-14-mapbox-buildings-design.md`): it is settled with
  Mapbox before anything from this work is pushed to the public `origin`.

Regenerate from the repository root (needs network; prints the two hashes above):

```bash
(
set -euo pipefail
for tool in curl openssl tar base64 python3 sha256sum; do
    command -v "$tool" > /dev/null || { echo "missing tool: $tool"; exit 1; }
done
python3 -I -c 'import cairo, gi; gi.require_version("Rsvg", "2.0"); from gi.repository import Rsvg' \
    || { echo "missing librsvg bindings: sudo apt-get install gir1.2-rsvg-2.0 python3-gi-cairo"; exit 1; }
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
curl -sfL -o "$work/mapbox-gl.tgz" https://registry.npmjs.org/mapbox-gl/-/mapbox-gl-3.33.0.tgz
test "sha512-$(openssl dgst -sha512 -binary "$work/mapbox-gl.tgz" | base64 -w0)" = \
    "sha512-2HpIrVedccHedolZyMVG/7FXdpgmsLTENgEDsX71x3/gKDGfYMf1FCRY0prntknkn/JXcraUKXlBTv7vkr0EXg==" \
    || { echo "mapbox-gl-3.33.0.tgz failed its npm integrity check"; exit 1; }
tar -xzf "$work/mapbox-gl.tgz" -C "$work" package/dist/mapbox-gl.css
python3 -I - "$work/package/dist/mapbox-gl.css" "$work/mapbox-logo.svg" <<'PY'
import re
import sys
import urllib.parse

css = open(sys.argv[1], encoding="utf-8").read()
m = re.search(r'a\.mapboxgl-ctrl-logo\{[^}]*?background-image:url\("data:image/svg\+xml;'
              r'charset=utf-8,([^"]+)"\)', css)
head = css[: m.start()] if m else ""
assert m and head.count("{") == head.count("}"), "no top-level a.mapboxgl-ctrl-logo rule"
open(sys.argv[2], "w", encoding="utf-8", newline="").write(urllib.parse.unquote(m.group(1)))
PY
mkdir -p ros/src/overlume_ros/assets/attribution
python3 -I - "$work/mapbox-logo.svg" ros/src/overlume_ros/assets/attribution/mapbox-logo.png <<'PY'
import sys

import cairo
import gi

gi.require_version("Rsvg", "2.0")
from gi.repository import Rsvg  # noqa: E402

handle = Rsvg.Handle.new_from_file(sys.argv[1])
surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 176, 46)
viewport = Rsvg.Rectangle()
viewport.x, viewport.y, viewport.width, viewport.height = 0, 0, 176, 46
assert handle.render_document(cairo.Context(surface), viewport)
surface.write_to_png(sys.argv[2])
PY
sha256sum "$work/mapbox-logo.svg" ros/src/overlume_ros/assets/attribution/mapbox-logo.png
)
```
