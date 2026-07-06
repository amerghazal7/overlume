# Virtual-Cam GUI + WebSocket Bridge — Design

**Date:** 2026-07-06
**Status:** approved (interactive Q&A)

## Goal

A desktop GUI that orbits the rendering node's virtual camera in realtime,
switches the 5 presets, and shows the live rendered frame — with the
telemetry/control path exposed as a generic WebSocket JSON API so third-party
apps can drive the camera the same way.

## Architecture

Three pieces; the GUI is deliberately just a reference WS client.

```
┌─────────────┐  ~/set_look (Float64MultiArray[6])   ┌──────────────────┐
│ rendering_  │◄─────────────────────────────────────│ vcam_ws_bridge   │◄──ws──┐
│ node (C++)  │  ~/set_virtual_cam (existing srv)    │ (rclpy+websockets│       │
│             │◄─────────────────────────────────────│  :8765, JSON)    │       │
│             │──────────────────────────────────────►                  │  ┌────┴─────┐
│             │  ~/vcam_state (Float64MultiArray[7]) └──────────────────┘  │ vcam_gui │
│             │                                                            │ (GTK3)   │
│             │  /rendering/image ── rosimagesrc ! videoconvert ! gtksink ─►          │
└─────────────┘                                                            └──────────┘
```

### 1. Rendering node additions (C++, small diff)

Reuses the existing `look_at`/`apply_lookpoint` math — same behaviour as the
Python prototype's orbit (`app.py` az/el/dist → eye, fixed look target).

- **Sub `~/set_look`** (`std_msgs/Float64MultiArray`, 6 floats
  `[eye xyz | target xyz]`, rig frame): applies the look-point immediately
  (no tween — orbiting streams continuous poses), cancels any preset tween,
  marks active preset as 0 (free look).
- **Pub `~/vcam_state`** (`std_msgs/Float64MultiArray`, 7 floats
  `[eye xyz | target xyz | active_preset]`, 0 = free look): published every
  render tick, before the frame-sync gate, so telemetry flows even while
  waiting for camera frames.

### 2. WebSocket bridge — `tools/vcam_ws_bridge.py`

rclpy node + `websockets` server (default `0.0.0.0:8765`). Thin and generic:
orbit semantics live in the client; the wire protocol is plain poses.

Client → server:
- `{"cmd":"set_look","eye":[x,y,z],"target":[x,y,z]}` → publishes `~/set_look`
- `{"cmd":"set_preset","preset":1..5}` → calls the existing service

Server → client:
- `{"type":"state","eye":[..],"target":[..],"preset":0..5}` — on change,
  throttled to ~15 Hz
- `{"type":"ack","cmd":"set_preset","success":..,"active":".."}`
- `{"type":"error","message":".."}` on malformed/unknown commands

### 3. GUI — `tools/vcam_gui.py`

GTK3 (PyGObject; everything already installed — zero new dependencies):
- Video area: `rosimagesrc ros-topic=/rendering/image ! videoconvert !
  gtksink` embedded as a native widget.
- Preset buttons 1–5 (config / reverse_follow / left_side / right_side /
  top_down).
- Mouse drag on the video = orbit around the robot origin, exactly the
  prototype's `orbit_shot` geometry: fixed target `[0,0,0.3]`, eye on a
  sphere centred at `z=+0.5`, elevation clamped 5°–85°; scroll wheel = dolly
  distance. az/el/dist persist across preset switches; grabbing the view
  jumps back to the stored orbit pose (the prototype's `[o]`-toggle
  behaviour).
- Streams `set_look` over the WS; status bar shows connection + live pose +
  active preset.
- WS client runs in a background asyncio thread; UI updates via
  `GLib.idle_add`; auto-reconnects.

## Testing

- `smoke_test.py` extension: publish `~/set_look`, assert `~/vcam_state`
  echoes the pose with preset flag 0 (end-to-end through the C++ node).
- `tools/test_vcam_ws_bridge.py`: unit test for the bridge's command
  parsing/validation (pure JSON, no ROS needed at import time).
- GUI verified interactively (needs a display + live node).

## Skipped (add when needed)

- Custom `.msg` types for look/state — `Float64MultiArray` avoids interface
  churn; promote to a proper msg if a third consumer appears.
- Server-side orbit commands (`az/el/dist` on the wire) — clients own orbit
  math; add if a third-party app can't.
- Auth/TLS on the WebSocket — LAN tool; add when exposed beyond localhost.
