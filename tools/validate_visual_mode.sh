#!/usr/bin/env bash
# validate_visual_mode.sh — one command to stand up the full visual-mode
# validation rig (visualization_node + fixture bag + tf flattener + vcam
# bridge/GUI) against the recorded fixture bag, and report PASS/FAIL.
#
# Usage: tools/validate_visual_mode.sh [--bag PATH] [--qos PATH] [--no-gui]
#                                       [--build] [--profile NAME] [--live]
#
# --live: validate against the LIVE autonomy stack instead of the fixture
#         bag -- skips bag playback and the tf flattener (a live stack
#         publishes /tf itself), runs the node on wall time (override with
#         LIVE_SIM_TIME=true when the live source, e.g. CARLA, publishes
#         /clock), and relaxes the ego z==0.0 health check (that asserts
#         the flattener's output, a bag-rig invariant). A FAIL in live mode
#         can also mean "the stack just isn't publishing yet" -- the rig
#         stays up for inspection either way.
#
# BAG MODE vs A RUNNING LIVE STACK (seen live 2026-08-20): if a live sim is
# up on the same ROS domain, its real /tf (z != 0) fights the flattener and
# the bag-mode health gate FAILs on z==0.0 with a nonsense z. Either stop
# the live stack, use --live, or isolate bag mode: ROS_DOMAIN_ID=<n> on
# BOTH this script and anything that needs to see its topics.
#
# ==========================================================================
# MILESTONE UPDATE LOG (Epic 2 — append one line per task as it lands; this
# script is a living deliverable, keep it in sync with what is visually
# checkable after each milestone. User directive 2026-08-20.)
#
#   2026-08-20  Rig created (pre-Task-1). Visually validatable: mode 3,
#               dark_adas ego-following ground+grid (Epic 1), against the
#               recorded fixture bag with TF flattened to z=0 for playback.
#               --profile is accepted but not yet forwarded to anything the
#               node understands (profile.yaml loading is Task 1/VM-020).
#   2026-08-20  Task 1/VM-020: --profile now selects config/<name>_profile.yaml
#               on the node side; a bad/missing name is a fatal on_configure
#               (node stays unconfigured, script reports CONFIGURE failed).
#               Nothing new is visually checkable yet -- no adapter subscribes.
#   2026-08-20  --live added (user directive): same rig + health gate against
#               live topics, no bag, no tf flattener, wall clock by default.
#   2026-08-20  Task 2/VM-024: HD-map lanes/crosswalks are now visible.
#               HdMapAdapter (urban profile: /hd_map_local_elements,
#               /hd_map_global_elements, /road_markers) subscribes per the
#               profile's namespace rules, and the placeholder 40m
#               origin-locked ground+grid now follows the ego (quantized to
#               the 2m grid pitch) instead of leaving it driving over a
#               void. Visually checkable in mode 3: lane paint + crosswalk
#               hatching travel with the ego as the bag plays. Health gate
#               now also samples /hd_map_local_elements' publish rate (the
#               bag's own map feed) as a precondition check -- a silent map
#               topic would otherwise look identical to a silently-broken
#               HdMapAdapter subscription, and this at least rules the
#               former out before anyone goes looking in the latter.
#   2026-09-07  Epic 3 Task 1/VM-036: lane kinds are now visually distinct
#               (CENTERLINE solid yellow-family, LEFT_BOUNDARY/RIGHT_BOUNDARY
#               dashed white -- the inverse of Epic 2's look), crosswalks
#               hatch on every recorded marker (not just synthetic 4-point
#               fixtures), and the road surface between a lane's two
#               boundaries now fills in its own darker `palette.road` tone
#               (both shipped themes) instead of every map element reading
#               as one undifferentiated stroke color. No new topic/param
#               this task -- the health gate's existing /hd_map_local_elements
#               rate check already covers the one input this task's
#               rendering depends on; nothing new to sample.
#   2026-09-08  User directive (post Task 1 candidate review): the road's
#               outer edges (MapKind::ROAD_EDGE, geometry-detected in
#               HdMapAdapter::fill()) now render SOLID yellow-family
#               (palette.road_edge); lane centerlines are HIDDEN BY DEFAULT
#               (urban/sim hd_map rows' centerline_ rule flipped to
#               render: drop) and, when re-enabled via profile YAML, render
#               as faint dot-guidance circles (build_centerline_dots()) in a
#               low-contrast lane-color fade instead of the old solid
#               strip. Visually checkable in mode 3: the two outermost lane
#               lines read as bold solid yellow, interior lane dividers stay
#               dashed white, no centerline strip down the middle of any
#               lane. Same /hd_map_local_elements rate check covers it;
#               nothing new to sample.
#   2026-09-08  Epic 3 Task 2/VM-034: the HD-map layer now FADES on a
#               silenced topic (was: pops) -- closes Epic 2's stated
#               deviation -- and a TF dropout (ego.valid==0) now fades map
#               geometry to invisible instead of leaving it floating,
#               disconnected, over an origin-snapped ground (the Epic 2 gate
#               cosmetic finding). New ~/diagnostics topic
#               (diagnostic_msgs/DiagnosticArray): one status per profile row
#               (per-topic age + dropped_malformed/dropped_stale/
#               dropped_no_tf/dropped_by_rule) plus a node-level render_ms
#               status, published every tick regardless of mode. vcam_gui.py
#               surfaces render_ms + a per-row staleness indicator (relayed
#               through vcam_ws_bridge.py, display-only). Health gate now
#               also samples /visualization_node/diagnostics' publish rate
#               (same lenient liveness threshold as /hd_map_local_elements)
#               -- it only rules out "the diagnostics publisher is silent,"
#               not the render_ms/staleness VALUES (that needs mode 3 active
#               and a human looking at the GUI panel or the topic echo).
#   2026-09-08  User directive (live-render review, 3 items): (1) crosswalk
#               hatch bars now run ALONG the direction of travel and stack
#               ACROSS the crossing width (was: rotated 90 degrees --
#               "horizontal lines instead of vertical"), with a pitch-derived
#               stripe count instead of a fixed 5. (2) path ribbons
#               (BEHAVIOR/GLOBAL/LOCAL) now CLIP to the ego's closest-approach
#               point and render only forward of it, when the ego is within
#               5m of the route -- a route the ego is far from (e.g. the
#               whole GLOBAL destination path) still renders whole. (3)
#               ribbons now draw as a lane FILL with per-role margins instead
#               of a flat stripe: BEHAVIOR (top) is narrowest, LOCAL middle,
#               GLOBAL (bottom) widest, so a lower ribbon's own color peeks
#               out as a rim around whichever is stacked above it. No new
#               topic/param this round -- same /hd_map_local_elements and
#               path-topic rate checks already cover the inputs these fixes
#               depend on; nothing new to sample.
#   2026-09-08  User directive (ribbons-candidate review): hero ribbon glow
#               KILLED in both themes (emissive.ribbon_strength -> 0.0) and
#               dark_adas's hero color is now a cold green (was neon green);
#               default lane-fill margins widened (0.3/0.8/1.3 for
#               GLOBAL/LOCAL/BEHAVIOR) so each stacked ribbon shows a 0.5m
#               rim per side. Theme-only change; nothing new to sample.
#   2026-09-08  Junction-interior cleanup: ROAD_EDGE lines crisscrossing a
#               road junction are now CUT at the junction -- clipped against
#               a MapKind::JUNCTION polygon where the feed has one
#               (/sim/hd_map/markers), and/or trimmed back 2.0 m either side
#               of any two ROAD_EDGE lines' own 2D crossing point everywhere
#               else (covers urban's local/global feed, which has no junction
#               geometry at all). Interior LEFT_/RIGHT_BOUNDARY dashed
#               separators are NEVER cut this way and stay visible through a
#               junction by default -- new profile row key
#               `junction_interior_boundaries: false` drops them there too,
#               on a row with junction polygon data. No new topic/param --
#               same /hd_map_local_elements rate check already covers the
#               one input this depends on. Visually checkable in mode 3:
#               yellow ROAD_EDGE lines no longer run through the middle of a
#               junction box: they stop, the interior reads as dashed white
#               separators only (or clean at the drop-flag), and the outer
#               edges resume past it.
#   2026-09-08  Junction gap-merge: a multi-lane junction crosses one
#               ROAD_EDGE line several times close together, and each
#               crossing's own 2.0 m trim window was independent -- a small
#               real gap between two nearby crossings survived as its own
#               tiny leftover yellow sliver (measured 0.02-6.30 m against the
#               recorded bag). Fix: those windows now MERGE across a gap
#               under kJunctionGapMergeM=6.6 m (below the shortest real road
#               ever observed adjacent to a cut, 7.03 m). Same
#               /hd_map_local_elements input, no new topic/param. Visually
#               checkable in mode 3: junction corners no longer show small
#               isolated yellow fragments between the outer cut and the
#               interior.
#   2026-09-08  Arc-aware cut refinement, round 1: the fixed 2.0 m trim
#               window had no notion of the recorded curb geometry, so it
#               lands at an arbitrary distance from any real corner fillet,
#               not at the fillet's own edge. Fix: each trim window's own
#               boundary now snaps OUTWARD to a real corner arc's own far
#               recorded vertex when one is found nearby on that same edge
#               (radius < 20 m, turn >= 15 deg, candidate located within 6 m
#               of the boundary -- a plain open-pavement crossing with no arc
#               is untouched). Same /hd_map_local_elements input, no new
#               topic/param. Visually checkable in mode 3: yellow ROAD_EDGE
#               lines through a junction corner now stop cleanly where the
#               curb was already curving away, instead of cutting off at an
#               oblique angle mid-curve.
#   2026-09-08  Code-review fix, round 2 (ceiling removal): the 6 m figure
#               above only LOCATES the candidate arc vertex -- it does not
#               bound how far the run it belongs to is reached. A located
#               run is now followed outward, vertex by vertex, to its own
#               true first/last vertex for as long as curvature keeps
#               clearing radius < 20 m. Same /hd_map_local_elements input, no
#               new topic/param. Visually checkable in mode 3: a
#               junction-corner cut now always resumes exactly at the curb's
#               own true corner vertex, never short of it the way the old
#               margin-bounded snap could land.
#   2026-09-08  Redundant arc-tail trim: a promoted edge's own recorded tail,
#               continuing past its own arc's rejoin vertex, sometimes
#               duplicates a DIFFERENT, independently-promoted ROAD_EDGE
#               piece for the rest of its length until the two converge at
#               an exact shared vertex (14 of 19 bag-wide corner instances
#               measured this way, order-independent discriminator). Fix:
#               new `TrimRedundantArcTails` pass trims the tail back to the
#               arc's own rejoin vertex whenever this discriminator fires.
#               Same /hd_map_local_elements input, no new topic/param.
#               Visually checkable in mode 3: a junction corner's arc still
#               renders in full, but the thin duplicate line running
#               alongside the exit road's own edge just past it is gone.
#   2026-09-09  Code-review fix, round 3: the trim above removes a redundant
#               duplicate tail, but a distinct leftover is a DIFFERENT edge:
#               the through road's own straight boundary, sharing the
#               connector's start node, whose fixed-backoff crossing-cut has
#               no notion of where the connector's own corner curves away and
#               so dead-ends past it. Fix: new
#               `SnapWindowsToNeighborArcDepartures` pass, the symmetric
#               counterpart of the round-1 arc-snap -- it snaps a STRAIGHT
#               neighbour's own window boundary back to a shared-node arc's
#               departure vertex, rather than snapping the arc-owning
#               piece's own window outward. Reuses the existing measured
#               node-coincidence and corner-vs-floor gates, no new constant.
#               Same /hd_map_local_elements input, no new topic/param.
#               Visually checkable in mode 3: a through road's yellow
#               boundary at a junction corner now stops exactly where the
#               connector's own curb starts curving away, instead of running
#               a couple more meters into the junction mouth.
#   2026-09-09  Epic 3 Task 3/VM-030: node-side HUD overlay lands -- a speed
#               chip ("<speed> m/s", TfAdapter's own m/s field, no unit
#               conversion) and an active-mode chip ("MODE <1|2|3>"),
#               composited in place onto the RGB8 frame after render_frame()
#               succeeds, colored by the live (possibly mid-theme-transition)
#               theme HUD colors via the new mpviz::get_hud_colors(). New
#               hud_font_path param (default: this checkout's committed
#               assets/fonts/NotoSans-Regular.ttf, OFL-1.1 -- same
#               per-checkout-path deviation VM-044/Epic 5 closes as
#               ego_model_path). Missing/unloadable font is non-fatal (WARN
#               once, frame left untouched, same clay-box-fallback
#               philosophy as set_ego_model). New `hud_enabled` param
#               (default true, STANDING directive's disable knob): false
#               skips CompositeHud() entirely, distinct from the font-path
#               fallback above. No new topic/param the health gate needs to
#               sample -- purely a mode-3 frame composite; nothing new to
#               check besides looking at the frame. Visually checkable in
#               mode 3: top-left corner shows the two chips over whatever's
#               rendered beneath them.
#   2026-09-09  Epic 3 Task 4/VM-031: alert callouts land -- the nearest
#               live obstacle (from the collision-adapter AlertPolygon data
#               already flowing into scene.alerts) gets a leader line + a
#               "<n.n> m" distance chip, drawn through hud_overlay's own
#               font/text/line primitives (extended, this task, with new
#               DrawText()/DrawLine() calls -- no second compositor). New
#               library entry point `project_to_screen()` (scene.h, additive,
#               kSceneVersion unchanged) projects a world point through
#               whatever camera the most recent render_frame() call set.
#               Colored by the same live theme `hud.accent_color` the HUD's
#               own mode chip already uses -- no new theme token needed. New
#               `callouts_enabled` param (default true, STANDING directive's
#               disable knob): false skips the chip builder entirely,
#               distinct from "no obstacle in view this tick" (which also
#               draws nothing, but is a live per-frame condition, not a
#               config switch). No new topic/param the health gate needs to
#               sample -- purely a mode-3 frame composite, same as VM-030's
#               own HUD chips. Visually checkable in mode 3: when the ego is
#               near a recorded collision/alert polygon, a short green line
#               + distance chip appears near it and tracks it as the virtual
#               camera orbits; with no nearby alert, nothing extra is drawn.
#   2026-09-09  Epic 3 Task 5/VM-032: layer visibility + quality-preset
#               plumbing lands. Seven new `layer_objects`/`layer_paths`/
#               `layer_map_elements`/`layer_grids`/`layer_alerts`/
#               `layer_markers`/`layer_point_clouds` bool params (default
#               true, STANDING directive's disable-knob-per-category story),
#               live via a SetParametersCallback -- a change takes effect on
#               the very next timer tick, no restart, unlike every other
#               param this node reads once at on_configure(). Node-side gate
#               only (clears the matching scene_asm_ vector right before
#               point_at() when false); no renderer/scene.h change.
#               `layer_point_clouds` is declared+live but has nothing to
#               gate yet (Task 6/VM-035 scope). `vcam_ws_bridge.py` gained
#               `set_layers {layer: bool, ...}` (one WS message -> one
#               set_parameters call carrying N Parameter entries) and
#               `set_quality <preset>` (writes the `quality` param only --
#               P4 defers the live in-process switch to Epic 5; GUI's
#               quality dropdown says "takes effect on next restart").
#               `create_renderer()`'s quality dispatch also now maps spec
#               §8's shadow-map resolution (2048 high/1024 medium),
#               shadow-disable at low, and low-preset 960x540 internal
#               render scale (Filament DynamicResolutionOptions, pinned
#               min==maxScale). No new topic the health gate needs to
#               sample -- purely param/renderer-config plumbing, same
#               "nothing new to check besides looking at the frame" shape
#               as VM-030/031. Visually checkable in mode 3: toggling a
#               layer via the GUI checklist makes that category's geometry
#               disappear/reappear on the very next frame.
#   2026-09-09  Epic 3 Task 6/VM-035: point clouds land -- `PointCloud[]`
#               appended to SceneGraph (kSceneVersion 1 -> 2), a new
#               PointCloudAdapter (adapter: point_cloud, no shipped-profile
#               row yet -- FIXTURE GAP, zero sensor_msgs/PointCloud2 topics
#               exist in any recording) bakes rgba8 per point node-side
#               (color_mode: auto|rgb|intensity|height|flat, plus
#               max_points/stride decimation), and a new UNLIT point_cloud.mat
#               (packed-rgba vertex color, one settable staleness-fade alpha
#               uniform, ONE MaterialInstance for the whole layer) renders
#               them. `layer_point_clouds` (declared by Task 5) now actually
#               gates `scene_asm_.point_clouds`. No topic the health gate
#               needs to sample yet (no live row ships) -- nothing new to
#               check against the fixture bag until a real PointCloud2
#               topic is profiled in. Visually checkable only via the
#               library/node's own synthetic-scene tests and goldens
#               (test_point_cloud.cpp, test_point_cloud_adapter.cpp), not
#               against this script's rig.
#   2026-09-09  User reports (2): point clouds now render VISIBLY (gl_Point
#               Size from theme point_cloud.point_size_px, default 2px (user-tuned from 4); was
#               1px invisible dust) and the DEFAULT bag is the full sensor
#               recording (lidar /iv_points_fusion rendered via the urban
#               profile's new point_cloud row, best_effort REQUIRED; six
#               camera topics present on replay for rviz/mode-1-2). Bag
#               stored decompressed for instant playback start.
#   2026-09-09  Epic 3 Task 7/VM-037: Epic-0 mux hardening + build hygiene
#               lands -- nothing new is visually checkable in this rig (mux
#               QoS/legacy-topic/initial_mode/vcam_state[8]/build-hygiene
#               fixes are behind-the-scenes: a restarted rendering_node now
#               correctly rejoins the LIVE global mode instead of its own
#               initial_mode default, and the legacy per-node mode switch
#               now correctly hands off from mode 3). GL_RENDERER now
#               logged once per create_renderer() call (stderr) -- look for
#               "[visual_renderer] GL_RENDERER: ..." near this rig's own
#               startup log to confirm real hardware vs. software
#               rasterizer for any render_ms number recorded here.
#   2026-09-09  VM-077 new-stack rendering lands -- output_trajectory_carpet
#               (a per-vertex velocity-colored ribbon, TRIANGLE_LIST from
#               /navigation_motion_obstacle_planner_node/output_trajectory_carpet)
#               is now VISIBLE (new TrajectoryCarpet scene.h category,
#               kSceneVersion 2->3; new trajectory_carpet.mat/adapter; disable
#               knob layer_trajectory_carpet). Collision alerts now come from
#               content-verified successors instead of the dead
#               /navigation_urban_collision_checker_testing_node/* namespace:
#               /behavior_path_planner/collision_markers (role: collision) and
#               /navigation_motion_obstacle_planner_node/collision_markers
#               (role: predicted, flagged judgment call -- see the plan's D2).
#               Two cheap generic-adapter rows added
#               (/navigation/debug_cruise_obstacle_marker, /local_map_corners).
#               Six other new topics explicitly SKIP/deferred with evidence
#               (duplicates or data-inconclusive) -- see the profile YAMLs'
#               own comments and the VM-077 plan's D4 table. Net active urban
#               row delta: 0 (−3 dormant collision, +1 carpet, +2 generic).
#   2026-09-10  User directive: output_trajectory_carpet redirected from a
#               flat translucent (0.7 alpha) TRIANGLE_LIST to a genuine
#               ribbon STACKED into the existing path-ribbon z-order: a
#               centerline (one station per dual-rail quad) extruded at a
#               constant half-width from the new ribbon.margin_velocity_m
#               theme token (soft default 1.05, between LOCAL's 0.8 and
#               BEHAVIOR's 1.3), z-lifted between LOCAL (0.045) and BEHAVIOR
#               (0.05). Per-vertex velocity color stays (the whole point of
#               this element) and is now OPAQUE while fresh, not 0.7
#               translucent. scene.h UNCHANGED (TrajectoryCarpet's existing
#               points field already fit a centerline+color encoding) -- no
#               kSceneVersion bump. layer_trajectory_carpet knob,
#               adapter/category names all unchanged (only their internal
#               meaning: raw wire vertex -> centerline station). Visually
#               checkable: the velocity ribbon now renders as a distinct
#               band nested between LOCAL and BEHAVIOR, correct
#               geometry/z-stack. This redirect implements the user's
#               prescription; the reported flicker itself was never
#               reproduced to a stable measured signature (an early N=24
#               rig comparison, and a follow-up re-measurement pass, both
#               proved too confounded/noisy to trust — see the VM-077 plan's
#               2026-09-10 section) and remains an OPEN QUESTION, not a
#               closed one -- if it's still visible on the real rig, that's
#               a fresh measurement pass to run, not a number already on
#               file here.
#
#   2026-09-10  Epic 4 Task 1/VM-050: geo-anchor lands -- the node now
#               solves a real WGS84<->map-frame anchor from
#               /sim/feedback/gps (50 Hz NavSatFix, PRIMARY) + the
#               map->base_link TF (kMinAnchorSamples=500 AND
#               kMinAnchorBaselineM=20.0 of map-frame displacement -- at
#               this recording's ~0.5 m/s mean speed the 20 m baseline is
#               the binding gate, so expect the anchor log ~40 s into
#               motion, not 10 s), or from the geo_datum_lat_deg/lon_deg/heading_deg
#               param override (all-or-nothing, GPS-denied replays). NOT
#               YET visually checkable on its own -- no rendered element
#               reads the anchor yet (Task 3/VM-052 is the first consumer
#               that draws anything from it); this milestone is the solved
#               anchor + its one-shot RCLCPP_INFO log
#               ("--anchor-lat/--anchor-lon/--anchor-heading-deg ..."),
#               confirmable via `ros2 topic echo /rosout` or the node's own
#               log while this rig is up. scene.h gained one appended POD
#               struct (GeoAnchor) and kSceneVersion 3->4 (sequencing
#               deviation, dated 2026-09-10: performed in Task 1, not
#               Task 3, since Task 1 is GeoAnchor's first consumer) -- no
#               existing struct/enum touched, no rendering behavior change.
#   2026-09-10  User directive (escalation, 4 fix attempts in): "NO THE
#               GREEN LOCAL PATH DISAPPEAR AND APEAR randomly causing the
#               flicker not the road?!" -- REPRODUCE pass found no dropout
#               in the library's single-threaded set_scene()/render_frame()
#               contract (tests/test_ribbon_dropout.cpp, committed as the
#               permanent regression), but did find the real hazard: every
#               ribbon slot (BEHAVIOR/GLOBAL/LOCAL via ribbon.cpp, the
#               velocity ribbon via trajectory_carpet.cpp) destroyed and
#               rebuilt its ENTIRE mesh on every quantized ego-clip tick
#               (kPolylineClipQuantizeM=0.05 -- ~92% of frames while
#               driving), the exact shape a torn cross-thread scene read
#               (scene.h's own documented single-thread-only contract) would
#               turn into a whole-frame dropout. FIX: the ego-clip no longer
#               feeds any ribbon's content signature (ribbon_signature()/
#               trajectory_carpet_signature() -- decision: station term
#               REMOVED from the signature entirely, not just re-quantized);
#               the full unclipped mesh now builds only on real content
#               change (~8 Hz message rate), and the clip is applied EVERY
#               frame as a degenerate-vertex position collapse re-uploaded
#               into the SAME VertexBuffer (polyline.hpp's
#               collapse_clipped_positions(), Filament's own
#               VertexBuffer::setBufferAt) -- no destroy, no new mesh, no
#               entity churn, so no frame can ever observe an absent slot.
#               kPolylineClipQuantizeM (still 0.05) now only gates that
#               per-frame upload (skip when the ego hasn't crossed a new
#               quantized station), not a rebuild. Measured: the driving
#               regression test's rebuild count dropped from ~55/60 frames
#               (pre-fix, clip-driven) to 8/60 (post-fix, message-churn-only
#               -- matches the injected content-change cadence exactly,
#               parked or driving). Stacking/margins/z-order untouched
#               (respine + margins + z as shipped at 33b2747) -- only the
#               rebuild/clip mechanism changed. All 155 library tests green,
#               including every ribbon/carpet golden (pixel-identical --
#               the collapse renders the same picture the old truncate-then-
#               rebuild did). Library rebuilt+installed
#               (libs_build.sh Release) and the node force-relinked
#               (colcon_build.sh micropilot_visualization_node, picked up
#               the new libvisual_renderer.a automatically) -- node's own
#               19/19 tests unaffected (adapter-level, not rendering-level).
#               LIVE evidence (ROS_DOMAIN_ID=93, urban profile,
#               stack_v2_full_sensors_2026-09-09 bag, 24-frame burst ~45s
#               in, crop rows 260-560/cols 440-840 -- docs/evidence/
#               vm077-flicker-2026-09-10/rebuild_clip_fix_burst_after.*):
#               HONEST SPLIT VERDICT, STILL OPEN, not a clean close. The
#               AMBER band (PathRole::LOCAL, palette.ribbon_local) -- what
#               the internal name "LOCAL" literally refers to -- is rock
#               solid across all 24 live frames: the r>b corridor-presence
#               metric never drops (0.1575-0.1625 throughout, per the
#               committed rebuild_clip_fix_burst_after_summary.json),
#               exactly the property this fix targets. BUT the same capture
#               shows the TOPMOST teal/green hero ribbon (PathRole::BEHAVIOR,
#               palette.ribbon_core [0.12,0.55,0.42] -- plausibly what
#               "green" in the user's own quote actually means, since it's
#               the only genuinely green-ish element and the most visually
#               prominent, directly on the ego) COMPLETELY ABSENT
#               (teal_fraction exactly 0.0, not merely faint) on 15 of 24
#               frames, present at ~0.0274 on the other 9 -- a STRUCTURAL
#               per-frame presence failure, confirmed visually (full_0.png
#               has no teal band, full_1.png shows it clearly, consecutive
#               frames). Correction (this pass): an earlier draft of this
#               entry described a different, since-overwritten capture
#               (~0.0022-0.003 on 8 frames vs ~0.029-0.030 on the rest,
#               corridor 0.16-0.2025) -- that capture's raw files were
#               replaced in place by the current one and aren't
#               recoverable, so the two can't be directly reconciled; what's
#               certain is the CURRENT, committed evidence is worse (15
#               frames totally absent, not 8 faintly present). Also fixed:
#               tools/flicker_burst_capture.py's any_teal_dropout flagged
#               this run false (its median-relative check exempted a run
#               where the absent frames are the MAJORITY, since the median
#               collapses to 0.0 right along with them) -- rekeyed off the
#               run's own max instead, committed JSON's flag corrected to
#               true. Mechanically this is NOT the destroy-rebuild path
#               (identical fixed code handles BEHAVIOR too, and BEHAVIOR
#               never swaps instances -- it fades via its own material
#               alpha only, see ribbon.cpp) -- most likely explanation,
#               STILL NOT PROVEN (no node-path instrumentation was run this
#               pass either): BEHAVIOR's row (urban_profile.yaml,
#               /behavior_path_planner/output_path_visualization,
#               timeout_sec=2.0) hits the renderer's OWN staleness fade
#               (kStaleFadeStartSec=0.5/kStaleFadeTimeoutSec=1.0,
#               renderer_internal.hpp) when real message-arrival gaps under
#               bag-replay timing exceed that margin -- the same class of
#               "zero margin against the fade threshold" this plan's own
#               2026-09-10 LOCAL-ribbon section already flagged (item 3,
#               there for /local_vel_path) -- NOT the mechanism this task's
#               FIX REQUIREMENTS targeted. LEFT OPEN, not silently declared
#               fixed: the next pass needs to instrument the BEHAVIOR slot's
#               staleness alpha and the topic's message-arrival timestamps
#               during a live burst to show (or disprove) alpha being
#               driven to 0 by a real message gap as a fade ramp, not the
#               observed binary 0.0274->0.0 toggle. The rebuild-mechanism
#               fix above stands on its own (already closed, shared by all
#               four ribbon categories) -- the user-visible BEHAVIOR-ribbon
#               dropout is NOT closed by it.
#   2026-09-10  Post-review fix: ribbon_emissive.mat's depthCulling:false
#               (added post-hoc to chase the BEHAVIOR staleness-fade drop
#               above, unproven) let the BEHAVIOR hero ribbon paint through
#               the ego body -- visible in the live burst evidence
#               (rebuild_clip_fix_burst_after_full_1.png/.gif) and untested
#               in either direction (no golden renders an ego body under a
#               BEHAVIOR ribbon; RibbonGolden.ThreeRoles_DarkAdas, the only
#               golden stacking all three ribbons, is pixel-identical with
#               the depth test ON, so no z-fight was being lost there
#               either). Reverted depthCulling:false; kept depthWrite:false
#               (harmless, matches Filament's blended default). Files
#               changed: cuda/src/libs/visual_renderer/assets/materials/
#               ribbon_emissive.mat only. Sanctioned reds: none. Golden
#               coverage of ego+BEHAVIOR-ribbon occlusion: none -- open
#               scope, not claimed. Library + node rebuilt, all suites
#               green; live burst evidence re-captured against the fixed
#               build.
#   2026-09-11  Surround-Stitching usability: node launched with bowl_enabled:=true
#               (default false left the stitching toggle a silent no-op -- no camera
#               ingest, no bowl entity); node-side WARN added for that state; M02P ego
#               glb re-provisioned on main (the vm044 worktree's gitignored copy died
#               with the worktree).
#   2026-09-11  GUI mode button pre-cutover: bridge launched --local-mode (set_render_mode
#               drives the merged node's render_mode param; the mux would idle the rig's
#               only publisher until VM-095); node launched hybrid_enabled:=true so
#               mode 2 shows the colorized cloud.
# ==========================================================================
set -euo pipefail
set -m  # each backgrounded job gets its OWN process group (job leader = its
        # own pid), even when this script itself is not a process-group
        # leader (piped from a wrapper, `bash -c`, CI, an agent harness).
        # Without this, all of this script's background jobs inherit
        # whatever pgid the invoking shell happened to have, and teardown()
        # below -- which kills by recorded child pgid -- ends up killing the
        # wrapper and its unrelated siblings instead of just the rig.

# ---------------------------------------------------------------------- args
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# stack_v2_full_sensors (VM-077, 2026-09-09, user: "I can't even see the
# lidar topic in the recorded bag nor cameras when I run the validate
# script?!"): the DEFAULT bag is the full sensor recording — everything the
# lighter stack_v2_fixtures carries PLUS /iv_points_fusion lidar (rendered
# by the urban profile's point_cloud row) and all six raw camera streams
# (published on replay for rviz/mode-1-2 consumers; mode 3 does not render
# them). Stored DECOMPRESSED for instant playback start (the .zstd archive
# sits alongside). Lighter fallbacks on disk: stack_v2_fixtures_2026-09-09
# (no sensors), epic2_fixtures_full (old stack).
BAG="${HOME}/TPSProjector-fixtures/stack_v2_full_sensors_2026-09-09"
QOS="${HOME}/TPSProjector-fixtures/qos_full.yaml"
NO_GUI=0
DO_BUILD=0
PROFILE=""
LIVE=0
BAG_SET=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag) BAG="$2"; BAG_SET=1; shift 2 ;;
        --qos) QOS="$2"; shift 2 ;;
        --no-gui) NO_GUI=1; shift ;;
        --build) DO_BUILD=1; shift ;;
        --profile) PROFILE="$2"; shift 2 ;;
        --live) LIVE=1; shift ;;
        -h|--help)
            grep '^# ' "${BASH_SOURCE[0]}" | head -6 | sed 's/^# //'
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

if [[ "${LIVE}" == "1" && "${BAG_SET}" == "1" ]]; then
    echo "--live and --bag are mutually exclusive (live mode plays no bag)" >&2
    exit 1
fi

LOG_DIR=/tmp/mpviz_validate
mkdir -p "${LOG_DIR}"

# ------------------------------------------------------------- teardown-first
# Kill any prior rig this script (or a previous run of it, or a hand-run
# session) left behind. pgrep -f matches full command lines; a naive
# unanchored pattern like "visualization_node" matches ANY process whose
# argv merely contains that text -- a `tail -f .../visualization_node.cpp`
# bystander, a colcon build's cc1plus compiling visualization_node.cpp, or
# (worst) the shell that is invoking this very script when it's wrapped by
# something that echoes its own command line into argv. Every pattern below
# is anchored to a path/phrase that only the actual rig member's argv
# contains -- the compiled binary is matched by its unique install path
# (not the bare binary name: process `comm` is truncated to 15 bytes by the
# kernel, so `pgrep -x visualization_node` never matches "visualization_n").
# We also still walk pgrep's PID list by hand and skip our own pid and our
# own process group, belt-and-suspenders. This bug bit us for real in the
# 2026-08-19/20 sessions; do not "simplify" back to bare-word pkill -f.
RIG_PATTERNS=(
    "ros2 run micropilot_visualization_node"
    "lib/micropilot_visualization_node/visualization_node"
    "ros2 bag play"
    "tools/tf_flatten_fixture\.py"
    "tools/vcam_ws_bridge\.py"
    "tools/vcam_gui\.py"
)

rig_candidate_pids() {
    local pattern
    for pattern in "${RIG_PATTERNS[@]}"; do
        pgrep -f "${pattern}" 2>/dev/null || true
    done | sort -un
}

kill_prior_rig() {
    local self_pid=$$
    local self_pgid
    self_pgid="$(ps -o pgid= -p "${self_pid}" 2>/dev/null | tr -d ' ')"
    local pid pgid killed=0
    for pid in $(rig_candidate_pids); do
        [[ "${pid}" == "${self_pid}" ]] && continue
        pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        [[ -n "${pgid}" && "${pgid}" == "${self_pgid}" ]] && continue
        kill "${pid}" 2>/dev/null && killed=1 || true
    done
    if [[ "${killed}" == "1" ]]; then
        sleep 1
        # second pass, force, for anything that ignored SIGTERM
        for pid in $(rig_candidate_pids); do
            [[ "${pid}" == "${self_pid}" ]] && continue
            pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
            [[ -n "${pgid}" && "${pgid}" == "${self_pgid}" ]] && continue
            kill -9 "${pid}" 2>/dev/null || true
        done
        echo "[teardown-first] killed leftover rig process(es) from a prior run"
    fi
}
kill_prior_rig

# --live HARD GUARANTEE (user report 2026-09-10: a leftover looping bag from
# a prior bag-mode run survived into a --live session and fought the live
# stack): live mode must not merely SKIP starting a bag -- it must refuse to
# run while any bag player exists, ours or anyone's.
if [[ "${LIVE}" == "1" ]]; then
    for _pass in 1 2; do
        while read -r _pid; do
            [[ -z "${_pid}" || "${_pid}" == "$$" ]] && continue
            kill -9 "${_pid}" 2>/dev/null || true
        done < <(pgrep -f "ros2 bag play" 2>/dev/null || true)
        sleep 0.5
    done
    if pgrep -f "ros2 bag play" >/dev/null 2>&1; then
        echo "[live] FATAL: a 'ros2 bag play' process is still running and could" >&2
        echo "       not be killed -- live mode will not fight a bag. Offender:" >&2
        pgrep -af "ros2 bag play" >&2
        exit 1
    fi
    # Functional check: a /clock publisher in live mode (without
    # LIVE_SIM_TIME=true, where the live source e.g. CARLA legitimately
    # publishes it) means a bag/sim clock somewhere on this domain will
    # fight wall time -- refuse rather than produce the confusing frozen/
    # stale-TF symptoms that fight causes.
    if [[ "${LIVE_SIM_TIME:-false}" != "true" ]]; then
        set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u
        _clock_pubs="$(timeout 5 ros2 topic info /clock 2>/dev/null | sed -n 's/^Publisher count: //p' || true)"
        if [[ -n "${_clock_pubs}" && "${_clock_pubs}" != "0" ]]; then
            echo "[live] FATAL: /clock has ${_clock_pubs} publisher(s) on this ROS domain." >&2
            echo "       Something is playing a bag or publishing sim time. Stop it, or" >&2
            echo "       run with LIVE_SIM_TIME=true if the live source owns /clock." >&2
            exit 1
        fi
    fi
    echo "[live] verified: no bag player, no unexpected /clock publisher"
fi

# ---------------------------------------------------------------- prereqs
if [[ ! -d "${REPO_ROOT}/cuda/install/ros_apps" ]]; then
    if [[ "${DO_BUILD}" == "1" ]]; then
        :  # built below
    else
        echo "cuda/install/ros_apps not found. Run cuda/scripts/ros_apps_build/colcon_build.sh" \
             "or re-run with --build." >&2
        exit 1
    fi
fi

if [[ "${DO_BUILD}" == "1" ]]; then
    echo "[build] running colcon_build.sh ..."
    ( cd "${REPO_ROOT}/cuda/scripts/ros_apps_build" && ./colcon_build.sh )
fi

if [[ "${LIVE}" != "1" ]]; then
    if [[ ! -e "${BAG}" ]]; then
        echo "bag not found: ${BAG}" >&2
        exit 1
    fi
    if [[ ! -e "${QOS}" ]]; then
        echo "qos override file not found: ${QOS}" >&2
        exit 1
    fi
fi

# ------------------------------------------------------------------- ROS env
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/cuda/install/ros_apps/setup.bash"
set -u

# ---------------------------------------------------------------- launch rig
# Track both the backgrounded PID and its process group. `ros2 run`/`ros2 bag
# play` do not always exec-replace themselves -- they can fork a real child
# (verified live: the visualization_node binary ran under a DIFFERENT pid
# than the `ros2 run` job's own $!, reparented to pid 1 once the launcher
# exited). Killing only the recorded $! then leaves that child running. A
# forked child always inherits its parent's process group though, and the
# group id is stable even after the group's leader exits -- so teardown()
# kills by recorded PGID (with the raw PIDs as a belt-and-suspenders
# fallback), not by chasing individual descendant PIDs.
declare -a CHILD_PIDS=()
declare -a CHILD_PGIDS=()
RIG_DOWN=0

track_child() {
    local pid="$1"
    CHILD_PIDS+=("${pid}")
    CHILD_PGIDS+=("$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')")
}

teardown() {
    [[ "${RIG_DOWN}" == "1" ]] && return
    RIG_DOWN=1
    local self_pid=$$ pid pgid
    local -a targets=()
    for pgid in "${CHILD_PGIDS[@]:-}"; do
        [[ -z "${pgid}" ]] && continue
        for pid in $(pgrep -g "${pgid}" 2>/dev/null || true); do
            [[ "${pid}" == "${self_pid}" ]] && continue
            targets+=("${pid}")
        done
    done
    targets+=("${CHILD_PIDS[@]:-}")
    if [[ "${#targets[@]}" -gt 0 ]]; then
        kill "${targets[@]}" 2>/dev/null || true
        sleep 1
        kill -9 "${targets[@]}" 2>/dev/null || true
    fi
    echo "rig down"
}
trap teardown INT TERM EXIT

echo "[launch] visualization_node (log: ${LOG_DIR}/visualization_node.log)"
PROFILE_ARGS=()
if [[ -n "${PROFILE}" ]]; then
    PROFILE_ARGS=(-p "profile:=${PROFILE}")
fi
# --params-file: still passed for the OTHER params it carries (bowl camera
# topics/extrinsics, layer flags, profile, etc). It is no longer why
# ego_model_path/hud_font_path/theme_assets_dir resolve correctly post-VM-044:
# those three now default to "" in default_params.yaml too, and
# on_configure() resolves "" via ament_index to this package's own installed
# share/assets/{ego,fonts,themes} regardless of whether --params-file is
# passed at all. CLI -p overrides still win over the file.
# use_sim_time: the bag publishes /clock (played with --clock), so bag mode
# runs on sim time. A live stack usually does NOT publish /clock -- sim time
# there freezes the node's clock at 0 and breaks staleness gating (seen live
# 2026-08-20) -- so live mode defaults to wall time; set LIVE_SIM_TIME=true
# when the live source (e.g. CARLA) does publish /clock.
USE_SIM_TIME=true
if [[ "${LIVE}" == "1" ]]; then
    USE_SIM_TIME="${LIVE_SIM_TIME:-false}"
fi
# bowl_enabled:=true so Surround Stitching / modes 1-2 are actually usable
# from this rig: the shipped default is false, under which the camera ingest
# never constructs, set_bowl_config() never runs, and the stitching toggle
# is a no-op on a bowl entity that doesn't exist (live finding, 2026-09-11).
# On a camera-less bag this only means the ingest waits on camera_info
# forever -- harmless, and the node WARNs when the toggle is flipped anyway.
ros2 run micropilot_visualization_node visualization_node --ros-args \
    --params-file "$(ros2 pkg prefix micropilot_visualization_node)/share/micropilot_visualization_node/config/default_params.yaml" \
    -p initial_mode:=3 -p use_sim_time:="${USE_SIM_TIME}" -p bowl_enabled:=true -p hybrid_enabled:=true "${PROFILE_ARGS[@]}" \
    > "${LOG_DIR}/visualization_node.log" 2>&1 &
track_child "$!"

echo "[lifecycle] configure + activate (retrying while the node registers)..."
# `ros2 lifecycle set` exits 0 even when the transition CALLBACK fails -- it
# only prints "Transitioning failed" to stdout/stderr and leaves the node in
# its old state. So we grep the actual transition result instead of trusting
# the CLI's exit status. A real "Transitioning failed" is deterministic
# (on_configure/on_activate rejected it) -- retrying won't change that, so
# fail immediately and name the transition + node log. Only a CLI/service
# error (node hasn't registered its lifecycle service yet) is worth retrying.
lifecycle_set_retry() {
    local transition="$1" tries=0 out upper
    upper="$(printf '%s' "${transition}" | tr '[:lower:]' '[:upper:]')"
    while true; do
        out="$(ros2 lifecycle set /visualization_node "${transition}" 2>&1)" || true
        if grep -q "Transitioning successful" <<<"${out}"; then
            return 0
        fi
        if grep -q "Transitioning failed" <<<"${out}"; then
            echo "${upper} failed: on_${transition} callback rejected the transition." >&2
            echo "  see node log: ${LOG_DIR}/visualization_node.log" >&2
            return 1
        fi
        tries=$((tries + 1))
        if [[ "${tries}" -ge 15 ]]; then
            echo "${upper} failed after ${tries} tries (node never became reachable)" >&2
            echo "  see node log: ${LOG_DIR}/visualization_node.log" >&2
            return 1
        fi
        sleep 1
    done
}
lifecycle_set_retry configure
lifecycle_set_retry activate

if [[ "${LIVE}" == "1" ]]; then
    # Live mode: the stack publishes /tf itself (no /tf_raw remap to bridge,
    # and flattening z would be WRONG against real TF), and there is no bag.
    echo "[live] skipping tf_flatten_fixture.py and bag playback -- reading live topics"
else
    echo "[launch] tf_flatten_fixture.py (log: ${LOG_DIR}/tf_flatten.log)"
    python3 "${REPO_ROOT}/tools/tf_flatten_fixture.py" \
        > "${LOG_DIR}/tf_flatten.log" 2>&1 &
    track_child "$!"

    echo "[launch] ros2 bag play --loop (log: ${LOG_DIR}/bag_play.log)"
    # stdin MUST be /dev/null: `set -m` (line 38) puts this job in its own
    # BACKGROUND process group, and rosbag2 with a TTY on stdin enables keyboard
    # controls and reads the terminal -- which SIGTTIN-stops a background group
    # before it prints a single byte. Symptom: 0-byte bag_play.log, no /clock,
    # no ego/map, only when launched from an interactive terminal (2026-08-20).
    ros2 bag play "${BAG}" --loop --clock \
        --qos-profile-overrides-path "${QOS}" --remap /tf:=/tf_raw \
        < /dev/null > "${LOG_DIR}/bag_play.log" 2>&1 &
    track_child "$!"
fi

echo "[launch] vcam_ws_bridge.py (log: ${LOG_DIR}/vcam_ws_bridge.log)"
# --local-mode: this rig runs the merged node alone, so the GUI's mode
# button drives its local render_mode param (VM-093) -- publishing the mux
# mode here would idle the only publisher (pre-VM-095 cutover reality).
python3 "${REPO_ROOT}/tools/vcam_ws_bridge.py" --local-mode \
    > "${LOG_DIR}/vcam_ws_bridge.log" 2>&1 &
track_child "$!"

if [[ "${NO_GUI}" != "1" && -n "${DISPLAY:-}" ]]; then
    echo "[launch] vcam_gui.py (log: ${LOG_DIR}/vcam_gui.log)"
    python3 "${REPO_ROOT}/tools/vcam_gui.py" \
        > "${LOG_DIR}/vcam_gui.log" 2>&1 &
    track_child "$!"
else
    echo "[viewer] GUI skipped (--no-gui or no DISPLAY). To view the stream:"
    echo "  rqt_image_view /rendering/image"
fi

# -------------------------------------------------------------- health gate
if [[ "${LIVE}" == "1" ]]; then
    echo "[health] waiting up to 20s for >=25 Hz on /rendering/image," \
         "valid ego_state, and a live /hd_map_local_elements feed ..."
else
    echo "[health] waiting up to 20s for >=25 Hz on /rendering/image," \
         "valid ego_state with z==0.0, and a live /hd_map_local_elements feed ..."
fi

read_hz() {
    timeout 4 ros2 topic hz /rendering/image 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}

# ego_state is std_msgs/Float64MultiArray: data = [x, y, z, heading, speed, valid]
read_ego_z_valid() {
    local out z valid
    out="$(timeout 3 ros2 topic echo --once /visualization_node/ego_state 2>/dev/null || true)"
    z="$(printf '%s\n' "${out}" | awk '/^data:/{f=1;next} f&&/^- /{n++; if(n==3){print $2; exit}}')"
    valid="$(printf '%s\n' "${out}" | awk '/^data:/{f=1;next} f&&/^- /{n++; if(n==6){print $2; exit}}')"
    printf '%s %s\n' "${z:-}" "${valid:-}"
}

# Task 2/VM-024: this is a precondition check on the BAG's own feed, not on
# HdMapAdapter -- it only rules out "the map topic itself is silent" (bag
# not playing, wrong topic name, QoS mismatch upstream) before anyone goes
# looking for a broken subscription. It intentionally does NOT prove the
# node is rendering lanes (that needs a human looking at the stream, Task 2
# Step 12) -- `ros2 topic hz` counts publishes regardless of who, if
# anyone, is subscribed.
read_hd_map_hz() {
    timeout 4 ros2 topic hz /hd_map_local_elements 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}

# Epic 3 Task 2 (VM-034): same "topic is alive, rate not asserted" precondition
# check as read_hd_map_hz above -- this only rules out "the diagnostics
# publisher itself is silent" (node not configured, wrong topic name) before
# anyone goes looking for a broken BuildDiagnostics()/timer_callback() wiring.
# It intentionally does NOT check render_ms's VALUE (that needs mode 3 active
# and a human/golden looking at the actual number) -- `ros2 topic hz` counts
# publishes regardless of content.
read_diagnostics_hz() {
    timeout 4 ros2 topic hz /visualization_node/diagnostics 2>/dev/null \
        | grep -o "average rate: [0-9.]*" | tail -1 | awk '{print $3}'
}

# Each iteration below runs four sequential probes (read_hz timeout 4,
# read_ego_z_valid timeout 3, read_hd_map_hz timeout 4, read_diagnostics_hz
# timeout 4) -- ~15s/iteration -- so the deadline must clear at least two
# iterations with margin, not one.
DEADLINE=$((SECONDS + 40))
HZ=""
EGO_Z=""
EGO_VALID=""
HD_MAP_HZ=""
DIAG_HZ=""
PASS=0
while [[ "${SECONDS}" -lt "${DEADLINE}" ]]; do
    HZ="$(read_hz || true)"
    read -r EGO_Z EGO_VALID < <(read_ego_z_valid)
    HD_MAP_HZ="$(read_hd_map_hz || true)"
    DIAG_HZ="$(read_diagnostics_hz || true)"
    HZ_OK=0
    if [[ -n "${HZ}" ]] && awk -v h="${HZ}" 'BEGIN{exit !(h>=25)}'; then
        HZ_OK=1
    fi
    EGO_OK=0
    # z==0.0 asserts the tf flattener's output -- a bag-rig invariant. Live
    # TF carries real z, so live mode checks validity only.
    if [[ "${LIVE}" == "1" ]]; then
        [[ "${EGO_VALID}" == "1.0" ]] && EGO_OK=1
    elif [[ "${EGO_VALID}" == "1.0" && "${EGO_Z}" == "0.0" ]]; then
        EGO_OK=1
    fi
    # Lenient threshold (>=1 Hz, not the bag's real ~18 Hz): this is a
    # liveness check, not a rate assertion -- the bag loops and this gate
    # must not flake on a loop-wrap gap.
    HD_MAP_OK=0
    if [[ -n "${HD_MAP_HZ}" ]] && awk -v h="${HD_MAP_HZ}" 'BEGIN{exit !(h>=1)}'; then
        HD_MAP_OK=1
    fi
    # Diagnostics publishes every tick regardless of mode (Step 0's own AC),
    # same lenient >=1 Hz liveness threshold as hd_map above.
    DIAG_OK=0
    if [[ -n "${DIAG_HZ}" ]] && awk -v h="${DIAG_HZ}" 'BEGIN{exit !(h>=1)}'; then
        DIAG_OK=1
    fi
    if [[ "${HZ_OK}" == "1" && "${EGO_OK}" == "1" && "${HD_MAP_OK}" == "1" && "${DIAG_OK}" == "1" ]]; then
        PASS=1
        break
    fi
done

echo "=============================================================="
if [[ "${PASS}" == "1" ]]; then
    echo "PASS  /rendering/image @ ${HZ} Hz  |  ego_state valid=${EGO_VALID} z=${EGO_Z}" \
         " |  /hd_map_local_elements @ ${HD_MAP_HZ} Hz  |  ~/diagnostics @ ${DIAG_HZ} Hz"
else
    echo "FAIL  /rendering/image @ ${HZ:-no-data} Hz  |  ego_state valid=${EGO_VALID:-?} z=${EGO_Z:-?}" \
         " |  /hd_map_local_elements @ ${HD_MAP_HZ:-no-data} Hz  |  ~/diagnostics @ ${DIAG_HZ:-no-data} Hz"
    if [[ "${LIVE}" == "1" ]]; then
        echo "  live mode: a FAIL can also mean the autonomy stack is not" \
             "publishing (yet) -- check TF and /hd_map_local_elements on the stack side."
    fi
    echo "  logs: ${LOG_DIR}/"
fi
echo "=============================================================="

echo "Rig is up (PASS/FAIL above). Ctrl-C to tear down."
set +e
wait
set -e
# PASS is recorded above; a caller (orchestrator/milestone script) chaining
# on this script's exit status must see non-zero on a failed health gate --
# printing "FAIL" and then exiting 0 makes a broken rig indistinguishable
# from a working one. Keeping the rig up for inspection until Ctrl-C is
# still fine; only the final exit code changes.
[[ "${PASS}" == "1" ]] || exit 1
