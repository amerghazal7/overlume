// renderer_internal.hpp — internal-only, `-I src` visibility, not installed,
// not POD (Epic 1 Task 2 Step 7e). The `VisualRenderer` class definition,
// plus the `Mesh`/`Vertex`/`add_mesh` helper types/functions it needs by
// value or by call, extracted out of `renderer.cpp`'s former anonymous
// namespace so `src/ego.cpp` (Task 4, a separate translation unit compiled
// as part of the same `visual_renderer` library target, and so sharing that
// target's PRIVATE Filament include access) can see `class VisualRenderer`
// and reuse `add_mesh` for the ego's clay-box fallback.
//
// Pulls in <filament/...> headers, so it is deliberately NOT included by any
// tests/*.cpp — see Task 4's ego_test_hooks.hpp (Filament-free) for what
// tests use instead. Test binaries are only given `-I src` for headers like
// scene_buffer.hpp that don't reach into Filament; they are never granted
// visual_renderer's own PRIVATE Filament include dir, so a test TU
// including this header would fail to compile.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/SwapChain.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>

// Ego (Epic 1 Task 4 / VM-012): gltfio's asset/loader/resource types, only
// needed for the handful of VisualRenderer member pointers below (the
// class body they populate stays in ego.cpp, a separate translation unit —
// see that file). Sourced from the same Filament::filament PRIVATE include
// dir every other <filament/...> header above already comes from (Task 2
// Step 3's GetFilament.cmake glob sweeps gltfio's headers/archives in too,
// no separate CMake target needed).
#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>

#include "scene_buffer.hpp"
#include "theme.hpp"
#include "theme_transition.hpp"

namespace mpviz {

// Full class body stays defined in renderer.cpp (no longer inside
// `namespace { ... }`, but still plain `namespace mpviz` scope there) —
// forward-declared here so VisualRenderer::platform (a bare pointer member)
// can name the type without this header needing the EGL-touching definition
// itself.
class HeadlessEglPlatform;

// One interleaved vertex: world-space position + the Filament "TANGENTS"
// quaternion that encodes the surface normal (see VertexBuffer::Builder::
// attribute()'s warning: TANGENTS is how normals are specified).
struct Vertex {
    filament::math::float3 position;
    filament::math::float4 tangentFrame;
};

filament::VertexBuffer* make_vertex_buffer(filament::Engine& engine, std::vector<Vertex> verts);
filament::IndexBuffer* make_index_buffer(filament::Engine& engine, std::vector<uint16_t> indices);

struct Mesh {
    filament::VertexBuffer* vb = nullptr;
    filament::IndexBuffer* ib = nullptr;
    utils::Entity entity;
    // Epic 3 Task 1 (VM-036) user directive 2026-09-08: the vertex count
    // add_mesh() built this with -- Filament's VertexBuffer exposes no
    // getter for it, so this mirrors what was passed in, the same "no
    // getter, so mirror the value at the call site" reasoning as every
    // *MaterialBaseColor field below. Set for every mesh add_mesh() builds
    // (ground/grid/lane/ribbon/ego/etc, not map-elements-specific), read by
    // map_element_total_vertex_count() (map_elements_test_hooks.hpp) to
    // distinguish a dot-disc-built CENTERLINE mesh from a strip-built one
    // without a full-frame SSIM.
    uint32_t vertexCount = 0;
    // Epic 3 Task 2 (VM-034): staleness fade per-entity MaterialInstance
    // swap, same mechanism/fields as ObjectEntity::fadeInstance/fadeAlpha
    // and AlertSlot::fadeInstance/fadeAlpha below — null while fresh (bound
    // to the shared opaque per-kind template), a clay_translucent.mat
    // instance while fading. Lives on Mesh itself (not a parallel map)
    // because map_elements.cpp's mapElementMeshes cache already IS the
    // per-content-signature persistent record every fading mesh needs; other
    // Mesh users (ground/grid/ribbon) simply never populate these fields.
    filament::MaterialInstance* fadeInstance = nullptr;
    float fadeAlpha = 1.0f;
};

// Epic 2 Task 2 (VM-024): the ego-following ground/grid patch, per-element
// map geometry (map_elements.cpp) is measured in whole METERS, not the
// half-metre a naive "just use the grid spacing constant" read would
// suggest — build_grid_lines()'s own `step` IS this same symbol (renderer.cpp),
// so the pitch the ground snaps to and the pitch the grid lines are drawn
// at can never drift apart. See the plan's "Grid-fade interaction" note for
// why quantizing to anything other than the real line pitch reintroduces
// the crawl this exists to prevent.
inline constexpr float kGridPitchM = 2.0f;

// Staleness fade (Epic 2 Task 4 / VM-022, shared by every stale-able
// category the epic adds -- objects is first to use these; later tasks
// reuse the same two constants rather than re-deriving them). spec §5's
// "~0.5s fade, never pop": SceneBuffer::staleness_alpha(now, last_update,
// kStaleFadeStartSec, kStaleFadeTimeoutSec) is the ONE ramp every category
// calls -- no per-category alpha math anywhere.
inline constexpr double kStaleFadeStartSec = 0.5;
inline constexpr double kStaleFadeTimeoutSec = 1.0;

// Alert severity constant alphas (Epic 2 Task 7 / VM-026 plan, Task 7 Step
// 4: "constant alphas -- info GHOSTED low (~0.18), warning ~0.35, critical
// ~0.45"). This is NOT a fade -- the ghost alpha is a property of the INFO
// severity ITSELF (AlertPolygon is frozen with no `role`, so "the ego
// sweep gets a lower constant alpha" is only expressible as "severity 0
// does" -- see the plan's "…and the material that can actually do it").
// Shared by alert_polygons.cpp (which MULTIPLIES staleness_alpha into this
// per-entity, never past it) and renderer.cpp's push_theme_to_scene()
// (which pushes it into the three eager per-severity TEMPLATE instances
// every call, alongside their palette.alert.* rgb -- alpha is not a theme
// field, this array is the whole reason it doesn't need to be).
inline constexpr float kAlertSeverityAlpha[3] = {0.18f, 0.35f, 0.45f};  // info, warning, critical

// Builds per-vertex orientation quaternions from flat-shaded normals
// (SurfaceOrientation's "normals only" mode) — shared by renderer.cpp's
// ground/grid builders and map_elements.cpp's lane/crosswalk builders (both
// bake a flat +Z tangent frame onto in-plane geometry). Declared here
// (Step 7e's promotion pattern) so a second translation unit can call it;
// defined once in renderer.cpp.
void fill_tangent_frames(std::vector<Vertex>& verts, const std::vector<filament::math::float3>& normals);
// One flat unit arrow (shaft+head, +X, tail x=0, tip x=1) shared by
// objects.cpp's velocity arrows AND generic_markers.cpp's ARROW primitive --
// promoted here (defined once in renderer.cpp, same pattern as
// fill_tangent_frames) when the marker fallback became a second caller and
// a verbatim copy shipped (review 2026-08-20). Both call sites fill
// r.sharedArrowMesh; there is exactly one GPU copy of this geometry.
void build_unit_arrow(std::vector<Vertex>& verts, std::vector<uint16_t>& indices);

// Tears down one Mesh's GPU resources and removes its entity from the
// scene. Promoted out of renderer.cpp's anonymous namespace (Task 2 /
// VM-024): map_elements.cpp's diff-based cache eviction needs the exact
// same teardown renderer.cpp's own destroy_renderer() uses, and duplicating
// it would be the second place that has to remember every step (scene
// removal, entity destruction, vb/ib destruction) in the same order.
void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh);

// Epic 2 Task 4 (VM-022): lazily builds the SHARED gltfio AssetLoader +
// ubershader MaterialProvider + ResourceLoader on `r`, the first time
// EITHER set_ego_model() (ego.cpp) or set_object_model_dir() (objects.cpp)
// needs one -- hoisted out of ego.cpp's former per-call machinery so a
// second AssetLoader never exists in this library (a blocking duplication
// finding per the plan). No-op (returns true) if already built. false only
// if MaterialProvider/AssetLoader construction itself fails.
bool ensure_gltf_loader(VisualRenderer& r);

// Epic 2 Task 4 (VM-022): one instanced glTF class's pool. `asset` is the
// PRIMARY asset (owns every instance below -- destroying it destroys them
// all, gltfio's own documented behavior); `pool` is every FilamentInstance*
// ever created for this class (the initial createInstancedAsset() batch
// plus any createInstance() growth past it); `freeList` is the subset NOT
// currently bound to a live TrackedObject (removed from the scene,
// recycled -- there is no destroyInstance() in gltfio, see objects.cpp).
// `unitFootprint` is this class's OWN normalized bounding-box extent
// (X length / Y width / Z height) read back once via
// FilamentAsset::getBoundingBox() -- the divisor in
// `scale = TrackedObject::dimensions / unitFootprint`.
struct ObjectClassPool {
    filament::gltfio::FilamentAsset* asset = nullptr;
    std::vector<filament::gltfio::FilamentInstance*> pool;
    std::vector<filament::gltfio::FilamentInstance*> freeList;
    Vec3 unitFootprint{1.0, 1.0, 1.0};
    bool growthLogged = false;  // DEBUG-log growth past kInitialInstancesPerClass once
    bool capWarned = false;     // WARN once when kMaxInstancesPerClass is hit
};
inline constexpr size_t kInitialInstancesPerClass = 16;
// ponytail: 256/class, recycled; a real LOD/cull budget is Epic 5.
inline constexpr size_t kMaxInstancesPerClass = 256;

// Epic 2 Task 4 (VM-022): one live per-track renderable, diffed every
// render_frame() call by update_objects() (acquire/update/release, never
// rebuilt wholesale). Exactly one of `glInstance` (a class pool's
// FilamentInstance) or `proceduralBox` (the UNKNOWN/missing-model/
// pool-exhausted fallback, ALWAYS built as a unit 1x1x1 box so its own
// TransformManager scale is `dims / (1,1,1)` -- identical math to the
// glTF path, no separate "box is baked at dims" special case) is populated.
struct ObjectEntity {
    ObjectClass cls = ObjectClass::UNKNOWN;
    filament::gltfio::FilamentInstance* glInstance = nullptr;
    Mesh proceduralBox;
    utils::Entity transformRoot;
    utils::Entity arrowEntity;  // velocity arrow; null iff velocity == {0,0,0}
    Mesh pathRibbon;            // predicted-path ribbon; empty iff no path data
    uint64_t pathSignature = 0; // content signature (see objects.cpp), rebuild-skip guard
    Vec3 appliedScale{0.0, 0.0, 0.0};  // test hook: DimensionsDriveScale (renderer_internal.hpp's
                                       // egoFallbackDims precedent -- NEVER the RenderableManager AABB)
    // Staleness fade (see renderer.cpp's push_theme_to_scene() comment and
    // the plan's "…and the material that can actually do it"): null while
    // fresh (bound to the shared opaque objectClassMaterial[cls] template).
    // Non-null while fading -- a per-entity clay_translucent.mat instance
    // created from VisualRenderer::clayTranslucentMaterial, NEVER a
    // MaterialInstance::duplicate() of the opaque template (still clay.mat:
    // opaque, float3 baseColor). fadeAlpha mirrors what was last pushed into
    // it (MaterialInstance has no getter).
    filament::MaterialInstance* fadeInstance = nullptr;
    float fadeAlpha = 1.0f;
};

// release_object_entity()/update_objects() are declared in objects.hpp
// (this epic's own per-frame counterpart to map_elements.hpp's
// update_map_elements()) -- ObjectEntity/ObjectClassPool live here because
// VisualRenderer's member fields need the complete types, but the
// functions that operate on them belong with the rest of Task 4's logic.

class VisualRenderer {
public:
    HeadlessEglPlatform* platform = nullptr;
    filament::Engine* engine = nullptr;
    filament::SwapChain* swapChain = nullptr;
    filament::Renderer* renderer = nullptr;
    filament::Scene* scene = nullptr;
    filament::View* view = nullptr;
    filament::Camera* camera = nullptr;
    utils::Entity cameraEntity;
    utils::Entity sunEntity;
    filament::IndirectLight* ambient = nullptr;
    filament::ColorGrading* colorGrading = nullptr;
    // clay.mat (shared: ground here, ego clay-box fallback + glTF remap in
    // Task 4) and clay_faded.mat (grid-only, per-vertex alpha) — replace
    // Epic 0's unlit simple_color.mat (Task 2 / VM-011).
    filament::Material* clayMaterial = nullptr;
    filament::Material* clayFadedMaterial = nullptr;
    filament::MaterialInstance* groundMaterial = nullptr;
    filament::MaterialInstance* gridMaterial = nullptr;
    // Ego contrast color (user directive 2026-08-20): a FOURTH clay.mat
    // instance, dedicated to the ego, tinted with its own cross-theme
    // palette.ego token instead of reusing groundMaterial (which would make
    // the ego read as the same color as the ground it stands on — the bug
    // this exists to fix). Created EAGERLY in create_renderer(), same
    // reasoning as laneMaterial just below: push_theme_to_scene() only runs
    // at create_renderer() and during set_theme() transitions, so a lazily-
    // created instance (e.g. only when set_ego_model() first runs) would
    // render with clay.mat's compiled-in defaults until the next theme
    // push. egoMaterialBaseColor mirrors it the same way laneMaterialBaseColor
    // does (Filament's MaterialInstance has no getter) — ego_test_hooks.hpp's
    // regression test reads this.
    filament::MaterialInstance* egoMaterial = nullptr;
    detail::Float3 egoMaterialBaseColor{};
    Mesh ground;
    Mesh grid;
    uint32_t width = 0;
    uint32_t height = 0;

    // Map elements (Epic 2 Task 2 / VM-024). laneMaterial is a clay.mat
    // instance (same material as ground/grid, tinted differently) created
    // EAGERLY in create_renderer() and registered in push_theme_to_scene()
    // — see that function's own comment for why "created lazily on first
    // map data" is the bug this avoids. laneMaterialBaseColor mirrors
    // exactly the value last pushed to laneMaterial's "baseColor" parameter
    // (Filament's MaterialInstance has no getter) — the map_elements_test_
    // hooks.hpp introspection Step 8a's regression test reads, so it
    // proves the actual setParameter call happened, not merely that
    // active_theme holds the expected value. mapElementMeshes is the diff-
    // cache update_map_elements() (map_elements.cpp) keys by a content
    // signature (point count + first/last point, not array position — the
    // source topic is a rolling window, so position is not stable) so a
    // rebuild only touches elements that actually changed since the last
    // publish (spec §5's "cached, no per-frame rebuild" AC).
    filament::MaterialInstance* laneMaterial = nullptr;
    detail::Float3 laneMaterialBaseColor{};
    // Per-kind tint instances (Epic 3 Task 1 / VM-036, decision #6) -- same
    // clay.mat template, same eager-creation/push_theme_to_scene reasoning
    // as laneMaterial above, one more instance per new token. laneMaterial
    // itself stays the fallback for STOPLINE/JUNCTION/OTHER (the kinds with
    // no dedicated token, per decision #6's soft-default note). ROAD_EDGE
    // got its own dedicated roadEdgeMaterial below (user directive
    // 2026-09-08) -- it no longer falls back to laneMaterial.
    filament::MaterialInstance* laneCenterlineMaterial = nullptr;
    detail::Float3 laneCenterlineMaterialBaseColor{};
    filament::MaterialInstance* laneBoundaryMaterial = nullptr;
    detail::Float3 laneBoundaryMaterialBaseColor{};
    filament::MaterialInstance* crosswalkMaterial = nullptr;
    detail::Float3 crosswalkMaterialBaseColor{};
    filament::MaterialInstance* roadMaterial = nullptr;
    detail::Float3 roadMaterialBaseColor{};
    // ROAD_EDGE (user directive 2026-09-08): the road's outer boundary gets
    // its own clay.mat instance, same eager-creation/push_theme_to_scene
    // reasoning as every other per-kind tint above -- solid yellow-family,
    // never dashed (map_elements.cpp gates dashing on LEFT_BOUNDARY/
    // RIGHT_BOUNDARY only, which ROAD_EDGE is not).
    filament::MaterialInstance* roadEdgeMaterial = nullptr;
    detail::Float3 roadEdgeMaterialBaseColor{};
    std::unordered_map<uint64_t, Mesh> mapElementMeshes;
    // Epic 3 Task 1 (VM-036) Step 7: Epic 2's untested "cached, no
    // per-frame rebuild" AC, finally checked. Incremented once per
    // adopt_or_build() CACHE-MISS branch in update_map_elements()
    // (map_elements.cpp) -- a content signature already present in
    // mapElementMeshes is adopted, not rebuilt, and does not bump this.
    uint64_t mapElementRebuildCount = 0;

    // Theme system (Task 2 Step 7/7b). theme_dir is the retained copy of
    // RenderConfig::theme_assets_dir (borrow-then-copy rule, see api.h) —
    // kept around for Task 3's set_theme() to re-load from later, even
    // though nothing in this task re-reads it yet. active_theme is
    // whatever create_renderer() resolved at startup (loaded file or
    // kFallbackTheme()); Task 3 is what makes it change after creation.
    std::string theme_dir;
    detail::Theme active_theme;
    // Gate-review addition (2026-08-20, spec §9 minor): true iff the theme
    // active right after create_renderer() actually came from disk
    // (load_theme() succeeded), false if create_renderer() had to substitute
    // detail::kFallbackTheme(). Set once at create_renderer() time, read
    // back by the theme_assets_loaded() free function in scene.h.
    bool theme_assets_loaded = false;

    // Epic 1 Task 3 (VM-014): set() when set_theme() is called, cleared
    // (nullopt) once the eased blend reaches t>=1.0 -- render_frame()'s
    // apply_current_theme() is then a no-op every subsequent call (steady
    // state), not a recomputation of an already-finished blend every frame.
    std::optional<detail::ThemeTransition> theme_transition;

    // Task 1's double-buffered scene ingest (VM-010). set_scene() (Task 2
    // Step 7) is exactly `r->scene_buffer.publish(scene);` — no Filament
    // call happens there; render_frame()/future ego-transform code reads
    // scene_buffer.active() instead.
    detail::SceneBuffer scene_buffer;

    // Ego (Epic 1 Task 4 / VM-012), built/loaded ONCE by set_ego_model()
    // (ego.cpp) and never rebuilt/reloaded afterward (Step 5's "No
    // reloading either way"). Exactly one of `egoAsset` (gltfio path) or
    // `egoFallback.entity` (clay-box path) ends up populated per call;
    // `egoTransformEntity` is whichever one render_frame()'s per-frame
    // update_ego_transform() (ego.cpp/ego.hpp) actually drives — the glTF
    // asset's transform root (already has a TransformManager component,
    // built by gltfio itself) or the fallback box's own mesh entity (given
    // one explicitly, since add_mesh()-built entities don't get a
    // TransformManager component automatically — Epic 1 Task 2's ground/
    // grid never moved, so that epic never needed one; Epic 2 Task 2 /
    // VM-024 is what gives ground.entity/grid.entity their own explicit
    // TransformManager component, for the ego-following patch below).
    // Epic 2 Task 4 (VM-022): hoisted out of being ego-specific -- objects.cpp
    // needs the exact same AssetLoader/MaterialProvider/ResourceLoader, and a
    // second one in this library is a blocking duplication finding (the
    // plan's own words). Lazily built by ensure_gltf_loader() the first time
    // EITHER set_ego_model() or set_object_model_dir() runs; shared, and torn
    // down exactly once in destroy_renderer() (AFTER both egoAsset and every
    // ObjectClassPool::asset below have been destroyed through it).
    filament::gltfio::MaterialProvider* sharedMaterialProvider = nullptr;
    filament::gltfio::AssetLoader* sharedAssetLoader = nullptr;
    filament::gltfio::ResourceLoader* sharedResourceLoader = nullptr;
    filament::gltfio::FilamentAsset* egoAsset = nullptr;
    Mesh egoFallback;
    utils::Entity egoTransformEntity;
    // The actual dims build_ego_box() used for egoFallback (Task 4 review
    // round 8): NOT the same as querying egoFallback's RenderableManager
    // AABB, which reports add_mesh()'s hard-coded declared culling box
    // (renderer.cpp's kGroundHalfExtent), unrelated to the ego's real
    // geometry. {0,0,0} until build_ego_fallback() runs.
    Vec3 egoFallbackDims{0.0, 0.0, 0.0};

    // Objects (Epic 2 Task 4 / VM-022). clay_translucent.mat (the epic's one
    // settable-alpha Material, built once here from
    // clay_translucent_filamat.h) and its per-class opaque clay.mat
    // TEMPLATE instances (indexed by static_cast<uint8_t>(ObjectClass); the
    // 6th slot is UNKNOWN) -- created EAGERLY in create_renderer() and
    // registered in push_theme_to_scene(), same "lazy creation is a known
    // trap" reasoning as laneMaterial/egoMaterial. objectClassTint mirrors
    // each template's last-pushed baseColor (MaterialInstance has no
    // getter) -- the staleness fade needs that color back (see
    // renderer.cpp's push_theme_to_scene() comment).
    static constexpr size_t kObjectClassCount = 6;  // CAR..UNKNOWN
    filament::Material* clayTranslucentMaterial = nullptr;
    filament::MaterialInstance* objectClassMaterial[kObjectClassCount] = {};
    detail::Float3 objectClassTint[kObjectClassCount] = {};
    // WARN-once flags (spec §9): missing/unloadable model per class, and
    // kMaxInstancesPerClass exhaustion per class -- both fall back to the
    // procedural box, neither is fatal, neither should spam per-object.
    bool objectClassMissingWarned[kObjectClassCount] = {};

    // Per-class instanced glTF pools (Task 4 Step 3), keyed by
    // static_cast<uint8_t>(ObjectClass). Absent key == no model ever loaded
    // for that class (Step 0's default, or that stem's file was missing/
    // unparseable) -- always the procedural box path, non-fatal.
    std::unordered_map<uint8_t, ObjectClassPool> objectClassPools;

    // Shared unit-arrow geometry (velocity arrows): built lazily the first
    // time any object actually has a nonzero velocity (objects.cpp), reused
    // by every object's own arrowEntity via RenderableManager -- "one
    // shared unit-arrow mesh scaled/rotated per object" (the plan). `entity`
    // is unused (this Mesh is a geometry template, never itself added to
    // the scene).
    Mesh sharedArrowMesh;

    // Live per-track entities (Task 4 Step 3), diffed every render_frame()
    // call by update_objects() (objects.cpp) -- keyed by TrackedObject::id.
    std::unordered_map<uint32_t, ObjectEntity> objectEntities;

    // Path ribbons (Epic 2 Task 5 / VM-023). ribbonEmissiveMaterial is the
    // BEHAVIOR role's dedicated Material (ribbon_emissive.mat, built once
    // from ribbon_emissive_filamat.h); ribbonMaterial[role] are the THREE
    // per-role opaque TEMPLATE instances (BEHAVIOR on ribbonEmissiveMaterial,
    // GLOBAL/LOCAL on clayMaterial) -- created EAGERLY in create_renderer()
    // and registered in push_theme_to_scene(), same "lazy creation is a
    // known trap" reasoning as laneMaterial/objectClassMaterial. ribbonTint
    // mirrors each template's last-pushed baseColor rgb (MaterialInstance
    // has no getter) -- the staleness fade needs that color back, same
    // pattern as objectClassTint.
    static constexpr size_t kPathRoleCount = 3;  // BEHAVIOR, GLOBAL, LOCAL
    filament::Material* ribbonEmissiveMaterial = nullptr;
    filament::MaterialInstance* ribbonMaterial[kPathRoleCount] = {};
    detail::Float3 ribbonTint[kPathRoleCount] = {};

    // THE keying decision (Task 5 Step 4, see ribbon.cpp): SceneAssembly::
    // paths routinely holds FOUR ribbons over THREE roles (both shipped
    // profiles put two rows on role LOCAL) -- keying by role would let the
    // second LOCAL ribbon silently overwrite the first's entity every
    // frame. Keyed by SLOT INDEX into active().paths instead, each slot
    // carrying its own content signature (role + point_count +
    // hash(first,last point), same diff-cache mechanism as
    // mapElementMeshes/chunk_signature) so a slot rebuilds only when ITS
    // ribbon's content actually changed -- a role change on a slot re-homes
    // it to the right material. `meshes` is >1 only when point_count
    // exceeds polyline_chunks()'s uint16-index-buffer ceiling.
    // fadeInstance/fadeAlpha are GLOBAL/LOCAL-only (see ribbon.cpp): BEHAVIOR
    // fades by setting ribbonMaterial[BEHAVIOR]'s own alpha directly
    // (ribbon_emissive.mat carries one), never by an instance swap.
    struct RibbonSlot {
        PathRole role = PathRole::LOCAL;
        uint64_t signature = 0;
        bool has_signature = false;  // false forces the first build regardless
                                      // of an (unlikely) signature==0 collision
        std::vector<Mesh> meshes;
        // Sum of every mesh's own vertex count, recorded at build time
        // (ribbon.cpp's build_slot_meshes()) -- Filament's RenderableManager
        // exposes no vertex-count getter, so this is the only source for
        // ribbon_test_hooks.hpp's ribbon_vertex_count() (Task 5 Step 4's
        // LongPathSplitsAcrossMeshesWithoutTruncation: proving no point is
        // lost at the uint16 chunk-split seam).
        uint32_t totalVertexCount = 0;
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 1.0f;
        // The actual half-width (theme.ribbon.width_m * 0.5) build_slot_
        // meshes() used for THIS slot's geometry the last time it rebuilt
        // (user directive 2026-08-20, ITEM 1) -- NOT a Filament AABB query:
        // add_mesh() gives every mesh the same hard-coded declared culling
        // box (renderer.cpp's kGroundHalfExtent), unrelated to the strip's
        // real extent, the identical reason ego.cpp's egoFallbackDims isn't
        // sourced from a bounding-box getter either (see that field's own
        // comment). ribbon_test_hooks.hpp's WidthChangeRebuildsGeometry test
        // reads this back to prove a width-only theme change (no PathRibbon
        // point data touched at all) actually rebuilt the strip at the new
        // width, since the vertex COUNT alone (2 per point) can't tell width
        // apart from any other rebuild -- it's width-independent.
        float halfWidthM = 0.0f;
    };
    std::vector<RibbonSlot> ribbonSlots;

    // Ground grids (Epic 2 Task 6 / VM-025): OGM occupancy grids as
    // theme-colored ground textures. groundGridMaterial is ground_grid.mat
    // (built once, from ground_grid_filamat.h); groundGridMaterialInstance
    // is TWO eager per-KIND instances (0 = dynamic OGM, 1 = gradient OGM),
    // same "created EAGERLY in create_renderer, themed in
    // push_theme_to_scene" reasoning as laneMaterial/objectClassMaterial/
    // ribbonMaterial. groundGridFreeColor/OccupiedColor mirror what was last
    // pushed (MaterialInstance has no getter, same pattern as
    // laneMaterialBaseColor); groundGridAlpha[kind] mirrors the staleness
    // knob update_ground_grids() (ground_grid.cpp) drives every frame --
    // ground_grid_test_hooks.hpp's regression test reads it.
    //
    // ponytail ceiling: the material instance (and therefore its texture
    // sampler binding) is keyed by KIND, not by slot -- unlike ribbonSlots'
    // per-SLOT keying (Task 5's fix for the shipped-two-LOCAL-rows case).
    // Both shipped profiles ship exactly ONE row per ogm kind
    // (dynamic_ogm/gradient_ogm), so this never collides today; a future
    // profile shipping TWO rows of the SAME kind would have the second
    // slot's texture silently overwrite the first's binding on that kind's
    // shared instance -- upgrade to per-slot instances (ribbon.cpp's
    // pattern) if that ever ships.
    static constexpr size_t kGroundGridKindCount = 2;  // dynamic OGM, gradient OGM
    filament::Material* groundGridMaterial = nullptr;
    filament::MaterialInstance* groundGridMaterialInstance[kGroundGridKindCount] = {};
    detail::Float3 groundGridFreeColor{};
    detail::Float3 groundGridOccupiedColor{};
    float groundGridAlpha[kGroundGridKindCount] = {1.0f, 1.0f};

    // One quad + one texture per LIVE GroundGridLayer, keyed by SLOT INDEX
    // into active().grids (same "index into the array, not a content key"
    // shape as ribbonSlots -- see ground_grid.cpp's own comment for why a
    // content signature isn't needed here: unlike map/ribbon geometry, an
    // OGM's dims/origin/resolution are effectively static after the
    // adapter's first full grid, per real occupancy-grid semantics, so a
    // plain field-equality check is enough to decide "rebuild the quad").
    struct GroundGridSlot {
        uint8_t kind = 0;
        Mesh quad;
        bool has_geometry = false;
        Vec3 origin{};
        double resolution_m = 0.0;
        uint32_t width_cells = 0, height_cells = 0;  // last GEOMETRY build's dims
        filament::Texture* texture = nullptr;
        uint32_t texWidth = 0, texHeight = 0;  // last TEXTURE build's dims (0 = none yet)
        // Incremented every time `texture` is destroyed+rebuilt (never on an
        // in-place setImage() upload) -- ground_grid_test_hooks.hpp's
        // DimensionChangeRecreatesTheTexture reads this rather than
        // comparing raw pointer values: Filament's Texture wrapper objects
        // are fixed-size regardless of pixel dims, so a destroy()
        // immediately followed by a same-size allocation can (and, measured
        // empirically writing this test, DOES) hand back the identical
        // address -- pointer non-equality is not a reliable proxy for "a
        // new object was built" here, only a generation counter is.
        uint32_t textureGeneration = 0;
        // Upload gate: the g.last_update_sec this slot's texture pixels were
        // LAST uploaded from (ground_grid.cpp's update_ground_grids() sets
        // this right after every upload_occupancy_texture() call). -1.0
        // (never a valid sim/wall clock reading) means "never uploaded yet",
        // so the first live grid always uploads regardless of its
        // last_update_sec value. Lets a same-content frame (no new ingest()
        // since the last render) skip the setImage() call entirely instead
        // of re-sending byte-identical pixels every frame.
        double last_upload_sec = -1.0;
        // Bumped every time upload_occupancy_texture() actually runs for
        // this slot (gated or not) -- ground_grid_test_hooks.hpp's
        // ground_grid_texture_upload_count() reads this to prove the gate
        // above actually suppresses redundant uploads.
        uint32_t uploadCount = 0;
    };
    std::vector<GroundGridSlot> groundGridSlots;

    // Alert polygons (Epic 2 Task 7 / VM-026). alertMaterial[severity] are
    // THREE eager clay_translucent.mat instances (0 info/1 warning/2
    // critical, indexed by AlertPolygon::severity directly -- no enum, see
    // scene.h), each seeded with its FIXED kAlertSeverityAlpha constant --
    // constant, not a fade, same "created EAGERLY in create_renderer() and
    // registered in push_theme_to_scene()" reasoning as every other
    // per-category template (laneMaterial/objectClassMaterial/
    // ribbonMaterial/groundGridMaterialInstance). alertTint mirrors each
    // template's last-pushed baseColor rgb (MaterialInstance has no
    // getter) -- the staleness fade needs that color back, same pattern as
    // objectClassTint/ribbonTint.
    static constexpr size_t kAlertSeverityCount = 3;  // info/warning/critical
    filament::MaterialInstance* alertMaterial[kAlertSeverityCount] = {};
    detail::Float3 alertTint[kAlertSeverityCount] = {};

    // One triangulated mesh per LIVE AlertPolygon, keyed by SLOT INDEX into
    // active().alerts -- AlertPolygon is frozen with no id (same as
    // PathRibbon), so slot index is the only stable key one publish's
    // array offers (RibbonSlot's exact reasoning, renderer_internal.hpp
    // above). `signature` (severity + point data, alert_polygons.cpp) is
    // the content-signature-diff pattern map_elements.cpp/ribbon.cpp both
    // use, keyed here by slot rather than by content hash because, unlike
    // the HD-map's rolling window, an alert slot's IDENTITY (not just its
    // content) is genuinely positional within one publish.
    //
    // UNLIKE ObjectEntity/RibbonSlot's opaque<->translucent SWITCH: an
    // alert polygon lives on clay_translucent.mat from the instant it
    // exists (its severity's constant alpha) -- there is no opaque state
    // to switch from. `fadeInstance` is therefore non-null ONLY while
    // staleness_alpha < 1.0 (a per-slot instance multiplying the
    // severity's constant down); nullptr means "bound directly to the
    // shared alertMaterial[severity] template", mirroring objects.cpp's/
    // ribbon.cpp's exact fadeInstance/fadeAlpha bookkeeping otherwise.
    // `fadeAlpha` mirrors whatever alpha was last actually applied to this
    // slot's renderable (constant, or constant*staleness) -- ALWAYS kept
    // current (even while fresh, where it equals kAlertSeverityAlpha[
    // severity]) so a theme switch mid-fade (push_theme_to_scene()) can
    // re-push a live fadeInstance's colour without resetting its alpha.
    struct AlertSlot {
        uint8_t severity = 0;
        uint64_t signature = 0;
        bool has_signature = false;
        Mesh mesh;
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 0.0f;
    };
    std::vector<AlertSlot> alertSlots;

    // Generic markers (Epic 2 Task 8 / VM-027, the §7 parity-guarantee
    // fallback). genericMarkerMaterial is ONE eager clay.mat instance (the
    // theme-neutral default -- GenericMarker::color alpha==0, "no colour
    // supplied" -- reusing palette.object_tints.unknown, the existing
    // "unclassified" tint token, per the epic's "zero new theme fields"
    // rule), created EAGERLY in create_renderer() and registered in
    // push_theme_to_scene(), same "lazy creation is a known trap" reasoning
    // as every other per-category template above. genericMarkerNeutralTint
    // mirrors its last-pushed baseColor rgb (MaterialInstance has no
    // getter) -- the staleness fade needs that color back, same pattern as
    // objectClassTint/ribbonTint/alertTint.
    filament::MaterialInstance* genericMarkerMaterial = nullptr;
    detail::Float3 genericMarkerNeutralTint{};

    // Per-supplied-color clay.mat instances (GenericMarker::color, alpha !=
    // 0), keyed by an 8-bit-per-channel quantized rgb packed into a
    // uint32_t (generic_markers.cpp's quantize_color()) -- created LAZILY
    // the first time a marker asks for that exact quantized color, unlike
    // every other per-category template above (the set of colors isn't
    // known until data arrives). ponytail: no eviction/cap -- a publisher
    // that churns a new color every frame would leak one instance per
    // distinct color forever; add an LRU cap if that ever shows up in
    // practice (no shipped profile does today).
    std::unordered_map<uint32_t, filament::MaterialInstance*> genericMarkerColorInstances;

    // Shared unit geometry for the primitive types that don't carry their
    // own point data (CUBE/SPHERE/CYLINDER/ARROW/TEXT -- see scene.h's
    // GenericMarker comment: `points` is LINE_*/POINTS/TRIANGLE_LIST-only),
    // built ONCE, lazily, the first time each is actually needed (same
    // "build lazily on first use" shape as objects.cpp's
    // ensure_shared_arrow_mesh() for the velocity arrow). Every marker of
    // that primitive gets its OWN entity (GenericMarkerSlot::
    // sharedGeomEntity below) bound to the SAME vb/ib, posed by its own
    // TransformManager transform -- the geometry itself is never rebuilt
    // per marker. `entity` on each of these is unused (template geometry,
    // never itself added to the scene, exactly sharedArrowMesh's
    // convention).
    // ARROW markers reuse sharedArrowMesh (objects.cpp's velocity arrow) --
    // no genericArrowMesh (review 2026-08-20, duplicate geometry).
    Mesh genericCubeMesh, genericSphereMesh, genericCylinderMesh, genericTextMesh;

    // Epic 2 Task 8 (VM-027): one live renderable per GenericMarker SLOT
    // INDEX into active().markers (GenericMarker is frozen with no id, same
    // "index is the only stable key one publish's array offers" reasoning
    // as RibbonSlot/AlertSlot above). Exactly one of three geometry shapes
    // is populated at a time, keyed by `primitive`:
    //  - CUBE/SPHERE/CYLINDER/ARROW/TEXT: `sharedGeomEntity`, bound to one
    //    of VisualRenderer's shared unit meshes above, posed every frame by
    //    a TransformManager transform from the marker's
    //    position/heading_rad/scale -- geometry is never rebuilt, only
    //    re-posed. TEXT is translation-only (a fixed-size placeholder
    //    billboard, see generic_markers.cpp Step 3 -- VM-030 owns real
    //    orientation/scale for text).
    //  - LINE_STRIP/LINE_LIST/POINTS/TRIANGLE_LIST: `ownMesh`, this slot's
    //    OWN geometry baked directly from the marker's already-world-space
    //    `points` (same convention as AlertPolygon/MapElement -- no
    //    TransformManager transform at all), rebuilt only when
    //    `geomSignature` changes. `ownMesh` doubles as the MESH
    //    load-failure clay-box fallback (same "own procedural triangle
    //    mesh" shape, see generic_markers.cpp).
    //  - MESH: `meshAsset`, this slot's OWN glTF asset (one createAsset()
    //    per slot, gltfio's simple non-instanced path -- ponytail: two
    //    MESH markers sharing the same mesh_path each get their OWN parse,
    //    not a shared/pooled asset the way ObjectClassPool shares one glTF
    //    parse across many TrackedObjects; upgrade to a path-keyed shared
    //    pool, objects.cpp's pattern, if a publisher ever spams many
    //    identical MESH markers), posed by a TransformManager transform
    //    exactly like the shared-geometry primitives above (ROS
    //    Marker::MESH_RESOURCE convention: `scale` multiplies the mesh's
    //    OWN native units directly -- there is no "measured bbox" for an
    //    arbitrary user mesh the way ObjectEntity's dims/unitFootprint
    //    scale has).
    struct GenericMarkerSlot {
        bool active = false;
        MarkerPrimitive primitive = MarkerPrimitive::CUBE;
        utils::Entity sharedGeomEntity;   // CUBE/SPHERE/CYLINDER/ARROW/TEXT: never owns a vb/ib
        Mesh ownMesh;                     // LINE_*/POINTS/TRIANGLE_LIST, and the MESH fallback box
        uint64_t geomSignature = 0;
        bool hasGeomSignature = false;
        filament::gltfio::FilamentAsset* meshAsset = nullptr;  // MESH, successful load only
        std::string meshPathLoaded;
        bool meshIsFallback = false;
        // Material bookkeeping -- same fresh<->stale swap shape as
        // ObjectEntity/RibbonSlot/AlertSlot above: `fadeInstance` is
        // non-null ONLY while staleness_alpha < 1.0 (a per-slot
        // clay_translucent.mat instance, created from
        // clayTranslucentMaterial, NEVER MaterialInstance::duplicate() of
        // the opaque template); nullptr means every one of this slot's
        // renderable entities is bound directly to the shared
        // genericMarkerMaterial template or this marker's own
        // quantized-color instance. `tint` mirrors whichever rgb is
        // currently applied (MaterialInstance has no getter) -- the fade
        // needs that color back, same pattern as
        // objectClassTint/ribbonTint/alertTint.
        detail::Float3 tint{};
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 1.0f;
    };
    std::vector<GenericMarkerSlot> genericMarkerSlots;

    // Bumped every time update_generic_markers() actually allocates a NEW
    // GPU/ECS resource for a slot (a shared-geometry entity, a per-slot
    // LINE_*/POINTS/TRIANGLE_LIST mesh, or a MESH marker's glTF asset/
    // fallback box) -- NEVER on a transform-only or material-swap-only
    // update. generic_markers_test_hooks.hpp's
    // PooledRenderablesNoPerFrameAllocation reads this: 30 renders of an
    // unchanged marker set must freeze it after frame 1.
    uint32_t genericMarkerAllocCount = 0;
    // Bumped once per marker whose primitive value isn't one of the ten
    // scene.h enum values this library handles (a caller passing a raw
    // out-of-range uint8_t -- the adapter itself never emits one, see
    // generic_marker.cpp's own ROS-type table -- this is the library's own
    // defensive floor). UnknownOrUnsupportedPrimitiveIsSkippedAndCounted.
    uint32_t genericMarkerUnknownCount = 0;
    // Distinct MESH mesh_path strings that have already WARNed once on
    // load failure (spec §9 "asset load failure -> clay-box fallback, WARN
    // once") -- same shape as objectClassMissingWarned but keyed by path
    // string since MESH markers aren't grouped into a fixed class set.
    std::unordered_set<std::string> genericMarkerMeshWarned;
};

// Namespace-scope free function (Step 7e) — was a lambda local to
// create_renderer() capturing `&engine`/`&em`/`&r`, which meant only
// renderer.cpp's own create_renderer() could call it. A lambda can't be
// called from ego.cpp, a different translation unit, which is exactly what
// Task 4 Step 5 needs to do for the ego's clay-box fallback. `r.engine` and
// utils::EntityManager::get() replace the old `&engine`/`&em` captures (`em`
// was always just that singleton accessor, nothing stateful worth threading
// through).
void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices,
              filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows);

}  // namespace mpviz
