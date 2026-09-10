// renderer_internal.hpp — internal-only (`-I src`), not installed, not POD.
// The `VisualRenderer` class definition, plus the `Mesh`/`Vertex`/`add_mesh`
// helper types/functions it needs, extracted out of renderer.cpp so other
// translation units in the same library target (ego.cpp, objects.cpp, etc)
// can see `class VisualRenderer` and reuse `add_mesh`.
//
// Pulls in <filament/...> headers, so it is deliberately never included by
// tests/*.cpp — see ego_test_hooks.hpp (Filament-free) for what tests use
// instead. Test binaries only get `-I src` for Filament-free headers like
// scene_buffer.hpp; they're never granted visual_renderer's PRIVATE
// Filament include dir, so a test TU including this header would fail to
// compile.
#pragma once

#include <cstdint>
#include <memory>
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

#include "camera_textures.hpp"
#include "scene_buffer.hpp"
#include "theme.hpp"
#include "theme_transition.hpp"

namespace mpviz {

// Full class body stays defined in renderer.cpp; forward-declared here so
// VisualRenderer::platform (a bare pointer member) can name the type
// without this header needing the EGL-touching definition itself.
class HeadlessEglPlatform;
// Forward-declared: VisualRenderer::environmentSource below is a
// std::unique_ptr member, which needs the complete type only where its
// destructor is actually instantiated (destroy_renderer(), renderer.cpp --
// which #includes environment.hpp before `delete r` ever runs). Full
// definition lives in environment.hpp (library-internal, same
// "never #included by node code" status as this header itself).
class EnvironmentSource;
// Forward-declared for the same reason as EnvironmentSource above:
// VisualRenderer::bowl (VM-091, Task 2) is a std::unique_ptr member whose
// destructor is only instantiated where bowl.hpp's full BowlState
// definition is visible (destroy_renderer(), renderer.cpp -- which
// #includes bowl.hpp before `delete r` runs).
struct BowlState;

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
    // Vertex count add_mesh() built this with -- Filament's VertexBuffer
    // has no getter for it, so this mirrors what was passed in (same
    // "no getter, so mirror the value at the call site" reasoning as every
    // *MaterialBaseColor field below). Read by
    // map_element_total_vertex_count() (map_elements_test_hooks.hpp).
    uint32_t vertexCount = 0;
    // Staleness fade: null while fresh (bound to the shared opaque
    // per-kind template), a clay_translucent.mat instance while fading —
    // same mechanism as ObjectEntity::fadeInstance/fadeAlpha and
    // AlertSlot::fadeInstance/fadeAlpha below. Lives on Mesh itself (not a
    // parallel map) because map_elements.cpp's mapElementMeshes cache is
    // already the per-content-signature record every fading mesh needs;
    // other Mesh users (ground/grid/ribbon) simply never populate these.
    filament::MaterialInstance* fadeInstance = nullptr;
    float fadeAlpha = 1.0f;
};

// Re-uploads `verts` into `mesh`'s EXISTING VertexBuffer via setBufferAt --
// no VertexBuffer::Builder, no entity/RenderableManager touch, so the
// mesh's identity and scene membership never change. Used by
// ribbon.cpp/trajectory_carpet.cpp to apply the ego-proximity clip as a
// same-buffer position update instead of a mesh rebuild.
// `verts.size()` must equal the buffer's original vertex count -- callers
// only ever pass back a same-sized copy of what add_mesh() built. No-op if
// `mesh.vb` is null.
void update_mesh_positions(filament::Engine& engine, Mesh& mesh, std::vector<Vertex> verts);

// Shared by the ego-following ground/grid patch and map_elements.cpp's
// per-element geometry (measured in whole meters). build_grid_lines()'s
// own `step` (renderer.cpp) IS this same symbol — not a hand-kept copy —
// so the ground and grid lines cannot drift out of alignment.
inline constexpr float kGridPitchM = 2.0f;

// Staleness fade, shared by every stale-able category (~0.5s fade, never
// pop). SceneBuffer::staleness_alpha(now, last_update, kStaleFadeStartSec,
// kStaleFadeTimeoutSec) is the one ramp every category calls -- no
// per-category alpha math anywhere.
inline constexpr double kStaleFadeStartSec = 0.5;
inline constexpr double kStaleFadeTimeoutSec = 1.0;

// Shared ribbon-family half-width floor: whatever a theme's margin math
// derives, the extruded strip never goes narrower than this. Lives here
// (not ribbon.cpp's anonymous namespace) because trajectory_carpet.cpp's
// velocity ribbon (VM-077 carpet-as-ribbon redirect, 2026-09-10) clamps
// against the identical floor and must not silently drift from ribbon.cpp's
// own three roles if this value is ever retuned.
inline constexpr float kRibbonMinHalfWidthM = 0.12f;

// Constant per-severity alphas (info ghosted low, warning, critical) —
// not a fade. AlertPolygon has no `role`, so a lower constant alpha is the
// only way to express "info reads as background". alert_polygons.cpp
// multiplies staleness_alpha into this per-entity (never past it);
// push_theme_to_scene() (renderer.cpp) pushes it into the three eager
// per-severity template instances alongside their palette.alert.* rgb —
// alpha isn't a theme field, this array is why.
inline constexpr float kAlertSeverityAlpha[3] = {0.18f, 0.35f, 0.45f};  // info, warning, critical

// Builds per-vertex orientation quaternions from flat-shaded normals
// (SurfaceOrientation's "normals only" mode) — shared by renderer.cpp's
// ground/grid builders and map_elements.cpp's lane/crosswalk builders.
// Declared here so other translation units can call it; defined once in
// renderer.cpp.
void fill_tangent_frames(std::vector<Vertex>& verts, const std::vector<filament::math::float3>& normals);
// One flat unit arrow (shaft+head, +X, tail x=0, tip x=1) shared by
// objects.cpp's velocity arrows and generic_markers.cpp's ARROW primitive.
// Defined once in renderer.cpp; both call sites fill r.sharedArrowMesh, so
// there is exactly one GPU copy of this geometry.
void build_unit_arrow(std::vector<Vertex>& verts, std::vector<uint16_t>& indices);

// Tears down one Mesh's GPU resources and removes its entity from the
// scene. Shared so map_elements.cpp's diff-based cache eviction and
// renderer.cpp's destroy_renderer() don't each have to remember the same
// teardown order (scene removal, entity destruction, vb/ib destruction).
void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh);

// Lazily builds the shared gltfio AssetLoader + ubershader
// MaterialProvider + ResourceLoader on `r`, the first time either
// set_ego_model() (ego.cpp) or set_object_model_dir() (objects.cpp) needs
// one -- so a second AssetLoader never exists in this library. No-op
// (returns true) if already built; false only if MaterialProvider/
// AssetLoader construction itself fails.
bool ensure_gltf_loader(VisualRenderer& r);

// One instanced glTF class's pool. `asset` is the primary asset (owns
// every instance below -- destroying it destroys them all, per gltfio).
// `pool` is every FilamentInstance* ever created for this class (initial
// batch plus createInstance() growth); `freeList` is the subset not
// currently bound to a live TrackedObject (there is no destroyInstance()
// in gltfio, see objects.cpp). `unitFootprint` is this class's own
// normalized bounding-box extent, read once via
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

// One live per-track renderable, diffed every render_frame() call by
// update_objects() (acquire/update/release, never rebuilt wholesale).
// Exactly one of `glInstance` (a class pool's FilamentInstance) or
// `proceduralBox` (the UNKNOWN/missing-model/pool-exhausted fallback,
// always built as a unit 1x1x1 box so its TransformManager scale is
// `dims / (1,1,1)` -- identical math to the glTF path) is populated.
struct ObjectEntity {
    ObjectClass cls = ObjectClass::UNKNOWN;
    filament::gltfio::FilamentInstance* glInstance = nullptr;
    Mesh proceduralBox;
    utils::Entity transformRoot;
    utils::Entity arrowEntity;  // velocity arrow; null iff velocity == {0,0,0}
    Mesh pathRibbon;            // predicted-path ribbon; empty iff no path data
    uint64_t pathSignature = 0; // content signature (see objects.cpp), rebuild-skip guard
    Vec3 appliedScale{0.0, 0.0, 0.0};  // test hook: DimensionsDriveScale -- same
                                       // "never the RenderableManager AABB" reasoning
                                       // as egoFallbackDims below.
    // Staleness fade: null while fresh (bound to the shared opaque
    // objectClassMaterial[cls] template); non-null while fading -- a
    // per-entity clay_translucent.mat instance created from
    // VisualRenderer::clayTranslucentMaterial, never
    // MaterialInstance::duplicate() of the opaque template. fadeAlpha
    // mirrors what was last pushed into it (MaterialInstance has no
    // getter).
    filament::MaterialInstance* fadeInstance = nullptr;
    float fadeAlpha = 1.0f;
};

// release_object_entity()/update_objects() are declared in objects.hpp;
// ObjectEntity/ObjectClassPool live here because VisualRenderer's member
// fields need the complete types.

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
    // clay.mat (shared: ground here, ego clay-box fallback + glTF remap)
    // and clay_faded.mat (grid-only, per-vertex alpha).
    filament::Material* clayMaterial = nullptr;
    filament::Material* clayFadedMaterial = nullptr;
    filament::MaterialInstance* groundMaterial = nullptr;
    filament::MaterialInstance* gridMaterial = nullptr;
    // Ego contrast color: a dedicated clay.mat instance tinted with its own
    // cross-theme palette.ego token instead of reusing groundMaterial
    // (which would render the ego the same color as the ground it stands
    // on). Created eagerly in create_renderer() -- push_theme_to_scene()
    // only runs at create_renderer() and during set_theme() transitions,
    // so a lazily-created instance would render with clay.mat's
    // compiled-in defaults until the next theme push.
    // egoMaterialBaseColor mirrors it (MaterialInstance has no getter) --
    // ego_test_hooks.hpp's regression test reads this.
    filament::MaterialInstance* egoMaterial = nullptr;
    detail::Float3 egoMaterialBaseColor{};
    Mesh ground;
    Mesh grid;
    uint32_t width = 0;
    uint32_t height = 0;

    // laneMaterial is a clay.mat instance (tinted differently from
    // ground/grid) created eagerly in create_renderer() and registered in
    // push_theme_to_scene() -- see that function for why lazy creation on
    // first map data is the bug this avoids. laneMaterialBaseColor mirrors
    // the value last pushed to "baseColor" (MaterialInstance has no
    // getter); map_elements_test_hooks.hpp's regression test reads it to
    // prove the setParameter call actually happened. mapElementMeshes is
    // the diff-cache update_map_elements() (map_elements.cpp) keys by
    // content signature (point count + first/last point, not array
    // position -- the source topic is a rolling window, so position isn't
    // stable), so a rebuild only touches elements that actually changed.
    filament::MaterialInstance* laneMaterial = nullptr;
    detail::Float3 laneMaterialBaseColor{};
    // Per-kind tint instances -- same clay.mat template, same
    // eager-creation/push_theme_to_scene reasoning as laneMaterial above.
    // laneMaterial itself stays the fallback for STOPLINE/JUNCTION/OTHER
    // (kinds with no dedicated token).
    filament::MaterialInstance* laneCenterlineMaterial = nullptr;
    detail::Float3 laneCenterlineMaterialBaseColor{};
    filament::MaterialInstance* laneBoundaryMaterial = nullptr;
    detail::Float3 laneBoundaryMaterialBaseColor{};
    filament::MaterialInstance* crosswalkMaterial = nullptr;
    detail::Float3 crosswalkMaterialBaseColor{};
    filament::MaterialInstance* roadMaterial = nullptr;
    detail::Float3 roadMaterialBaseColor{};
    // ROAD_EDGE: the road's outer boundary, solid yellow-family, never
    // dashed (map_elements.cpp gates dashing on LEFT_BOUNDARY/
    // RIGHT_BOUNDARY only) -- no longer falls back to laneMaterial.
    filament::MaterialInstance* roadEdgeMaterial = nullptr;
    detail::Float3 roadEdgeMaterialBaseColor{};
    std::unordered_map<uint64_t, Mesh> mapElementMeshes;
    // Incremented once per adopt_or_build() cache-miss branch in
    // update_map_elements() (map_elements.cpp) -- a content signature
    // already present in mapElementMeshes is adopted, not rebuilt, and
    // does not bump this.
    uint64_t mapElementRebuildCount = 0;

    // theme_dir is the retained copy of RenderConfig::theme_assets_dir
    // (borrow-then-copy rule, see api.h) -- kept for set_theme() to
    // re-load from later. active_theme is whatever create_renderer()
    // resolved at startup (loaded file or kFallbackTheme()); set_theme()
    // is what changes it after creation.
    std::string theme_dir;
    detail::Theme active_theme;
    // True iff the theme active right after create_renderer() actually
    // came from disk (load_theme() succeeded), false if create_renderer()
    // substituted detail::kFallbackTheme(). Read back by
    // theme_assets_loaded() (scene.h).
    bool theme_assets_loaded = false;

    // Set when set_theme() is called, cleared (nullopt) once the eased
    // blend reaches t>=1.0 -- render_frame()'s apply_current_theme() is
    // then a no-op every subsequent call, not a recomputation of an
    // already-finished blend every frame.
    std::optional<detail::ThemeTransition> theme_transition;

    // Double-buffered scene ingest. set_scene() is exactly
    // `r->scene_buffer.publish(scene);` -- no Filament call happens there;
    // render_frame() reads scene_buffer.active() instead.
    detail::SceneBuffer scene_buffer;

    // Ego, built/loaded once by set_ego_model() (ego.cpp), never
    // rebuilt/reloaded afterward. Exactly one of `egoAsset` (gltfio path)
    // or `egoFallback.entity` (clay-box path) is populated;
    // `egoTransformEntity` is whichever one update_ego_transform()
    // (ego.cpp) actually drives -- the glTF asset's transform root
    // (already has a TransformManager component, built by gltfio) or the
    // fallback box's own mesh entity (given one explicitly, since
    // add_mesh()-built entities don't get a TransformManager component
    // automatically).
    //
    // sharedMaterialProvider/sharedAssetLoader/sharedResourceLoader are
    // not ego-specific -- objects.cpp needs the exact same machinery, and
    // a second AssetLoader in this library would duplicate it. Lazily
    // built by ensure_gltf_loader() the first time either set_ego_model()
    // or set_object_model_dir() runs; torn down exactly once in
    // destroy_renderer(), after egoAsset and every ObjectClassPool::asset
    // below have been destroyed through it.
    filament::gltfio::MaterialProvider* sharedMaterialProvider = nullptr;
    filament::gltfio::AssetLoader* sharedAssetLoader = nullptr;
    filament::gltfio::ResourceLoader* sharedResourceLoader = nullptr;
    filament::gltfio::FilamentAsset* egoAsset = nullptr;
    Mesh egoFallback;
    utils::Entity egoTransformEntity;
    // The actual dims build_ego_box() used for egoFallback -- not the same
    // as querying egoFallback's RenderableManager AABB, which reports
    // add_mesh()'s hard-coded declared culling box (renderer.cpp's
    // kGroundHalfExtent), unrelated to the ego's real geometry. {0,0,0}
    // until build_ego_fallback() runs.
    Vec3 egoFallbackDims{0.0, 0.0, 0.0};

    // clay_translucent.mat (the one settable-alpha Material, built once
    // from clay_translucent_filamat.h) and its per-class opaque clay.mat
    // template instances (indexed by static_cast<uint8_t>(ObjectClass); the
    // 6th slot is UNKNOWN) -- created eagerly in create_renderer() and
    // registered in push_theme_to_scene(). objectClassTint mirrors each
    // template's last-pushed baseColor (MaterialInstance has no getter) --
    // the staleness fade needs that color back.
    static constexpr size_t kObjectClassCount = 6;  // CAR..UNKNOWN
    filament::Material* clayTranslucentMaterial = nullptr;
    filament::MaterialInstance* objectClassMaterial[kObjectClassCount] = {};
    detail::Float3 objectClassTint[kObjectClassCount] = {};
    // Warn-once flags: missing/unloadable model per class, and
    // kMaxInstancesPerClass exhaustion per class -- both fall back to the
    // procedural box, neither is fatal, neither should spam per-object.
    bool objectClassMissingWarned[kObjectClassCount] = {};

    // Per-class instanced glTF pools, keyed by
    // static_cast<uint8_t>(ObjectClass). Absent key == no model ever
    // loaded for that class -- always the procedural box path, non-fatal.
    std::unordered_map<uint8_t, ObjectClassPool> objectClassPools;

    // Shared unit-arrow geometry (velocity arrows): built lazily the first
    // time any object has a nonzero velocity (objects.cpp), reused by
    // every object's own arrowEntity via RenderableManager -- one shared
    // mesh scaled/rotated per object. `entity` is unused (a geometry
    // template, never itself added to the scene).
    Mesh sharedArrowMesh;

    // Live per-track entities, diffed every render_frame() call by
    // update_objects() (objects.cpp) -- keyed by TrackedObject::id.
    std::unordered_map<uint32_t, ObjectEntity> objectEntities;

    // ribbonEmissiveMaterial is the BEHAVIOR role's dedicated Material
    // (ribbon_emissive.mat, built once); ribbonMaterial[role] are the
    // three per-role opaque template instances (BEHAVIOR on
    // ribbonEmissiveMaterial, GLOBAL/LOCAL on clayMaterial) -- created
    // eagerly in create_renderer() and registered in
    // push_theme_to_scene(). ribbonTint mirrors each template's
    // last-pushed baseColor rgb (MaterialInstance has no getter), same
    // pattern as objectClassTint.
    static constexpr size_t kPathRoleCount = 3;  // BEHAVIOR, GLOBAL, LOCAL
    filament::Material* ribbonEmissiveMaterial = nullptr;
    filament::MaterialInstance* ribbonMaterial[kPathRoleCount] = {};
    detail::Float3 ribbonTint[kPathRoleCount] = {};

    // Keying decision: SceneAssembly::paths routinely holds four ribbons
    // over three roles (both shipped profiles put two rows on role
    // LOCAL) -- keying by role would let the second LOCAL ribbon silently
    // overwrite the first's entity every frame. Keyed by slot index into
    // active().paths instead, each slot carrying its own content signature
    // (role + point_count + hash(first,last point)) so a slot rebuilds
    // only when its own content changed; a role change re-homes it to the
    // right material. `meshes` is >1 only when point_count exceeds
    // polyline_chunks()'s uint16-index-buffer ceiling.
    // fadeInstance/fadeAlpha are GLOBAL/LOCAL-only: BEHAVIOR fades by
    // setting ribbonMaterial[BEHAVIOR]'s own alpha directly
    // (ribbon_emissive.mat carries one), never by an instance swap.
    struct RibbonSlot {
        PathRole role = PathRole::LOCAL;
        uint64_t signature = 0;
        bool has_signature = false;  // false forces the first build regardless
                                      // of an (unlikely) signature==0 collision
        std::vector<Mesh> meshes;
        // Sum of every mesh's own vertex count, recorded at build time
        // (ribbon.cpp's build_slot_meshes()) -- RenderableManager has no
        // vertex-count getter. Read by ribbon_test_hooks.hpp's
        // ribbon_vertex_count() to prove no point is lost at the uint16
        // chunk-split seam.
        uint32_t totalVertexCount = 0;
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 1.0f;
        // The actual half-width (theme.ribbon.width_m * 0.5) build_slot_
        // meshes() used for this slot's geometry the last time it rebuilt
        // -- not a Filament AABB query: add_mesh() gives every mesh the
        // same hard-coded declared culling box, unrelated to the strip's
        // real extent (same reason egoFallbackDims isn't sourced from a
        // bounding-box getter). ribbon_test_hooks.hpp's
        // WidthChangeRebuildsGeometry reads this to prove a width-only
        // theme change actually rebuilt the strip, since vertex count
        // alone can't tell width apart from any other rebuild.
        float halfWidthM = 0.0f;
        // The first geometry point build_slot_meshes() actually used --
        // ribbon.cpp's update_ribbons() may hand it a polyline truncated
        // at an interpolated ego-proximity clip point rather than the raw
        // PathRibbon::points[0], so this mirrors what was really built
        // (same "not a Filament read-back" reasoning as halfWidthM).
        // ribbon_test_hooks.hpp's ribbon_slot_first_point() reads it to
        // prove a clipped ribbon starts at the interpolated clip point. Set
        // at build time to the raw first point (unclipped baseline);
        // apply_ribbon_clip() (ribbon.cpp) overwrites it with the CURRENT
        // effective first vertex whenever a clip is actually applied, so it
        // always mirrors what's really on screen, not just what was last
        // built.
        Vec3 firstPointM{};
        // Per-mesh CPU copies of the FULL, unclipped extruded strip (2*m
        // positions) and each of its m points' own arc-length station --
        // retained so the ego-proximity clip can be re-applied EVERY frame
        // as a degenerate-vertex collapse (polyline.hpp's
        // collapse_clipped_positions()) instead of folding into the
        // content signature and forcing a rebuild. Populated only at
        // build_slot_meshes() (content-change, ~8Hz message rate); parallel
        // to `meshes`.
        std::vector<std::vector<Vec3>> baseStripPositions;
        std::vector<std::vector<double>> pointStations;
        // The clip state actually painted onto the GPU buffers last time
        // apply_ribbon_clip() ran -- lets it skip a redundant
        // setBufferAt() when the ego hasn't crossed a new quantized
        // station since the previous frame (parked ego -> zero uploads).
        // Compared by quantized_units (an exact integer), not the derived
        // station_m double -- same "hash the integer, not its float
        // expansion" reasoning polyline.hpp's PolylineClip comment gives.
        // has_applied_clip false forces one apply right after a rebuild
        // (whose fresh buffers are always unclipped).
        bool has_applied_clip = false;
        bool appliedClipActive = false;
        int64_t appliedClipUnits = 0;
    };
    std::vector<RibbonSlot> ribbonSlots;
    // Incremented once per slot rebuild (content, role, or quantized
    // ego-clip station changed) -- same cache-miss-counter pattern as
    // mapElementRebuildCount above. Used by
    // Ribbon.ParkedEgoCausesZeroRibbonRebuilds to prove a stationary ego
    // causes no rebuilds across many frames, not just "the pixels look
    // the same".
    uint64_t ribbonRebuildCount = 0;

    // OGM occupancy grids as theme-colored ground textures.
    // groundGridMaterial is ground_grid.mat (built once);
    // groundGridMaterialInstance is two eager per-kind instances (0 =
    // dynamic OGM, 1 = gradient OGM), created eagerly and themed in
    // push_theme_to_scene(). groundGridFreeColor/OccupiedColor mirror what
    // was last pushed (MaterialInstance has no getter);
    // groundGridAlpha[kind] mirrors the staleness knob
    // update_ground_grids() (ground_grid.cpp) drives every frame.
    //
    // ponytail: the material instance (and its texture sampler binding)
    // is keyed by kind, not by slot, unlike ribbonSlots' per-slot keying.
    // Both shipped profiles ship exactly one row per ogm kind, so this
    // never collides today; a future profile shipping two rows of the
    // same kind would have the second slot's texture silently overwrite
    // the first's binding -- upgrade to per-slot instances (ribbon.cpp's
    // pattern) if that ever ships.
    static constexpr size_t kGroundGridKindCount = 2;  // dynamic OGM, gradient OGM
    filament::Material* groundGridMaterial = nullptr;
    filament::MaterialInstance* groundGridMaterialInstance[kGroundGridKindCount] = {};
    detail::Float3 groundGridFreeColor{};
    detail::Float3 groundGridOccupiedColor{};
    float groundGridAlpha[kGroundGridKindCount] = {1.0f, 1.0f};

    // One quad + one texture per live GroundGridLayer, keyed by slot index
    // into active().grids (same shape as ribbonSlots). No content
    // signature needed here -- an OGM's dims/origin/resolution are
    // effectively static after the adapter's first full grid, so a plain
    // field-equality check decides "rebuild the quad" (see
    // ground_grid.cpp).
    struct GroundGridSlot {
        uint8_t kind = 0;
        Mesh quad;
        bool has_geometry = false;
        Vec3 origin{};
        double resolution_m = 0.0;
        uint32_t width_cells = 0, height_cells = 0;  // last GEOMETRY build's dims
        filament::Texture* texture = nullptr;
        uint32_t texWidth = 0, texHeight = 0;  // last TEXTURE build's dims (0 = none yet)
        // Incremented every time `texture` is destroyed+rebuilt (never on
        // an in-place setImage() upload). Filament's Texture wrapper
        // objects are fixed-size regardless of pixel dims, so a destroy()
        // immediately followed by a same-size allocation can (measured
        // empirically) hand back the identical address -- pointer
        // non-equality is not a reliable "new object" proxy here, only
        // this generation counter is.
        // ground_grid_test_hooks.hpp's DimensionChangeRecreatesTheTexture
        // reads this.
        uint32_t textureGeneration = 0;
        // Upload gate: the g.last_update_sec this slot's texture pixels
        // were last uploaded from (update_ground_grids() sets this after
        // every upload_occupancy_texture() call). -1.0 means "never
        // uploaded yet", so the first live grid always uploads. Lets a
        // same-content frame skip setImage() instead of re-sending
        // byte-identical pixels.
        double last_upload_sec = -1.0;
        // Bumped every time upload_occupancy_texture() actually runs for
        // this slot -- ground_grid_texture_upload_count() reads this to
        // prove the gate above suppresses redundant uploads.
        uint32_t uploadCount = 0;
    };
    std::vector<GroundGridSlot> groundGridSlots;

    // alertMaterial[severity] are three eager clay_translucent.mat
    // instances (0 info/1 warning/2 critical, indexed by
    // AlertPolygon::severity directly -- no enum, see scene.h), each
    // seeded with its fixed kAlertSeverityAlpha constant (not a fade).
    // alertTint mirrors each template's last-pushed baseColor rgb
    // (MaterialInstance has no getter), same pattern as
    // objectClassTint/ribbonTint.
    static constexpr size_t kAlertSeverityCount = 3;  // info/warning/critical
    filament::MaterialInstance* alertMaterial[kAlertSeverityCount] = {};
    detail::Float3 alertTint[kAlertSeverityCount] = {};

    // One triangulated mesh per live AlertPolygon, keyed by slot index
    // into active().alerts -- AlertPolygon has no id, so slot index is the
    // only stable key one publish's array offers (same reasoning as
    // RibbonSlot above). `signature` (severity + point data,
    // alert_polygons.cpp) is the same content-signature-diff pattern
    // map_elements.cpp/ribbon.cpp use, keyed by slot rather than content
    // hash because an alert slot's identity (not just its content) is
    // positional within one publish.
    //
    // Unlike ObjectEntity/RibbonSlot's opaque<->translucent switch: an
    // alert polygon lives on clay_translucent.mat from the instant it
    // exists, so there's no opaque state to switch from. `fadeInstance` is
    // non-null only while staleness_alpha < 1.0 (a per-slot instance
    // multiplying the severity's constant down); nullptr means bound
    // directly to the shared alertMaterial[severity] template.
    // `fadeAlpha` mirrors whatever alpha was last applied (constant, or
    // constant*staleness) -- kept current even while fresh, so a theme
    // switch mid-fade can re-push a live fadeInstance's colour without
    // resetting its alpha.
    struct AlertSlot {
        uint8_t severity = 0;
        uint64_t signature = 0;
        bool has_signature = false;
        Mesh mesh;
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 0.0f;
    };
    std::vector<AlertSlot> alertSlots;

    // genericMarkerMaterial is one eager clay.mat instance (the
    // theme-neutral default for GenericMarker::color alpha==0, reusing
    // palette.object_tints.unknown -- zero new theme fields), created
    // eagerly and registered in push_theme_to_scene().
    // genericMarkerNeutralTint mirrors its last-pushed baseColor rgb
    // (MaterialInstance has no getter), same pattern as
    // objectClassTint/ribbonTint/alertTint.
    filament::MaterialInstance* genericMarkerMaterial = nullptr;
    detail::Float3 genericMarkerNeutralTint{};

    // Per-supplied-color clay.mat instances (GenericMarker::color, alpha
    // != 0), keyed by an 8-bit-per-channel quantized rgb packed into a
    // uint32_t (generic_markers.cpp's quantize_color()) -- created lazily
    // the first time a marker asks for that exact color (the set of
    // colors isn't known until data arrives).
    // ponytail: no eviction/cap -- a publisher that churns a new color
    // every frame would leak one instance per distinct color forever; add
    // an LRU cap if that ever shows up in practice.
    std::unordered_map<uint32_t, filament::MaterialInstance*> genericMarkerColorInstances;

    // Shared unit geometry for primitive types that don't carry their own
    // point data (CUBE/SPHERE/CYLINDER/ARROW/TEXT -- `points` is
    // LINE_*/POINTS/TRIANGLE_LIST-only, see scene.h's GenericMarker
    // comment), built once, lazily, the first time each is needed. Every
    // marker of that primitive gets its own entity
    // (GenericMarkerSlot::sharedGeomEntity below) bound to the same
    // vb/ib, posed by its own TransformManager transform -- geometry is
    // never rebuilt per marker. `entity` on each of these is unused
    // (template geometry, same convention as sharedArrowMesh).
    // ARROW markers reuse sharedArrowMesh (objects.cpp's velocity arrow) --
    // no separate genericArrowMesh.
    Mesh genericCubeMesh, genericSphereMesh, genericCylinderMesh, genericTextMesh;

    // One live renderable per GenericMarker slot index into
    // active().markers (GenericMarker has no id, same reasoning as
    // RibbonSlot/AlertSlot above). Exactly one of three geometry shapes is
    // populated at a time, keyed by `primitive`:
    //  - CUBE/SPHERE/CYLINDER/ARROW/TEXT: `sharedGeomEntity`, bound to one
    //    of VisualRenderer's shared unit meshes above, posed every frame by
    //    a TransformManager transform from the marker's
    //    position/heading_rad/scale -- geometry is never rebuilt, only
    //    re-posed. TEXT is translation-only (a fixed-size placeholder
    //    billboard; real orientation/scale for text is VM-030).
    //  - LINE_STRIP/LINE_LIST/POINTS/TRIANGLE_LIST: `ownMesh`, this slot's
    //    own geometry baked directly from the marker's already-world-space
    //    `points` (no TransformManager transform at all), rebuilt only
    //    when `geomSignature` changes. `ownMesh` doubles as the MESH
    //    load-failure clay-box fallback.
    //  - MESH: `meshAsset`, this slot's own glTF asset (one createAsset()
    //    per slot, gltfio's simple non-instanced path), posed by a
    //    TransformManager transform like the shared-geometry primitives
    //    above (ROS Marker::MESH_RESOURCE convention: `scale` multiplies
    //    the mesh's own native units directly -- there is no measured
    //    bbox for an arbitrary user mesh the way ObjectEntity's
    //    dims/unitFootprint scale has).
    //    ponytail: two MESH markers sharing the same mesh_path each get
    //    their own parse, not a shared/pooled asset like ObjectClassPool;
    //    upgrade to a path-keyed shared pool (objects.cpp's pattern) if a
    //    publisher ever spams many identical MESH markers.
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
        // non-null only while staleness_alpha < 1.0 (a per-slot
        // clay_translucent.mat instance, created from
        // clayTranslucentMaterial, never MaterialInstance::duplicate() of
        // the opaque template); nullptr means bound directly to the
        // shared genericMarkerMaterial template or this marker's own
        // quantized-color instance. `tint` mirrors whichever rgb is
        // currently applied (MaterialInstance has no getter), same
        // pattern as objectClassTint/ribbonTint/alertTint.
        detail::Float3 tint{};
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 1.0f;
    };
    std::vector<GenericMarkerSlot> genericMarkerSlots;

    // Bumped every time update_generic_markers() allocates a new GPU/ECS
    // resource for a slot -- never on a transform-only or
    // material-swap-only update. generic_markers_test_hooks.hpp's
    // PooledRenderablesNoPerFrameAllocation requires this to freeze after
    // frame 1 for an unchanged marker set.
    uint32_t genericMarkerAllocCount = 0;
    // Bumped once per marker whose primitive value isn't one of the ten
    // scene.h enum values this library handles (a defensive floor against
    // a raw out-of-range uint8_t; the adapter itself never emits one).
    uint32_t genericMarkerUnknownCount = 0;
    // Distinct MESH mesh_path strings that have already warned once on
    // load failure -- same shape as objectClassMissingWarned but keyed by
    // path string since MESH markers aren't grouped into a fixed class
    // set.
    std::unordered_set<std::string> genericMarkerMeshWarned;

    // Point clouds (Epic 3 Task 6 / VM-035). pointCloudMaterial is
    // point_cloud.mat (built once); pointCloudMaterialInstance is the
    // SINGLE instance the whole layer shares (Step 2's decision: a layer
    // that fades as one unit needs no per-entity/per-chunk instancing) --
    // created eagerly and themed nowhere (point_cloud.mat carries no
    // baseColor/tint parameter; the per-point rgba IS the color).
    // pointCloudAlpha mirrors the last-pushed "alpha" staleness knob
    // (MaterialInstance has no getter).
    filament::Material* pointCloudMaterial = nullptr;
    filament::MaterialInstance* pointCloudMaterialInstance = nullptr;
    float pointCloudAlpha = 1.0f;

    // One (possibly chunked, past kMaxPointsPerMesh) set of Filament meshes
    // per live PointCloud, keyed by slot index into active().point_clouds --
    // PointCloud has no id, same reasoning as RibbonSlot/GroundGridSlot/
    // AlertSlot above. `signature` is a content signature (point count +
    // first/last point) so an unchanged cloud causes zero rebuilds, same
    // "diff, don't always rebuild" shape as every other category.
    struct PointCloudSlot {
        uint64_t signature = 0;
        bool has_signature = false;
        std::vector<Mesh> meshes;
        uint32_t totalVertexCount = 0;
    };
    std::vector<PointCloudSlot> pointCloudSlots;

    // Trajectory carpets (VM-077, redirected 2026-09-10 to a velocity-colored
    // RIBBON per the user's own directive -- see trajectory_carpet.cpp's file
    // header for the full redirect rationale; layer/knob names kept
    // unchanged, only the geometry path changed). trajectoryCarpetMaterial is
    // trajectory_carpet.mat (built once); trajectoryCarpetMaterialInstance is
    // the SINGLE instance the whole layer shares -- same "a layer that fades
    // as one unit needs no per-entity/per-chunk instancing" reasoning as
    // pointCloudMaterialInstance above. trajectoryCarpetAlpha mirrors the
    // last-pushed "alpha" staleness knob (MaterialInstance has no getter).
    filament::Material* trajectoryCarpetMaterial = nullptr;
    filament::MaterialInstance* trajectoryCarpetMaterialInstance = nullptr;
    // Death-fade twin (2026-09-10 flicker fix): trajectory_carpet.mat is
    // OPAQUE now; while staleness_alpha() < 1 trajectory_carpet.cpp rebinds
    // carpet meshes to this single shared trajectory_carpet_faded.mat
    // instance (settable alpha, blending fade) -- ribbon.cpp's own
    // fresh-opaque/stale-translucent swap shape, one eager instance instead
    // of per-slot lazy ones because the whole layer fades as one unit.
    filament::Material* trajectoryCarpetFadedMaterial = nullptr;
    filament::MaterialInstance* trajectoryCarpetFadedMaterialInstance = nullptr;
    float trajectoryCarpetAlpha = 1.0f;

    // One (possibly chunked, past kMaxPointsPerMesh) set of Filament meshes
    // per live TrajectoryCarpet, keyed by slot index into
    // active().trajectory_carpets -- same reasoning as PointCloudSlot above
    // (TrajectoryCarpet has no id). `signature` mirrors ribbon_signature()'s
    // shape (point count + first/last point + half-width + ego-clip state,
    // deliberately NO color term -- see the plan's 2026-09-10 section for
    // why this property is pinned independent of the disproven H2 root-cause
    // theory) so an unchanged carpet (or one whose color alone drifted)
    // causes zero rebuilds.
    struct TrajectoryCarpetSlot {
        uint64_t signature = 0;
        bool has_signature = false;
        std::vector<Mesh> meshes;
        uint32_t totalVertexCount = 0;
        // The actual half-width build_slot_meshes() used for this slot's
        // geometry the last time it rebuilt -- same "not a Filament AABB
        // query" reasoning as RibbonSlot::halfWidthM.
        float halfWidthM = 0.0f;
        // Test-hook mirror ONLY (trajectory_carpet_test_hooks.hpp) -- the
        // resolved (post alpha-zero-sentinel substitution) per-vertex rgba
        // of the FIRST built mesh, kept purely so a test can prove the
        // per-vertex color path without a Filament vertex-buffer read-back.
        // Same "not a Filament read-back" reasoning as RibbonSlot's
        // firstPointM/halfWidthM above.
        std::vector<uint32_t> firstMeshRgba;
        // Test-hook mirror ONLY: the actual world-space z each first-mesh
        // vertex was built with, INCLUDING the velocity-ribbon z-lift --
        // catches the "adapter flattens to 0.0, renderer must lift it above
        // the opaque ground plane or it z-fights invisible" regression a
        // live-bag verification pass found (VM-077, 2026-09-09): every
        // other vertex-count/mesh-count assertion in this file would have
        // stayed green even with the lift silently removed.
        std::vector<float> firstMeshZ;
        // Same "clip via degenerate-vertex collapse, applied every frame,
        // no rebuild" mechanism as RibbonSlot -- see its own comment.
        // baseStripRgba is the resolved (post alpha-zero-sentinel) per-
        // vertex color, parallel to baseStripPositions; collapse only ever
        // touches position, so color is copied back verbatim on re-upload.
        std::vector<std::vector<Vec3>> baseStripPositions;
        std::vector<std::vector<uint32_t>> baseStripRgba;
        std::vector<std::vector<double>> pointStations;
        bool has_applied_clip = false;
        bool appliedClipActive = false;
        int64_t appliedClipUnits = 0;
        // True while this slot's meshes are bound to the shared faded
        // instance (staleness swap) -- reset by build_slot_meshes(), which
        // always binds fresh geometry to the opaque instance.
        bool boundFaded = false;
        // Mirrors RibbonSlot::firstPointM -- the current effective first
        // vertex (post-collapse), for the same test-hook reasoning.
        Vec3 firstPointM{};
    };
    std::vector<TrajectoryCarpetSlot> trajectoryCarpetSlots;
    // Incremented once per slot rebuild (content, half-width, or quantized
    // ego-clip station changed) -- same cache-miss-counter pattern as
    // ribbonRebuildCount above. TrajectoryCarpet.SameStationPositionsWith
    // DriftingColorAloneCausesNoRebuild (test_trajectory_carpet.cpp) reads
    // this to pin the signature property: a message that only changes
    // per-vertex color, with identical positions, must NOT bump this counter.
    uint64_t trajectoryCarpetRebuildCount = 0;

    // Camera bowl (VM-090, unified-engine migration Task 1; ADR-0005): up to
    // kMaxBowlCameras persistent camera textures, keyed by camera index.
    // cameraCount == 0 means set_bowl_config() has never succeeded -- every
    // camera_textures.cpp entry point gates on this, same "0 = unconfigured"
    // convention as every other category count above. Task 2's bowl.cpp is
    // the first real mesh/material consumer of these slots (no bowl entity
    // exists yet at this task).
    uint32_t cameraCount = 0;
    CameraTextureSlot cameraSlots[kMaxBowlCameras];
    // Requested bowl visibility (set_bowl_visible) -- this task only stores
    // it; Task 2's update_bowl() is what actually adds/removes the bowl
    // entity from the scene based on this flag.
    bool bowlVisible = false;
    // Bowl mesh/material (VM-091, Task 2): null until build_bowl() (bowl.cpp,
    // called from camera_textures.cpp's set_bowl_config()) succeeds. A full
    // re-bake destroys and replaces this outright -- see build_bowl()'s own
    // comment.
    std::unique_ptr<BowlState> bowl;
    // Environment (VM-052): buildingMaterial is an eager clay.mat instance
    // (Decision 4 -- no new .mat file), themed from palette.building in
    // push_theme_to_scene() alongside every other eager instance above.
    // Buildings are the whole-time OPAQUE clay this comment block's own
    // header note describes -- no fade instance, no translucent twin.
    // environmentSource is null until set_environment_source() (scene.h)
    // succeeds; render_frame() calls its update() once per tick, gated on
    // it being non-null AND on ego.valid (renderer.cpp).
    filament::MaterialInstance* buildingMaterial = nullptr;
    std::unique_ptr<EnvironmentSource> environmentSource;
};

// Namespace-scope free function so a different translation unit (ego.cpp,
// objects.cpp) can call it -- a lambda local to create_renderer() couldn't.
void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices,
              filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows);

}  // namespace mpviz
