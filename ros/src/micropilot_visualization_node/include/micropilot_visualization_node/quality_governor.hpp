#pragma once
/** @file quality_governor.hpp
 *  @brief VM-040 (Epic 5): quality auto-drop with hysteresis.
 *
 *  Reads the render_ms instrumentation VM-034 already computes in
 *  timer_callback() (VisualizationNode::render_ms_) and, once
 *  governor_enabled_ is true, decides when to drop or recover one
 *  mpviz::RenderConfig::quality preset. The node applies that decision by
 *  calling mpviz::set_quality() (VM-040's own appended library entry point,
 *  scene.h) -- this class never touches the renderer itself, on purpose:
 *  it's a pure, ROS-free state machine over a stream of doubles, same
 *  "small shared header, no ROS types, easily unit-tested" shape as
 *  diagnostics.hpp (BuildDiagnostics is the same kind of pure transform).
 *  That separation IS the test seam the AC asks for: a test feeds
 *  record_render_ms() synthetic samples directly -- deterministic, CI-safe,
 *  no GPU, no timer, no real overload needed to prove drop+recover.
 *
 *  Windows close by count (window_size samples) and are evaluated by p95
 *  (nearest-rank, same percentile() shape tools/viz_benchmark.cpp already
 *  uses); DROP/RECOVER/hysteresis/dwell semantics are the backlog VM-040
 *  Done note's decision of record -- see that note for the design rationale,
 *  not restated here. Per-field threshold justifications are on each param
 *  below.
 */

#include <cstdint>
#include <vector>

namespace mpviz_node
{

// A no-op most ticks -- only set on the tick that CLOSES a window and that
// window's own verdict actually changes the preset.
enum class QualityTransition : uint8_t
{
    NONE = 0,
    DROPPED = 1,
    RECOVERED = 2,
};

struct QualityGovernorParams
{
    // render_ms samples per evaluation window. At the node's fixed 33 ms
    // publish timer (~30 Hz), 30 samples is ~1 s of ticks.
    uint32_t window_size{30};
    // A window's p95 above this drops one preset (once the dwell floor has
    // elapsed). Comfortably under budget_probe.md's 33 ms wall-timer
    // ceiling -- high enough that only real, sustained overload trips it.
    double drop_threshold_ms{28.0};
    // A window's p95 below this counts toward "headroom" (see
    // recover_windows_required below). Comfortably under
    // budget_probe.md's real bowl-on-driving render_ms p99 numbers
    // (~21-22 ms), so recovering back up requires meaningfully better than
    // today's typical load, not just noise. drop_threshold_ms >
    // recover_threshold_ms is the hysteresis gap itself -- a window that
    // lands between the two moves nothing.
    double recover_threshold_ms{18.0};
    // Consecutive good (below recover_threshold_ms) windows required before
    // RECOVER fires -- "sustained", not one lucky window.
    uint32_t recover_windows_required{3};
    // Windows that must be evaluated after ANY transition (drop or
    // recover) before another one can fire, regardless of how extreme the
    // samples in between are. The actual anti-flap floor.
    uint32_t min_dwell_windows{3};
};

// Pure hysteresis state machine -- no ROS types, no Filament, no renderer
// pointer. The node owns applying a returned transition (mpviz::set_quality()
// + a WARN log); this class only ever decides.
class QualityGovernor
{
public:
    QualityGovernor(QualityGovernorParams params, uint32_t initial_preset);

    // Feeds one render_ms sample (one render_frame() tick). Returns the
    // transition this sample's window closed on (NONE on every tick that
    // doesn't complete a window, and on a completed window whose p95
    // doesn't cross a threshold or is still inside the dwell floor).
    QualityTransition record_render_ms(double render_ms);

    uint32_t current_preset() const { return preset_; }

private:
    QualityGovernorParams params_;
    uint32_t preset_;
    std::vector<double> window_;
    // Starts already at the dwell floor (constructor) -- with no prior
    // transition, the dwell requirement is trivially satisfied, so the very
    // first window evaluated can drop (or recover) immediately if it
    // warrants it. Reset to 0 on every actual transition; see .cpp.
    uint32_t windows_since_transition_;
    uint32_t consecutive_good_windows_{0};
};

}  // namespace mpviz_node
