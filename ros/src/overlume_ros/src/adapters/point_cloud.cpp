// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/point_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

// STATED DEVIATION (see point_cloud.hpp's own header comment): soft-
// defaulted theme-token stand-ins for the intensity/height ramp endpoints
// -- no cross-toolchain path exists today for this gcc/libstdc++ adapter
// to read the clang/libc++ library's parsed theme.yaml. Replace with a
// real theme lookup if/when that read path is ever added.
constexpr uint8_t kIntensityLowRgb[3] = {24, 24, 40};
constexpr uint8_t kIntensityHighRgb[3] = {255, 214, 120};
constexpr uint8_t kHeightLowRgb[3] = {40, 70, 170};
constexpr uint8_t kHeightHighRgb[3] = {214, 60, 50};

enum class Tier { kRgb, kIntensity, kHeight, kFlat };

Tier ResolveTier(const std::string& color_mode, bool has_color, bool has_intensity) {
    if (color_mode == "flat") return Tier::kFlat;
    if (color_mode == "height") return Tier::kHeight;
    if (color_mode == "intensity") return has_intensity ? Tier::kIntensity : Tier::kHeight;
    // "rgb" and "auto" share the same fallback chain (this file's header
    // comment): rgb -> intensity -> height.
    if (has_color) return Tier::kRgb;
    return has_intensity ? Tier::kIntensity : Tier::kHeight;
}

uint8_t LerpByte(uint8_t lo, uint8_t hi, float t) {
    return static_cast<uint8_t>(std::lround(static_cast<float>(lo) +
                                            (static_cast<float>(hi) - static_cast<float>(lo)) * t));
}

// t in [0,1] (0.5 when the observed range is degenerate, lo==hi) -> a
// packed opaque rgba between `lo`/`hi`'s endpoint bytes.
uint32_t RampColor(const uint8_t lo[3], const uint8_t hi[3], float t) {
    return PackRgba(LerpByte(lo[0], hi[0], t), LerpByte(lo[1], hi[1], t), LerpByte(lo[2], hi[2], t),
                    255);
}

}  // namespace

PointCloudAdapter::PointCloudAdapter(const ProfileRow& row,
                                     const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf) {}

void PointCloudAdapter::ingest(const sensor_msgs::msg::PointCloud2& msg, double sim_time_sec) {
    ++stats_.msgs;

    const uint64_t n64 = static_cast<uint64_t>(msg.width) * msg.height;
    if (n64 == 0 || n64 > std::numeric_limits<uint32_t>::max()) {
        ++stats_.dropped_malformed;
        return;
    }
    const uint32_t n = static_cast<uint32_t>(n64);
    if (msg.point_step == 0 || msg.data.size() < static_cast<size_t>(n) * msg.point_step) {
        ++stats_.dropped_malformed;
        return;
    }

    // Field scan -- prior art: rendering_node.cpp:448-462. Only FLOAT32
    // fields are recognized (see this adapter's header comment); a field
    // present under a different wire datatype reads as absent.
    int offX = -1, offY = -1, offZ = -1, offColor = -1, offIntensity = -1;
    for (const auto& f : msg.fields) {
        if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
        if (f.name == "x")
            offX = static_cast<int>(f.offset);
        else if (f.name == "y")
            offY = static_cast<int>(f.offset);
        else if (f.name == "z")
            offZ = static_cast<int>(f.offset);
        else if (f.name == "rgb" && offColor < 0)
            offColor = static_cast<int>(f.offset);
        else if (f.name == "rgba" && offColor < 0)
            offColor = static_cast<int>(f.offset);
        else if (f.name == "intensity")
            offIntensity = static_cast<int>(f.offset);
    }
    if (offX < 0 || offY < 0 || offZ < 0) {
        // No coherent geometry at all -- whole message dropped, previously-
        // stored cloud (if any) keeps rendering.
        ++stats_.dropped_malformed;
        return;
    }

    // ONE lookup for the whole message -- same "per message, never per
    // point" rule every other adapter's frame transform follows.
    tf2::Transform xform;
    if (!tf_.lookup(msg.header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }

    const Tier tier = ResolveTier(row_.color_mode, offColor >= 0, offIntensity >= 0);

    // Pass 1: stride/max_points decimation + per-point transform (z is
    // NEVER flattened here -- see this file's header comment) + the
    // per-point malformed guard (NaN/Inf x/y/z drops just that point).
    struct RawPoint {
        tf2::Vector3 world;
        uint32_t color_bits = 0;  // meaningful iff tier == kRgb
        float intensity = 0.0f;   // meaningful iff tier == kIntensity
    };
    std::vector<RawPoint> raw;
    raw.reserve(row_.stride > 0 ? (n / row_.stride) + 1 : n);
    bool any_nan = false;
    const uint8_t* base = msg.data.data();
    for (uint32_t i = 0; i < n; i += row_.stride) {
        if (row_.max_points != 0 && raw.size() >= row_.max_points) break;
        const uint8_t* rec = base + static_cast<size_t>(i) * msg.point_step;
        float x, y, z;
        std::memcpy(&x, rec + offX, 4);
        std::memcpy(&y, rec + offY, 4);
        std::memcpy(&z, rec + offZ, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            any_nan = true;
            continue;
        }
        const tf2::Vector3 world = xform * tf2::Vector3(x, y, z);
        if (!std::isfinite(world.x()) || !std::isfinite(world.y()) || !std::isfinite(world.z())) {
            any_nan = true;
            continue;
        }
        RawPoint rp;
        rp.world = world;
        if (tier == Tier::kRgb) std::memcpy(&rp.color_bits, rec + offColor, 4);
        if (tier == Tier::kIntensity) std::memcpy(&rp.intensity, rec + offIntensity, 4);
        raw.push_back(rp);
    }
    if (any_nan) ++stats_.dropped_malformed;

    // Pass 2: intensity/height auto-range, over exactly the survivors
    // above (post-decimation) -- no profile-level override field exists
    // (see this file's header comment).
    float lo = std::numeric_limits<float>::infinity();
    float hi = -std::numeric_limits<float>::infinity();
    if (tier == Tier::kIntensity) {
        for (const auto& rp : raw) {
            lo = std::min(lo, rp.intensity);
            hi = std::max(hi, rp.intensity);
        }
    } else if (tier == Tier::kHeight) {
        for (const auto& rp : raw) {
            const float z = static_cast<float>(rp.world.z());
            lo = std::min(lo, z);
            hi = std::max(hi, z);
        }
    }

    std::vector<overlume::PointCloudPoint> next;
    next.reserve(raw.size());
    for (const auto& rp : raw) {
        overlume::PointCloudPoint p{};
        p.position = {rp.world.x(), rp.world.y(), rp.world.z()};
        switch (tier) {
            case Tier::kRgb: {
                // PCL packed-float convention: the field's bit pattern,
                // reinterpreted as a uint32_t, is 0x00RRGGBB / 0xAARRGGBB.
                const uint8_t r = static_cast<uint8_t>((rp.color_bits >> 16) & 0xFFu);
                const uint8_t g = static_cast<uint8_t>((rp.color_bits >> 8) & 0xFFu);
                const uint8_t b = static_cast<uint8_t>(rp.color_bits & 0xFFu);
                p.rgba = PackRgba(r, g, b, 255);
                break;
            }
            case Tier::kIntensity: {
                const float t =
                    (hi > lo) ? std::clamp((rp.intensity - lo) / (hi - lo), 0.0f, 1.0f) : 0.5f;
                p.rgba = RampColor(kIntensityLowRgb, kIntensityHighRgb, t);
                break;
            }
            case Tier::kHeight: {
                const float z = static_cast<float>(rp.world.z());
                const float t = (hi > lo) ? std::clamp((z - lo) / (hi - lo), 0.0f, 1.0f) : 0.5f;
                p.rgba = RampColor(kHeightLowRgb, kHeightHighRgb, t);
                break;
            }
            case Tier::kFlat:
            default:
                // alpha==0 sentinel (scene.h's PointCloudPoint comment) --
                // point_cloud.cpp substitutes the theme's neutral token.
                p.rgba = 0;
                break;
        }
        next.push_back(p);
    }

    storage_ = std::move(next);
    has_data_ = true;
    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void PointCloudAdapter::fill(overlume::ros::SceneAssembly& out) const {
    if (!has_data_) return;  // never received a valid message yet

    overlume::PointCloud pc{};
    pc.points = storage_.data();
    pc.point_count = static_cast<uint32_t>(storage_.size());
    pc.last_update_sec = last_update_sec_;
    out.point_clouds.push_back(pc);
}

}  // namespace overlume::ros
