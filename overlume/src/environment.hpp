// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gltfio/FilamentAsset.h>

#include "overlume/scene.h"

namespace overlume {

class VisualRenderer;

class EnvironmentSource {
public:
    virtual ~EnvironmentSource() = default;
    virtual void update(VisualRenderer& r, Vec3 ego_map_pos) = 0;
    virtual void teardown(VisualRenderer& r) = 0;
    virtual void set_visible(VisualRenderer& r, bool visible) = 0;
    virtual size_t loaded_count() const = 0;
    virtual size_t scene_membership_count(VisualRenderer& r) const = 0;
    virtual EnvironmentSourceState state() const = 0;
    virtual bool provides_ground() const { return false; }
};

inline constexpr double kLoadRadiusM = 300.0;
inline constexpr double kUnloadRadiusM = 400.0;

struct EnvironmentChunk {
    std::string id;
    std::string path;
    Vec3 center{};
    double radius_m = 0.0;
};

class BakedEnvironmentSource : public EnvironmentSource {
public:
    BakedEnvironmentSource(std::string dir, std::vector<EnvironmentChunk> chunks, GeoAnchor anchor);

    void update(VisualRenderer& r, Vec3 ego_map_pos) override;
    void teardown(VisualRenderer& r) override;
    void set_visible(VisualRenderer& r, bool visible) override;

    size_t loaded_chunk_count() const { return loaded_.size(); }
    size_t loaded_count() const override { return loaded_chunk_count(); }
    size_t scene_membership_count(VisualRenderer& r) const override;
    EnvironmentSourceState state() const override { return EnvironmentSourceState::BAKED; }

private:
    std::string dir_;
    std::vector<EnvironmentChunk> chunks_;
    GeoAnchor anchor_;

    struct LoadedChunk {
        filament::gltfio::FilamentAsset* asset = nullptr;
        Vec3 center{};
    };
    std::unordered_map<std::string, LoadedChunk> loaded_;
    bool visible_ = true;
    std::unordered_set<std::string> failed_;
    const EnvironmentChunk* find_chunk(const std::string& id) const {
        for (const EnvironmentChunk& c : chunks_) {
            if (c.id == id) return &c;
        }
        return nullptr;
    }
};

std::unique_ptr<BakedEnvironmentSource> open_baked_environment_source(const std::string& dir,
                                                                      GeoAnchor anchor);

std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor);

}
