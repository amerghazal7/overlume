// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_mvt.hpp"

#include <bit>
#include <cmath>
#include <limits>
#include <string_view>
#include <utility>

namespace overlume::mapbox {

namespace {

constexpr uint64_t kPolygon = 3;  // Feature.type POLYGON

// One protobuf field. Vector tiles use wire types 0 (varint), 1 (fixed64), 2 (length-delimited)
// and 5 (fixed32); groups (3, 4) and unknown types make the enclosing message malformed.
struct Field {
    uint64_t number = 0;
    uint32_t wire = 0;
    uint64_t varint = 0;            // wire type 0
    const uint8_t* data = nullptr;  // wire types 1, 2 and 5: the payload
    size_t size = 0;
};

// At most 10 bytes; a 10th byte's bits past bit 63 are dropped, as protobuf does.
bool read_varint(const uint8_t*& p, const uint8_t* end, uint64_t& out) {
    out = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        if (p == end) return false;
        const uint8_t b = *p++;
        out |= static_cast<uint64_t>(b & 0x7f) << shift;
        if (!(b & 0x80)) return true;
    }
    return false;
}

bool read_u32(const uint8_t*& p, const uint8_t* end, uint32_t& out) {
    uint64_t v = 0;
    if (!read_varint(p, end, v) || v > std::numeric_limits<uint32_t>::max()) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

// Reads the field at p and moves p past it. false: broken framing.
bool next_field(const uint8_t*& p, const uint8_t* end, Field& f) {
    uint64_t key = 0;
    if (!read_varint(p, end, key)) return false;
    f.number = key >> 3;
    f.wire = static_cast<uint32_t>(key & 7);
    if (f.wire == 0) return read_varint(p, end, f.varint);
    uint64_t size = 0;
    if (f.wire == 1) {
        size = 8;
    } else if (f.wire == 5) {
        size = 4;
    } else if (f.wire != 2 || !read_varint(p, end, size)) {
        return false;
    }
    if (size > static_cast<uint64_t>(end - p)) return false;
    f.data = p;
    f.size = static_cast<size_t>(size);
    p += f.size;
    return true;
}

uint64_t fixed_le(const Field& f) {
    uint64_t v = 0;
    for (size_t i = 0; i < f.size; ++i) v |= static_cast<uint64_t>(f.data[i]) << (8 * i);
    return v;
}

int64_t zigzag64(uint64_t v) { return static_cast<int64_t>((v >> 1) ^ (~(v & 1) + 1)); }

int32_t zigzag32(uint32_t v) { return static_cast<int32_t>((v >> 1) ^ (~(v & 1) + 1)); }

// A layer's Value message. It should hold one field; when it holds several, each is kept.
struct TagValue {
    std::optional<double> number;  // float, double, int64, uint64 or sint64
    std::optional<bool> boolean;
    std::optional<std::string> text;
};

bool decode_value(const Field& value, TagValue& out) {
    const uint8_t* const end = value.data + value.size;
    Field f;
    for (const uint8_t* p = value.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number == 1 && f.wire == 2) {
            out.text.emplace(reinterpret_cast<const char*>(f.data), f.size);
        } else if (f.number == 2 && f.wire == 5) {
            out.number = std::bit_cast<float>(static_cast<uint32_t>(fixed_le(f)));
        } else if (f.number == 3 && f.wire == 1) {
            out.number = std::bit_cast<double>(fixed_le(f));
        } else if (f.number == 4 && f.wire == 0) {
            out.number = static_cast<double>(static_cast<int64_t>(f.varint));
        } else if (f.number == 5 && f.wire == 0) {
            out.number = static_cast<double>(f.varint);
        } else if (f.number == 6 && f.wire == 0) {
            out.number = static_cast<double>(zigzag64(f.varint));
        } else if (f.number == 7 && f.wire == 0) {
            out.boolean = f.varint != 0;
        }
    }
    return true;
}

// A NaN or infinite height would reach the mesh; it counts as absent.
std::optional<double> finite(const std::optional<double>& n) {
    return n && std::isfinite(*n) ? n : std::nullopt;
}

// streets-v8 writes `extrude` and `underground` as the strings "true"/"false"; a bool also counts.
std::optional<bool> flag(const TagValue& v) {
    if (v.boolean) return v.boolean;
    if (v.text == "true") return true;
    if (v.text == "false") return false;
    return std::nullopt;
}

// Packed (key index, value index) pairs. false: an odd count or an index out of range.
bool decode_tags(const Field& tags, const std::vector<std::string>& keys,
                 const std::vector<TagValue>& values, MvtBuilding& b) {
    const uint8_t* p = tags.data;
    const uint8_t* const end = tags.data + tags.size;
    while (p < end) {
        uint32_t k = 0, v = 0;
        if (!read_u32(p, end, k) || !read_u32(p, end, v)) return false;
        if (k >= keys.size() || v >= values.size()) return false;
        const std::string& key = keys[k];
        const TagValue& value = values[v];
        if (key == "height") {
            b.height = finite(value.number);
        } else if (key == "min_height") {
            b.min_height = finite(value.number);
        } else if (key == "extrude") {
            b.extrude = flag(value);
        } else if (key == "underground") {
            b.underground = flag(value);
        } else if (key == "type" && value.text) {
            b.type = *value.text;
        }
    }
    return true;
}

// Drops a closing repeat of the first vertex, then classifies the ring by its surveyor's-formula
// area in tile coordinates (y down): positive starts a polygon, negative is a hole of the last
// one, zero (which covers fewer than three distinct vertices) drops the ring. false: a hole came
// before any exterior ring.
bool close_ring(std::vector<MvtPoint>& ring, std::vector<MvtPolygon>& polygons) {
    while (ring.size() > 1 && ring.back().x == ring.front().x && ring.back().y == ring.front().y)
        ring.pop_back();
    double twice_area = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const MvtPoint& a = ring[i];
        const MvtPoint& b = ring[(i + 1) % ring.size()];
        twice_area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    if (twice_area > 0.0) {
        polygons.push_back({std::move(ring), {}});
    } else if (twice_area < 0.0) {
        if (polygons.empty()) return false;
        polygons.back().holes.push_back(std::move(ring));
    }
    return true;
}

// Polygon geometry (MVT 2.1 section 4.3): per ring a MoveTo of one point, a LineTo and a
// ClosePath, as zigzag deltas from a cursor that starts at (0, 0) and persists across rings.
// Repeated vertices are dropped. false: malformed.
bool decode_polygons(const Field& geometry, std::vector<MvtPolygon>& polygons) {
    constexpr uint32_t kMoveTo = 1, kLineTo = 2, kClosePath = 7;
    constexpr int64_t kMin = std::numeric_limits<int32_t>::min();
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    const uint8_t* p = geometry.data;
    const uint8_t* const end = geometry.data + geometry.size;
    int64_t x = 0, y = 0;
    std::vector<MvtPoint> ring;
    bool open = false;
    while (p < end) {
        uint32_t command = 0;
        if (!read_u32(p, end, command)) return false;
        const uint32_t id = command & 7, count = command >> 3;
        if (id == kClosePath) {
            if (count != 1 || !open || !close_ring(ring, polygons)) return false;
            open = false;
            continue;
        }
        if (id == kMoveTo && count == 1 && !open) {
            ring.clear();
            open = true;
        } else if (id != kLineTo || !open) {
            return false;
        }
        // Nothing is reserved from `count`: a forged count runs out of bytes first.
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t dx = 0, dy = 0;
            if (!read_u32(p, end, dx) || !read_u32(p, end, dy)) return false;
            x += zigzag32(dx);
            y += zigzag32(dy);
            if (x < kMin || x > kMax || y < kMin || y > kMax) return false;
            const MvtPoint pt{static_cast<int32_t>(x), static_cast<int32_t>(y)};
            if (ring.empty() || pt.x != ring.back().x || pt.y != ring.back().y) ring.push_back(pt);
        }
    }
    return !open;
}

// One feature. false: not a polygon, malformed, or left without a ring; the caller counts it.
bool decode_feature(const Field& feature, const std::vector<std::string>& keys,
                    const std::vector<TagValue>& values, MvtBuilding& b) {
    const uint8_t* const end = feature.data + feature.size;
    Field f, tags, geometry;
    uint64_t type = 0;  // UNKNOWN
    for (const uint8_t* p = feature.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number == 1 && f.wire == 0) {
            b.id = f.varint;
        } else if (f.number == 2 && f.wire == 2) {
            tags = f;
        } else if (f.number == 3 && f.wire == 0) {
            type = f.varint;
        } else if (f.number == 4 && f.wire == 2) {
            geometry = f;
        }
    }
    return type == kPolygon && decode_tags(tags, keys, values, b) &&
           decode_polygons(geometry, b.polygons) && !b.polygons.empty();
}

// nullopt: the layer's framing breaks before its name, so it may be the building layer.
std::optional<bool> named_building(const Field& layer) {
    const uint8_t* const end = layer.data + layer.size;
    Field f;
    for (const uint8_t* p = layer.data; p < end;) {
        if (!next_field(p, end, f)) return std::nullopt;
        if (f.number == 1 && f.wire == 2)
            return std::string_view(reinterpret_cast<const char*>(f.data), f.size) == "building";
    }
    return false;
}

// false: the layer's framing or extent is broken, which leaves nothing in the tile usable.
bool decode_building_layer(const Field& layer, MvtBuildingLayer& out) {
    const uint8_t* const end = layer.data + layer.size;
    std::vector<std::string> keys;
    std::vector<TagValue> values;
    Field f;
    // The key and value tables may follow the features, so a first pass reads them; it also
    // checks the framing of every field. The version (field 15) is not read: version 1 layers
    // decode by the version 2 rules (streets-v8 tiles are version 2).
    for (const uint8_t* p = layer.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number == 3 && f.wire == 2) {
            keys.emplace_back(reinterpret_cast<const char*>(f.data), f.size);
        } else if (f.number == 4 && f.wire == 2) {
            if (!decode_value(f, values.emplace_back())) return false;
        } else if (f.number == 5 && f.wire == 0) {
            // Tile units are divided by the extent downstream.
            if (f.varint == 0 || f.varint > std::numeric_limits<uint32_t>::max()) return false;
            out.extent = static_cast<uint32_t>(f.varint);
        }
    }
    for (const uint8_t* p = layer.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number != 2 || f.wire != 2) continue;
        MvtBuilding b;
        if (decode_feature(f, keys, values, b)) {
            out.buildings.push_back(std::move(b));
        } else {
            ++out.skipped_features;
        }
    }
    return true;
}

}

MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size) {
    MvtBuildingLayer out;
    const uint8_t* const end = data + size;
    Field layer;
    for (const uint8_t* p = data; p < end;) {
        if (!next_field(p, end, layer)) return {};
        if (layer.number != 3 || layer.wire != 2) continue;  // Tile.layers
        const std::optional<bool> building = named_building(layer);
        if (!building || (*building && !decode_building_layer(layer, out))) return {};
    }
    out.ok = true;
    return out;
}

bool passes_standard_3d_filter(const MvtBuilding& b) {
    return b.extrude.value_or(false) && !b.underground.value_or(true);
}

}
