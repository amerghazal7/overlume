// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Mapbox Vector Tiles (MVT 2.1 protobuf) and gzip streams synthesized in the test process, so no
// Mapbox bytes enter the tree. Header-only, C++17.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace overlume::testing::mvt {

// A vector-tile value (string, float, double, int64, uint64, sint64 or bool) as its encoded Value
// message.
struct Value {
    std::vector<uint8_t> message;
};

struct Feature {
    std::optional<uint64_t> id;
    std::vector<std::vector<std::array<int32_t, 2>>> rings;  // open rings, written as given
    std::vector<std::pair<std::string, Value>> tags;
    uint32_t geom_type = 3;  // POLYGON
};

struct Layer {
    std::string name;
    uint32_t extent = 4096;
    uint32_t version = 2;
    std::vector<Feature> features;
};

// Protobuf writers; tests also use them to hand-build malformed input.
namespace detail {

inline void put_varint(std::vector<uint8_t>& out, uint64_t v) {
    for (; v >= 0x80; v >>= 7) out.push_back(static_cast<uint8_t>(v | 0x80));
    out.push_back(static_cast<uint8_t>(v));
}

inline void put_tag(std::vector<uint8_t>& out, uint32_t field, uint32_t wire) {
    put_varint(out, (uint64_t{field} << 3) | wire);
}

inline void put_bytes(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint8_t>& b) {
    put_tag(out, field, 2);
    put_varint(out, b.size());
    out.insert(out.end(), b.begin(), b.end());
}

inline void put_string(std::vector<uint8_t>& out, uint32_t field, const std::string& s) {
    put_bytes(out, field, std::vector<uint8_t>(s.begin(), s.end()));
}

inline void put_le(std::vector<uint8_t>& out, uint64_t v, int bytes) {
    for (int i = 0; i < bytes; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

// Packed repeated uint32: feature tags and geometry.
inline void put_packed(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint32_t>& v) {
    std::vector<uint8_t> payload;
    for (uint32_t x : v) put_varint(payload, x);
    put_bytes(out, field, payload);
}

inline uint32_t command(uint32_t id, uint32_t count) { return (id & 7) | (count << 3); }

inline uint32_t zigzag32(int32_t v) {
    return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31);
}

inline Value scalar(uint32_t field, uint32_t wire, uint64_t bits) {
    Value v;
    put_tag(v.message, field, wire);
    if (wire == 0) {
        put_varint(v.message, bits);
    } else {
        put_le(v.message, bits, wire == 1 ? 8 : 4);
    }
    return v;
}

// The index of `item` in a layer's key or value table, appended when new.
template <typename T>
uint32_t intern(std::vector<T>& table, const T& item) {
    const auto it = std::find(table.begin(), table.end(), item);
    if (it != table.end()) return static_cast<uint32_t>(it - table.begin());
    table.push_back(item);
    return static_cast<uint32_t>(table.size() - 1);
}

inline uint32_t crc32_ieee(const std::vector<uint8_t>& bytes) {
    uint32_t crc = 0xffffffffu;
    for (uint8_t b : bytes) {
        crc ^= b;
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

}

inline Value str(std::string s) {
    Value v;
    detail::put_string(v.message, 1, s);
    return v;
}

inline Value flt(float f) {
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    return detail::scalar(2, 5, bits);
}

inline Value dbl(double d) {
    uint64_t bits = 0;
    std::memcpy(&bits, &d, sizeof bits);
    return detail::scalar(3, 1, bits);
}

inline Value i64(int64_t i) { return detail::scalar(4, 0, static_cast<uint64_t>(i)); }

inline Value u64(uint64_t u) { return detail::scalar(5, 0, u); }

inline Value s64(int64_t i) {
    return detail::scalar(6, 0, (static_cast<uint64_t>(i) << 1) ^ static_cast<uint64_t>(i >> 63));
}

inline Value boolean(bool b) { return detail::scalar(7, 0, b ? 1 : 0); }

// Each layer is written version, name, extent, features, then the key and value tables: the order
// that makes a decoder read the whole layer before it can resolve a feature's tags.
inline std::vector<uint8_t> encode_tile(const std::vector<Layer>& layers) {
    std::vector<uint8_t> tile;
    for (const Layer& layer : layers) {
        std::vector<std::string> keys;
        std::vector<std::vector<uint8_t>> values;
        std::vector<uint8_t> msg;
        detail::put_tag(msg, 15, 0);
        detail::put_varint(msg, layer.version);
        detail::put_string(msg, 1, layer.name);
        detail::put_tag(msg, 5, 0);
        detail::put_varint(msg, layer.extent);
        for (const Feature& f : layer.features) {
            std::vector<uint32_t> tags;
            for (const auto& [key, value] : f.tags) {
                tags.push_back(detail::intern(keys, key));
                tags.push_back(detail::intern(values, value.message));
            }
            std::vector<uint32_t> geometry;
            int32_t cx = 0, cy = 0;  // the cursor persists across rings
            for (const auto& ring : f.rings) {
                for (size_t i = 0; i < ring.size(); ++i) {
                    if (i == 0) geometry.push_back(detail::command(1, 1));  // MoveTo
                    if (i == 1) {
                        const auto line_to = static_cast<uint32_t>(ring.size() - 1);
                        geometry.push_back(detail::command(2, line_to));  // LineTo
                    }
                    geometry.push_back(detail::zigzag32(ring[i][0] - cx));
                    geometry.push_back(detail::zigzag32(ring[i][1] - cy));
                    cx = ring[i][0];
                    cy = ring[i][1];
                }
                geometry.push_back(detail::command(7, 1));  // ClosePath
            }
            std::vector<uint8_t> feature;
            if (f.id) {
                detail::put_tag(feature, 1, 0);
                detail::put_varint(feature, *f.id);
            }
            detail::put_packed(feature, 2, tags);
            detail::put_tag(feature, 3, 0);
            detail::put_varint(feature, f.geom_type);
            detail::put_packed(feature, 4, geometry);
            detail::put_bytes(msg, 2, feature);
        }
        for (const std::string& key : keys) detail::put_string(msg, 3, key);
        for (const std::vector<uint8_t>& value : values) detail::put_bytes(msg, 4, value);
        detail::put_bytes(tile, 3, msg);
    }
    return tile;
}

// A complete gzip stream (RFC 1952) of stored deflate blocks (RFC 1951 3.2.4), so any inflater,
// CesiumUtility::gunzip included, reads it back and verifies its CRC-32 and size.
inline std::vector<uint8_t> gzip_stored(const std::vector<uint8_t>& raw) {
    std::vector<uint8_t> out = {0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
    size_t pos = 0;
    do {
        const size_t n = std::min<size_t>(raw.size() - pos, 0xffff);
        out.push_back(pos + n == raw.size() ? 0x01 : 0x00);  // BFINAL, BTYPE 00 (stored)
        detail::put_le(out, n, 2);
        detail::put_le(out, ~n & 0xffff, 2);  // NLEN
        out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
                   raw.begin() + static_cast<std::ptrdiff_t>(pos + n));
        pos += n;
    } while (pos < raw.size());
    detail::put_le(out, detail::crc32_ieee(raw), 4);
    detail::put_le(out, raw.size() & 0xffffffffu, 4);  // ISIZE
    return out;
}

// A streets-v8-style building feature: height, min_height, extrude, underground, type.
inline Feature building(uint64_t id, std::vector<std::vector<std::array<int32_t, 2>>> rings,
                        double height, double min_height = 0.0, bool extrude = true,
                        bool underground = false, std::string type = "building") {
    Feature f;
    f.id = id;
    f.rings = std::move(rings);
    f.tags = {{"height", dbl(height)},
              {"min_height", dbl(min_height)},
              {"extrude", str(extrude ? "true" : "false")},
              {"underground", str(underground ? "true" : "false")},
              {"type", str(std::move(type))}};
    return f;
}

}
