// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Landmark GLBs in the layout Mapbox's 3dtiles endpoint serves (spec section 3), built in the test
// process so no Mapbox bytes are stored. Header-only C++17. stb_image_write's implementation comes
// from golden.cpp, which every test binary compiles.

#include <meshoptimizer.h>

#include "stb_image_write.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace overlume::testing::glb {

struct LandmarkSpec {
    uint64_t id = 1;
    double tx = 4096.0, ty = 4096.0;  // node anchor, z14 tile units (0..8192)
    double lat_deg = 25.0;            // latitude for Mapbox's node scale
    double half_x_m = 10.0, half_y_m = 10.0, height_m = 100.0;  // the building box, local metres
    double footprint_half_m = 12.0;  // the footprint square's half size, local metres
};

struct GlbOptions {
    bool meshopt = true;  // EXT_meshopt_compression, vertex codec v0, on vertex and index views
    // KHR_mesh_quantization: NORMAL normalized byte, TEXCOORD_0 normalized ushort.
    bool quantize = true;
    bool lod_scene = true;             // a second scene "LOD" with low-detail duplicate nodes
    bool occlusion_texture = true;     // a material with a small JPEG occlusionTexture (stb)
    bool bad_matrix_on_first = false;  // the first building node gets a rotated matrix
    bool corrupt_meshopt = false;      // the first compressed vertex stream is garbled
};

namespace detail {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr int kFloat = 5126, kByte = 5120, kUnsignedShort = 5123;

inline std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return buf;
}

inline std::string join(const std::vector<std::string>& items) {
    std::string out = "[";
    for (size_t i = 0; i < items.size(); ++i) out += (i == 0 ? "" : ",") + items[i];
    return out + "]";
}

inline void pad(std::vector<uint8_t>& bytes, size_t alignment) {
    while (bytes.size() % alignment != 0) bytes.push_back(0);
}

inline void put_u32(std::vector<uint8_t>& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

inline std::vector<uint8_t> float_bytes(const std::vector<float>& values) {
    std::vector<uint8_t> out(values.size() * sizeof(float));
    std::memcpy(out.data(), values.data(), out.size());
    return out;
}

// One mesh in local metres (x east, y north, z up).
struct Geometry {
    std::vector<float> positions;  // x, y, z
    std::vector<float> normals;    // x, y, z; empty: no NORMAL
    std::vector<float> uvs;        // u, v; empty: no TEXCOORD_0
    std::vector<uint16_t> indices;
};

// The box (-hx, -hy, 0)..(hx, hy, h): 24 vertices with per-face normals and UVs, 36 indices,
// counter-clockwise seen from outside.
inline Geometry box(double hx, double hy, double h) {
    const double lo[3] = {-hx, -hy, 0.0}, hi[3] = {hx, hy, h};
    static constexpr int kCorner[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    Geometry g;
    for (int a = 0; a < 3; ++a) {
        for (const int sign : {1, -1}) {
            const int u = (a + 1) % 3, v = (a + 2) % 3;
            const auto base = static_cast<uint16_t>(g.positions.size() / 3);
            for (int k = 0; k < 4; ++k) {
                const int* c = kCorner[sign > 0 ? k : 3 - k];
                double p[3];
                p[a] = sign > 0 ? hi[a] : lo[a];
                p[u] = c[0] != 0 ? hi[u] : lo[u];
                p[v] = c[1] != 0 ? hi[v] : lo[v];
                for (int i = 0; i < 3; ++i) {
                    g.positions.push_back(static_cast<float>(p[i]));
                    g.normals.push_back(i == a ? static_cast<float>(sign) : 0.0f);
                }
                g.uvs.push_back(static_cast<float>(c[0]));
                g.uvs.push_back(static_cast<float>(c[1]));
            }
            for (const int i : {0, 1, 2, 0, 2, 3}) {
                g.indices.push_back(static_cast<uint16_t>(base + i));
            }
        }
    }
    return g;
}

// The square (-half, -half)..(half, half) at height z, counter-clockwise seen from above.
inline Geometry square(double half, double z, bool with_normals) {
    static constexpr int kCorner[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    Geometry g;
    for (const auto& c : kCorner) {
        g.positions.push_back(static_cast<float>(c[0] * half));
        g.positions.push_back(static_cast<float>(c[1] * half));
        g.positions.push_back(static_cast<float>(z));
        if (with_normals) g.normals.insert(g.normals.end(), {0.0f, 0.0f, 1.0f});
    }
    g.indices = {0, 1, 2, 0, 2, 3};
    return g;
}

// A 16×16 grey ramp as a real JPEG stream.
inline std::vector<uint8_t> occlusion_jpeg() {
    constexpr int kSide = 16;
    std::vector<uint8_t> pixels(kSide * kSide * 3);
    for (int y = 0; y < kSide; ++y) {
        for (int x = 0; x < kSide; ++x) {
            for (int c = 0; c < 3; ++c) {
                pixels[(y * kSide + x) * 3 + c] = static_cast<uint8_t>(64 + 8 * x + 4 * y);
            }
        }
    }
    std::vector<uint8_t> jpeg;
    stbi_write_jpg_to_func(
        [](void* context, void* data, int size) {
            auto* out = static_cast<std::vector<uint8_t>*>(context);
            const auto* bytes = static_cast<const uint8_t*>(data);
            out->insert(out->end(), bytes, bytes + size);
        },
        &jpeg, kSide, kSide, 3, pixels.data(), 90);
    return jpeg;
}

// Buffer views, accessors and buffer 0 in Mapbox's layout. With meshopt, every view lives in the
// fallback buffer 1, which holds no bytes, and its EXT_meshopt_compression points at the
// compressed stream in buffer 0.
struct Writer {
    GlbOptions opts;
    bool corrupt_next_vertex_stream = false;
    std::vector<uint8_t> bin;   // buffer 0, the BIN chunk
    size_t fallback_bytes = 0;  // buffer 1's byteLength (meshopt only)
    std::vector<std::string> views, accessors;

    int add_view(const std::vector<uint8_t>& raw, const std::vector<uint8_t>& compressed,
                 size_t count, size_t stride, const char* mode, bool vertex) {
        const std::string view_stride = vertex ? ",\"byteStride\":" + std::to_string(stride) : "";
        pad(bin, 4);
        std::string json;
        if (opts.meshopt) {
            fallback_bytes = (fallback_bytes + 3) / 4 * 4;
            json = "{\"buffer\":1,\"byteOffset\":" + std::to_string(fallback_bytes) +
                   ",\"byteLength\":" + std::to_string(raw.size()) + view_stride +
                   ",\"extensions\":{\"EXT_meshopt_compression\":{\"buffer\":0,\"byteOffset\":" +
                   std::to_string(bin.size()) +
                   ",\"byteLength\":" + std::to_string(compressed.size()) +
                   ",\"byteStride\":" + std::to_string(stride) +
                   ",\"count\":" + std::to_string(count) + ",\"mode\":\"" + mode + "\"}}}";
            fallback_bytes += raw.size();
            bin.insert(bin.end(), compressed.begin(), compressed.end());
        } else {
            json = "{\"buffer\":0,\"byteOffset\":" + std::to_string(bin.size()) +
                   ",\"byteLength\":" + std::to_string(raw.size()) + view_stride + "}";
            bin.insert(bin.end(), raw.begin(), raw.end());
        }
        views.push_back(json);
        return static_cast<int>(views.size()) - 1;
    }

    int vertex_view(const std::vector<uint8_t>& raw, size_t count, size_t stride) {
        std::vector<uint8_t> compressed;
        if (opts.meshopt) {
            compressed.resize(meshopt_encodeVertexBufferBound(count, stride));
            compressed.resize(meshopt_encodeVertexBuffer(compressed.data(), compressed.size(),
                                                         raw.data(), count, stride));
            if (corrupt_next_vertex_stream) {
                // No valid header byte: every meshopt decoder rejects the stream.
                std::fill(compressed.begin(), compressed.end(), uint8_t{0xff});
                corrupt_next_vertex_stream = false;
            }
        }
        return add_view(raw, compressed, count, stride, "ATTRIBUTES", true);
    }

    int index_view(const std::vector<uint16_t>& indices, size_t vertex_count) {
        std::vector<uint8_t> raw(indices.size() * 2);
        std::memcpy(raw.data(), indices.data(), raw.size());
        std::vector<uint8_t> compressed;
        if (opts.meshopt) {
            const std::vector<unsigned int> wide(indices.begin(), indices.end());
            compressed.resize(meshopt_encodeIndexBufferBound(wide.size(), vertex_count));
            compressed.resize(meshopt_encodeIndexBuffer(compressed.data(), compressed.size(),
                                                        wide.data(), wide.size()));
        }
        return add_view(raw, compressed, indices.size(), 2, "TRIANGLES", false);
    }

    int image_view(const std::vector<uint8_t>& jpeg) {
        pad(bin, 4);
        views.push_back("{\"buffer\":0,\"byteOffset\":" + std::to_string(bin.size()) +
                        ",\"byteLength\":" + std::to_string(jpeg.size()) + "}");
        bin.insert(bin.end(), jpeg.begin(), jpeg.end());
        return static_cast<int>(views.size()) - 1;
    }

    int accessor(int view, int component_type, const char* type, size_t count, bool normalized,
                 const std::string& bounds = "") {
        accessors.push_back("{\"bufferView\":" + std::to_string(view) +
                            ",\"componentType\":" + std::to_string(component_type) +
                            ",\"count\":" + std::to_string(count) + ",\"type\":\"" + type + "\"" +
                            (normalized ? ",\"normalized\":true" : "") + bounds + "}");
        return static_cast<int>(accessors.size()) - 1;
    }

    // One TRIANGLES primitive: POSITION float with min/max; NORMAL and TEXCOORD_0 quantized as
    // Mapbox's are, or float; u16 indices.
    std::string mesh(const Geometry& g, int material) {
        const size_t n = g.positions.size() / 3;
        std::array<float, 3> lo{g.positions[0], g.positions[1], g.positions[2]}, hi = lo;
        for (size_t i = 0; i < n; ++i) {
            for (size_t c = 0; c < 3; ++c) {
                lo[c] = std::min(lo[c], g.positions[i * 3 + c]);
                hi[c] = std::max(hi[c], g.positions[i * 3 + c]);
            }
        }
        const std::string bounds = ",\"min\":[" + num(lo[0]) + "," + num(lo[1]) + "," + num(lo[2]) +
                                   "],\"max\":[" + num(hi[0]) + "," + num(hi[1]) + "," +
                                   num(hi[2]) + "]";
        std::string attributes =
            "\"POSITION\":" + std::to_string(accessor(vertex_view(float_bytes(g.positions), n, 12),
                                                      kFloat, "VEC3", n, false, bounds));
        if (!g.normals.empty()) {
            int normal = 0;
            if (opts.quantize) {
                std::vector<uint8_t> q(n * 4, 0);  // normalized BYTE x, y, z and a zero pad byte
                for (size_t i = 0; i < n; ++i) {
                    for (size_t c = 0; c < 3; ++c) {
                        q[i * 4 + c] = static_cast<uint8_t>(
                            static_cast<int8_t>(std::lround(g.normals[i * 3 + c] * 127.0f)));
                    }
                }
                normal = accessor(vertex_view(q, n, 4), kByte, "VEC3", n, true);
            } else {
                normal =
                    accessor(vertex_view(float_bytes(g.normals), n, 12), kFloat, "VEC3", n, false);
            }
            attributes += ",\"NORMAL\":" + std::to_string(normal);
        }
        if (!g.uvs.empty()) {
            int uv = 0;
            if (opts.quantize) {
                std::vector<uint8_t> q(n * 4);  // normalized UNSIGNED_SHORT u, v
                for (size_t i = 0; i < n * 2; ++i) {
                    const auto value = static_cast<uint16_t>(std::lround(g.uvs[i] * 65535.0f));
                    std::memcpy(&q[i * 2], &value, 2);
                }
                uv = accessor(vertex_view(q, n, 4), kUnsignedShort, "VEC2", n, true);
            } else {
                uv = accessor(vertex_view(float_bytes(g.uvs), n, 8), kFloat, "VEC2", n, false);
            }
            attributes += ",\"TEXCOORD_0\":" + std::to_string(uv);
        }
        const int indices =
            accessor(index_view(g.indices, n), kUnsignedShort, "SCALAR", g.indices.size(), false);
        return "{\"primitives\":[{\"attributes\":{" + attributes +
               "},\"indices\":" + std::to_string(indices) +
               (material >= 0 ? ",\"material\":" + std::to_string(material) : std::string()) +
               ",\"mode\":4}]}";
    }
};

// Mapbox's node matrix, column-major: diag(s, -s, 1) plus (tx, ty, 0), s from the mean-radius
// sphere at the node's latitude; `rotated` turns it 5 degrees about z, which keeps the diagonal
// within 0.4% of s, so only a rotation check can reject it.
inline std::string node_matrix(const LandmarkSpec& l, bool rotated) {
    const double s = 8192.0 * 16384.0 / (2.0 * kPi * 6371008.8 * std::cos(l.lat_deg * kPi / 180.0));
    const double c = rotated ? std::cos(kPi / 36.0) : 1.0, r = rotated ? std::sin(kPi / 36.0) : 0.0;
    const double m[16] = {s * c, s * r, 0, 0, s * r, -s * c, 0, 0, 0, 0, 1, 0, l.tx, l.ty, 0, 1};
    std::string out = "[";
    for (int k = 0; k < 16; ++k) out += (k == 0 ? "" : ",") + num(m[k]);
    return out + "]";
}

}

inline std::vector<uint8_t> build_landmark_glb(const std::vector<LandmarkSpec>& landmarks,
                                               const GlbOptions& opts) {
    using namespace detail;
    // EXT_meshopt_compression is specified on vertex codec v0, the only one Filament's older
    // meshoptimizer (first on the link line) decodes; meshoptimizer 1.1's encoder defaults to v1.
    meshopt_encodeVertexVersion(0);
    Writer w;
    w.opts = opts;
    w.corrupt_next_vertex_stream = opts.corrupt_meshopt;
    const size_t n = landmarks.size();

    // Meshes and nodes: buildings 0..n-1, footprints n..2n-1, LOD duplicates 2n..3n-1. The first
    // view written is building 0's POSITION, so the BIN chunk opens with a vertex stream.
    std::vector<std::string> meshes, nodes, materials, default_scene, lod_scene;
    for (size_t i = 0; i < n; ++i) {
        const LandmarkSpec& l = landmarks[i];
        meshes.push_back(w.mesh(box(l.half_x_m, l.half_y_m, l.height_m), static_cast<int>(i)));
    }
    for (size_t i = 0; i < n; ++i) {
        meshes.push_back(w.mesh(square(landmarks[i].footprint_half_m, 0.0, false), -1));
    }
    if (opts.lod_scene) {
        for (size_t i = 0; i < n; ++i) {
            const LandmarkSpec& l = landmarks[i];
            meshes.push_back(w.mesh(square(l.half_x_m, l.height_m, true), static_cast<int>(i)));
        }
    }
    for (size_t i = 0; i < n; ++i) {
        const LandmarkSpec& l = landmarks[i];
        const std::string id = "\"" + std::to_string(l.id) + "\"";
        nodes.push_back("{\"mesh\":" + std::to_string(i) +
                        ",\"matrix\":" + node_matrix(l, i == 0 && opts.bad_matrix_on_first) +
                        ",\"extras\":{\"id\":" + id + "}}");
        default_scene.push_back(std::to_string(i));
    }
    for (size_t i = 0; i < n; ++i) {
        const LandmarkSpec& l = landmarks[i];
        nodes.push_back("{\"mesh\":" + std::to_string(n + i) +
                        ",\"matrix\":" + node_matrix(l, false) +
                        ",\"extras\":{\"mapbox:footprint:id\":\"" + std::to_string(l.id) + "\"}}");
        default_scene.push_back(std::to_string(n + i));
    }
    if (opts.lod_scene) {
        for (size_t i = 0; i < n; ++i) {
            const LandmarkSpec& l = landmarks[i];
            nodes.push_back("{\"mesh\":" + std::to_string(2 * n + i) +
                            ",\"matrix\":" + node_matrix(l, false) + ",\"extras\":{\"id\":\"" +
                            std::to_string(l.id) + "\"}}");
            lod_scene.push_back(std::to_string(2 * n + i));
        }
    }
    for (size_t i = 0; i < n; ++i) {
        materials.push_back(
            std::string("{\"doubleSided\":true,\"pbrMetallicRoughness\":{\"metallicFactor\":0}") +
            (opts.occlusion_texture ? ",\"occlusionTexture\":{\"index\":0,\"extensions\":{"
                                      "\"KHR_texture_transform\":{\"scale\":[1,1]}}}"
                                    : "") +
            "}");
    }

    std::vector<std::string> used, required;
    if (opts.meshopt) {
        used.push_back("\"EXT_meshopt_compression\"");
        required.push_back("\"EXT_meshopt_compression\"");
    }
    if (opts.quantize) {
        used.push_back("\"KHR_mesh_quantization\"");
        required.push_back("\"KHR_mesh_quantization\"");
    }
    if (opts.occlusion_texture) used.push_back("\"KHR_texture_transform\"");

    // Mapbox writes no top-level "scene".
    std::string json = "{\"asset\":{\"version\":\"2.0\",\"generator\":\"overlume test fixture\"}";
    if (!used.empty()) json += ",\"extensionsUsed\":" + join(used);
    if (!required.empty()) json += ",\"extensionsRequired\":" + join(required);
    json += ",\"scenes\":[{\"name\":\"Default Scene\",\"nodes\":" + join(default_scene) + "}";
    if (opts.lod_scene) json += ",{\"name\":\"LOD\",\"nodes\":" + join(lod_scene) + "}";
    json += "],\"nodes\":" + join(nodes) + ",\"meshes\":" + join(meshes) +
            ",\"materials\":" + join(materials);
    if (opts.occlusion_texture) {
        // The JPEG goes last in the BIN chunk, after every compressed stream.
        const int view = w.image_view(occlusion_jpeg());
        json +=
            ",\"textures\":[{\"sampler\":0,\"source\":0}],\"samplers\":[{\"magFilter\":9729,"
            "\"minFilter\":9987}],\"images\":[{\"bufferView\":" +
            std::to_string(view) + ",\"mimeType\":\"image/jpeg\"}]";
    }
    json += ",\"accessors\":" + join(w.accessors) + ",\"bufferViews\":" + join(w.views) +
            ",\"buffers\":[{\"byteLength\":" + std::to_string(w.bin.size()) + "}";
    if (opts.meshopt) {
        json += ",{\"byteLength\":" + std::to_string(w.fallback_bytes) +
                ",\"extensions\":{\"EXT_meshopt_compression\":{\"fallback\":true}}}";
    }
    json += "]}";

    std::vector<uint8_t> bin = w.bin;
    pad(bin, 4);
    while (json.size() % 4 != 0) json.push_back(' ');
    std::vector<uint8_t> out;
    put_u32(out, 0x46546C67);  // "glTF"
    put_u32(out, 2);
    put_u32(out, static_cast<uint32_t>(12 + 8 + json.size() + 8 + bin.size()));
    put_u32(out, static_cast<uint32_t>(json.size()));
    put_u32(out, 0x4E4F534A);  // "JSON"
    out.insert(out.end(), json.begin(), json.end());
    put_u32(out, static_cast<uint32_t>(bin.size()));
    put_u32(out, 0x004E4942);  // "BIN"
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}

}
