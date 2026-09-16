// gltf_normals.cpp — see gltf_normals.hpp for the why. Implementation
// summary:
//   1. Split the .glb container into its JSON + BIN chunks (fixed 12-byte
//      header + length-prefixed chunks -- glTF 2.0 spec §3.2, no library
//      needed).
//   2. Parse the JSON chunk with yaml-cpp (JSON is a subset of YAML flow
//      style; verified against the real committed fixture chunks).
//   3. For every primitive missing NORMAL, read POSITION + indices
//      straight out of the BIN chunk bytes and compute an area-weighted
//      flat normal per vertex.
//   4. Append the new accessor/bufferView/bytes to the (mutated) parsed
//      tree, then re-serialize it as real JSON -- yaml-cpp's own Emitter
//      prints unquoted flow style (`key: value`), which is not valid JSON,
//      so write_json() below is a small dedicated writer instead.
#include "gltf_normals.hpp"

#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

namespace mpviz {

namespace {

// ── GLB container (12-byte header + length-prefixed chunks) ─────────────

constexpr uint32_t kGlbMagic = 0x46546C67;       // "glTF"
constexpr uint32_t kChunkTypeJson = 0x4E4F534A;  // "JSON"
constexpr uint32_t kChunkTypeBin = 0x004E4942;   // "BIN\0"

bool read_u32(const std::vector<uint8_t>& bytes, size_t offset, uint32_t& out) {
    if (offset + 4 > bytes.size()) return false;
    std::memcpy(&out, bytes.data() + offset, 4);
    return true;
}

// Splits a .glb blob into its JSON text and (optional) BIN chunk bytes.
// Returns false for anything that isn't a well-formed GLB container with a
// JSON chunk -- every real caller only ever hands this bytes read straight
// from a .glb file or produced by CesiumGltfWriter::writeGlb(), both of
// which always emit one.
bool parse_glb(const std::vector<uint8_t>& bytes, std::string& json_out,
               std::vector<uint8_t>& bin_out) {
    uint32_t magic = 0, version = 0, length = 0;
    if (!read_u32(bytes, 0, magic) || magic != kGlbMagic) return false;
    if (!read_u32(bytes, 4, version) || !read_u32(bytes, 8, length)) return false;
    size_t offset = 12;
    bool have_json = false;
    while (offset + 8 <= bytes.size()) {
        uint32_t chunk_length = 0, chunk_type = 0;
        read_u32(bytes, offset, chunk_length);
        read_u32(bytes, offset + 4, chunk_type);
        const size_t data_start = offset + 8;
        if (data_start + chunk_length > bytes.size()) return false;
        if (chunk_type == kChunkTypeJson && !have_json) {
            json_out.assign(reinterpret_cast<const char*>(bytes.data() + data_start), chunk_length);
            have_json = true;
        } else if (chunk_type == kChunkTypeBin) {
            bin_out.assign(bytes.begin() + static_cast<long>(data_start),
                           bytes.begin() + static_cast<long>(data_start + chunk_length));
        }
        offset = data_start + chunk_length;
    }
    return have_json;
}

std::vector<uint8_t> build_glb(const std::string& json, const std::vector<uint8_t>& bin) {
    auto pad4 = [](size_t n) { return (4 - (n % 4)) % 4; };
    std::string padded_json = json;
    padded_json.append(pad4(padded_json.size()), ' ');  // JSON chunk pads with spaces (spec §3.2)
    std::vector<uint8_t> padded_bin = bin;
    padded_bin.resize(padded_bin.size() + pad4(padded_bin.size()), uint8_t{0});  // BIN pads with zeros

    const uint32_t json_len = static_cast<uint32_t>(padded_json.size());
    const uint32_t bin_len = static_cast<uint32_t>(padded_bin.size());
    const bool have_bin = bin_len > 0;
    const uint32_t total = 12 + 8 + json_len + (have_bin ? 8 + bin_len : 0);

    std::vector<uint8_t> out;
    out.reserve(total);
    auto put_u32 = [&out](uint32_t v) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
        out.insert(out.end(), p, p + 4);
    };
    put_u32(kGlbMagic);
    put_u32(2);  // version
    put_u32(total);
    put_u32(json_len);
    put_u32(kChunkTypeJson);
    out.insert(out.end(), padded_json.begin(), padded_json.end());
    if (have_bin) {
        put_u32(bin_len);
        put_u32(kChunkTypeBin);
        out.insert(out.end(), padded_bin.begin(), padded_bin.end());
    }
    return out;
}

// ── Minimal JSON writer for a YAML::Node parsed FROM JSON ────────────────
// yaml-cpp stores every scalar as its original source string (Scalar()),
// so a value we never touch round-trips byte-for-byte here; we only need
// to re-decide, per scalar, whether it prints quoted (string) or bare
// (number/bool/null). Content-sniffing that (e.g. "does it parse as a
// number") is wrong: glTF's own "asset":{"version":"2.0"} is a STRING that
// happens to look numeric, and a content heuristic emits it bare -- which
// cgltf then rejects (verified: that exact miscompile made every patched
// baked chunk fail "Unable to parse glTF file." against the real Filament
// gltfio loader, not just this file's own yaml-cpp round-trip). The
// correct signal is YAML's own quote-vs-bare distinction, which yaml-cpp
// keeps: Tag() is "!" for anything that was double/single-quoted in the
// source (JSON strings are always quoted) and "?" for a bare/plain token
// (JSON only ever leaves true/false/null/numbers bare) -- verified against
// yaml-cpp 0.8.0's own resolver. A node WE construct fresh (never parsed,
// Tag() == "") falls back to content: safe there because we control
// exactly what we assign (numeric C++ types for numeric fields, and only
// non-numeric-looking strings like "VEC3" for string fields).
bool looks_like_json_number(const std::string& s) {
    if (s.empty()) return false;
    char* end = nullptr;
    errno = 0;
    std::strtod(s.c_str(), &end);
    return end == s.c_str() + s.size();
}

void write_json_string(std::string& out, const std::string& s) {
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            case '\r': out += "\\r"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    out += '"';
}

void write_json(std::string& out, const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Scalar: {
            const std::string& s = node.Scalar();
            const std::string& tag = node.Tag();
            bool bare;
            if (tag == "!") {
                bare = false;  // was quoted in the source JSON -- always a string
            } else if (tag == "?") {
                bare = true;  // was a bare token in the source JSON -- number/bool/null
            } else {
                // Freshly constructed node (never parsed) -- we control the content.
                bare = (s == "true" || s == "false" || looks_like_json_number(s));
            }
            if (bare) {
                out += s;
            } else {
                write_json_string(out, s);
            }
            break;
        }
        case YAML::NodeType::Sequence: {
            out += '[';
            bool first = true;
            for (const YAML::Node& child : node) {
                if (!first) out += ',';
                first = false;
                write_json(out, child);
            }
            out += ']';
            break;
        }
        case YAML::NodeType::Map: {
            out += '{';
            bool first = true;
            for (const auto& kv : node) {
                if (!first) out += ',';
                first = false;
                write_json_string(out, kv.first.Scalar());
                out += ':';
                write_json(out, kv.second);
            }
            out += '}';
            break;
        }
        default:
            out += "null";
    }
}

// ── Reading raw accessor data straight out of the BIN chunk ─────────────

struct AccessorInfo {
    int component_type = 0;
    std::string type;
    size_t count = 0;
    size_t buffer_view = 0;
    size_t byte_offset = 0;
};

std::optional<AccessorInfo> read_accessor(const YAML::Node& accessors, int index) {
    if (!accessors || !accessors.IsSequence() || index < 0 ||
        static_cast<size_t>(index) >= accessors.size()) {
        return std::nullopt;
    }
    const YAML::Node& a = accessors[static_cast<size_t>(index)];
    if (!a["componentType"] || !a["type"] || !a["count"] || !a["bufferView"]) return std::nullopt;
    AccessorInfo info;
    info.component_type = a["componentType"].as<int>();
    info.type = a["type"].as<std::string>();
    info.count = a["count"].as<size_t>();
    info.buffer_view = a["bufferView"].as<size_t>();
    info.byte_offset = a["byteOffset"] ? a["byteOffset"].as<size_t>() : 0;
    return info;
}

struct BufferViewInfo {
    size_t buffer = 0;
    size_t byte_offset = 0;
    bool has_stride = false;
};

std::optional<BufferViewInfo> read_buffer_view(const YAML::Node& buffer_views, size_t index) {
    if (!buffer_views || !buffer_views.IsSequence() || index >= buffer_views.size()) {
        return std::nullopt;
    }
    const YAML::Node& v = buffer_views[index];
    if (!v["byteLength"]) return std::nullopt;
    BufferViewInfo info;
    info.buffer = v["buffer"] ? v["buffer"].as<size_t>() : 0;
    info.byte_offset = v["byteOffset"] ? v["byteOffset"].as<size_t>() : 0;
    info.has_stride = static_cast<bool>(v["byteStride"]);
    return info;
}

// Reads a POSITION-shaped accessor (componentType FLOAT, type VEC3,
// tightly packed, sourced from the single embedded buffer -- buffer index
// 0, which is what every read_buffer_view() caller here requires).
std::optional<std::vector<float>> read_float3_accessor(const YAML::Node& gltf,
                                                        const std::vector<uint8_t>& bin,
                                                        int accessor_index) {
    auto acc = read_accessor(gltf["accessors"], accessor_index);
    if (!acc || acc->component_type != 5126 /* FLOAT */ || acc->type != "VEC3") return std::nullopt;
    auto view = read_buffer_view(gltf["bufferViews"], acc->buffer_view);
    if (!view || view->buffer != 0 || view->has_stride) return std::nullopt;
    const size_t start = view->byte_offset + acc->byte_offset;
    const size_t nbytes = acc->count * 3 * sizeof(float);
    if (start + nbytes > bin.size()) return std::nullopt;
    std::vector<float> out(acc->count * 3);
    std::memcpy(out.data(), bin.data() + start, nbytes);
    return out;
}

// Reads an indices accessor (any unsigned glTF component type) as uint32.
std::optional<std::vector<uint32_t>> read_index_accessor(const YAML::Node& gltf,
                                                          const std::vector<uint8_t>& bin,
                                                          int accessor_index) {
    auto acc = read_accessor(gltf["accessors"], accessor_index);
    if (!acc || acc->type != "SCALAR") return std::nullopt;
    auto view = read_buffer_view(gltf["bufferViews"], acc->buffer_view);
    if (!view || view->buffer != 0 || view->has_stride) return std::nullopt;
    const size_t start = view->byte_offset + acc->byte_offset;
    std::vector<uint32_t> out(acc->count);
    if (acc->component_type == 5121) {  // UNSIGNED_BYTE
        if (start + acc->count > bin.size()) return std::nullopt;
        for (size_t i = 0; i < acc->count; ++i) out[i] = bin[start + i];
    } else if (acc->component_type == 5123) {  // UNSIGNED_SHORT
        if (start + acc->count * 2 > bin.size()) return std::nullopt;
        for (size_t i = 0; i < acc->count; ++i) {
            uint16_t v = 0;
            std::memcpy(&v, bin.data() + start + i * 2, 2);
            out[i] = v;
        }
    } else if (acc->component_type == 5125) {  // UNSIGNED_INT
        if (start + acc->count * 4 > bin.size()) return std::nullopt;
        for (size_t i = 0; i < acc->count; ++i) {
            uint32_t v = 0;
            std::memcpy(&v, bin.data() + start + i * 4, 4);
            out[i] = v;
        }
    } else {
        return std::nullopt;
    }
    return out;
}

// Area-weighted flat normals: one normal per POSITION vertex, accumulated
// (unnormalized, so larger faces weigh more) from every triangle that
// references it and normalized at the end -- glTF 2.0 spec §3.7.2.1's own
// suggested fallback for meshes with no NORMAL attribute.
std::vector<float> compute_flat_normals(const std::vector<float>& positions,
                                        const std::vector<uint32_t>& indices) {
    const size_t vertex_count = positions.size() / 3;
    std::vector<float> normals(vertex_count * 3, 0.0f);
    auto at = [&](uint32_t i, size_t c) { return positions[static_cast<size_t>(i) * 3 + c]; };
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const uint32_t i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];
        const float e1[3] = {at(i1, 0) - at(i0, 0), at(i1, 1) - at(i0, 1), at(i1, 2) - at(i0, 2)};
        const float e2[3] = {at(i2, 0) - at(i0, 0), at(i2, 1) - at(i0, 1), at(i2, 2) - at(i0, 2)};
        const float fn[3] = {
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        };
        for (uint32_t idx : {i0, i1, i2}) {
            normals[static_cast<size_t>(idx) * 3 + 0] += fn[0];
            normals[static_cast<size_t>(idx) * 3 + 1] += fn[1];
            normals[static_cast<size_t>(idx) * 3 + 2] += fn[2];
        }
    }
    for (size_t v = 0; v < vertex_count; ++v) {
        float* n = &normals[v * 3];
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len > 1e-12f) {
            n[0] /= len;
            n[1] /= len;
            n[2] /= len;
        } else {
            // Degenerate (unreferenced or zero-area) vertex: arbitrary but
            // well-defined, never divides by zero downstream.
            n[0] = 0.0f;
            n[1] = 0.0f;
            n[2] = 1.0f;
        }
    }
    return normals;
}

}  // namespace

std::vector<uint8_t> ensure_flat_normals(std::vector<uint8_t> glb_bytes) {
    try {
        std::string json_text;
        std::vector<uint8_t> bin;
        if (!parse_glb(glb_bytes, json_text, bin)) return glb_bytes;  // not a GLB we understand

        YAML::Node gltf = YAML::Load(json_text);
        const YAML::Node meshes = gltf["meshes"];
        if (!meshes || !meshes.IsSequence()) return glb_bytes;
        if (!gltf["buffers"] || gltf["buffers"].size() == 0) return glb_bytes;

        // Two-phase: first collect every primitive we can and know how to
        // fix, without mutating anything -- a read failure partway through
        // then just leaves that one primitive unfixed, never a half-patched
        // file. Second phase (below) does all the mutation.
        struct Fix {
            YAML::Node attributes;  // this primitive's "attributes" map (reference semantics)
            size_t vertex_count = 0;
            std::vector<float> normals;
        };
        std::vector<Fix> fixes;

        for (YAML::Node mesh : meshes) {
            const YAML::Node primitives = mesh["primitives"];
            if (!primitives || !primitives.IsSequence()) continue;
            for (YAML::Node prim : primitives) {
                YAML::Node attributes = prim["attributes"];
                if (!attributes || attributes["NORMAL"]) continue;       // already has one -- untouched
                if (!attributes["POSITION"] || !prim["indices"]) continue;  // nothing to build from
                if (prim["mode"] && prim["mode"].as<int>() != 4) continue;  // only TRIANGLES (glTF default)

                auto positions =
                    read_float3_accessor(gltf, bin, attributes["POSITION"].as<int>());
                auto indices = read_index_accessor(gltf, bin, prim["indices"].as<int>());
                if (!positions || !indices) continue;   // unsupported accessor/bufferView shape
                if (indices->empty() || indices->size() % 3 != 0) continue;

                Fix fix;
                fix.attributes = attributes;
                fix.vertex_count = positions->size() / 3;
                fix.normals = compute_flat_normals(*positions, *indices);
                fixes.push_back(std::move(fix));
            }
        }
        if (fixes.empty()) return glb_bytes;  // every primitive already has NORMAL -- no-op

        YAML::Node accessors = gltf["accessors"];
        YAML::Node buffer_views = gltf["bufferViews"];
        size_t next_accessor = accessors.size();
        size_t next_buffer_view = buffer_views.size();
        std::vector<uint8_t> appended;

        for (Fix& fix : fixes) {
            const size_t byte_length = fix.normals.size() * sizeof(float);

            YAML::Node buffer_view;
            buffer_view["buffer"] = 0;
            buffer_view["byteOffset"] = bin.size() + appended.size();
            buffer_view["byteLength"] = byte_length;
            buffer_views.push_back(buffer_view);

            YAML::Node accessor;
            accessor["componentType"] = 5126;  // FLOAT
            accessor["type"] = "VEC3";
            accessor["count"] = fix.vertex_count;
            accessor["bufferView"] = next_buffer_view;
            accessors.push_back(accessor);

            fix.attributes["NORMAL"] = next_accessor;

            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(fix.normals.data());
            appended.insert(appended.end(), bytes, bytes + byte_length);

            ++next_accessor;
            ++next_buffer_view;
        }
        bin.insert(bin.end(), appended.begin(), appended.end());
        gltf["buffers"][0]["byteLength"] = bin.size();

        std::string new_json;
        write_json(new_json, gltf);
        return build_glb(new_json, bin);
    } catch (const std::exception&) {
        return glb_bytes;  // malformed/unexpected input -- fail safe, same as the caller's own convention
    }
}

}  // namespace mpviz
