// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_landmarks.hpp"

#include "mapbox_test_hooks.hpp"

#include <CesiumGltf/AccessorUtility.h>
#include <CesiumGltf/AccessorView.h>
#include <CesiumGltf/ExtensionBufferExtMeshoptCompression.h>
#include <CesiumGltf/ExtensionBufferViewExtMeshoptCompression.h>
#include <CesiumGltfContent/GltfUtilities.h>
#include <CesiumGltfReader/GltfReader.h>

#include <glm/vec3.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <type_traits>
#include <variant>

namespace overlume::mapbox {

namespace {

using CesiumGltfContent::GltfUtilities;

constexpr char kFootprintIdKey[] = "mapbox:footprint:id";

// GL JS's rule: the scene named "Default Scene", else `scene`, else the first.
const CesiumGltf::Scene* choose_scene(const CesiumGltf::Model& model) {
    for (const CesiumGltf::Scene& scene : model.scenes) {
        if (scene.name == "Default Scene") return &scene;
    }
    const CesiumGltf::Scene* pointed = CesiumGltf::Model::getSafe(&model.scenes, model.scene);
    return pointed != nullptr ? pointed : CesiumGltf::Model::getSafe(&model.scenes, 0);
}

// Appends the footprint mesh's triangles, mapped by `to_frame`, as tile-frame x, y.
void append_footprint_triangles(const CesiumGltf::Model& model, const CesiumGltf::Mesh& mesh,
                                const Mat4d& to_frame, std::vector<std::array<double, 6>>& out) {
    for (const CesiumGltf::MeshPrimitive& prim : mesh.primitives) {
        const auto position = prim.attributes.find("POSITION");
        if (prim.mode != CesiumGltf::MeshPrimitive::Mode::TRIANGLES ||
            position == prim.attributes.end()) {
            continue;
        }
        const CesiumGltf::AccessorView<glm::vec3> positions(model, position->second);
        if (positions.status() != CesiumGltf::AccessorViewStatus::Valid) continue;
        std::vector<int64_t> corners;
        std::visit(
            [&](const auto& indices) {
                if constexpr (std::is_same_v<std::decay_t<decltype(indices)>, std::monostate>) {
                    for (int64_t i = 0; i < positions.size(); ++i) corners.push_back(i);
                } else if (indices.status() == CesiumGltf::AccessorViewStatus::Valid) {
                    for (int64_t i = 0; i < indices.size(); ++i) {
                        corners.push_back(static_cast<int64_t>(indices[i]));
                    }
                }
            },
            CesiumGltf::getIndexAccessorView(model, prim));
        for (size_t i = 0; i + 2 < corners.size(); i += 3) {
            std::array<double, 6> triangle{};
            bool valid = true;
            for (size_t k = 0; k < 3; ++k) {
                const int64_t c = corners[i + k];
                if (c < 0 || c >= positions.size()) {
                    valid = false;
                    break;
                }
                const glm::vec3& p = positions[c];
                triangle[2 * k] =
                    to_frame[0] * p.x + to_frame[4] * p.y + to_frame[8] * p.z + to_frame[12];
                triangle[2 * k + 1] =
                    to_frame[1] * p.x + to_frame[5] * p.y + to_frame[9] * p.z + to_frame[13];
            }
            if (valid) out.push_back(triangle);
        }
    }
}

// What the pruned model must hold before compactBuffers, which erases byte ranges unchecked (its
// asserts are compiled out), and before gltfio, whose cgltf rejects the whole tile's GLB over one
// dangling index. Clay mode's texture pruning is where this reader could leave one; other indices
// are not checked.
const char* kept_model_problem(const CesiumGltf::Model& model) {
    for (const CesiumGltf::Buffer& buffer : model.buffers) {
        if (buffer.byteLength != static_cast<int64_t>(buffer.cesium.data.size())) {
            return "a kept buffer does not hold its bytes";
        }
    }
    for (const CesiumGltf::BufferView& view : model.bufferViews) {
        const CesiumGltf::Buffer* buffer = CesiumGltf::Model::getSafe(&model.buffers, view.buffer);
        if (buffer == nullptr || view.byteOffset < 0 || view.byteLength < 0 ||
            view.byteOffset > buffer->byteLength ||
            view.byteLength > buffer->byteLength - view.byteOffset) {
            return "a kept bufferView lies outside its buffer";
        }
    }
    const auto missing = [&model](const auto& slot) {
        return slot.has_value() &&
               CesiumGltf::Model::getSafe(&model.textures, slot->index) == nullptr;
    };
    for (const CesiumGltf::Material& m : model.materials) {
        const auto& pbr = m.pbrMetallicRoughness;
        if (missing(m.occlusionTexture) || missing(m.normalTexture) || missing(m.emissiveTexture) ||
            (pbr && (missing(pbr->baseColorTexture) || missing(pbr->metallicRoughnessTexture)))) {
            return "a material names a texture the model does not have";
        }
    }
    return nullptr;
}

}

LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame,
                                bool keep_materials) {
    LandmarkTile out;
    // A 200 can still carry an HTML or JSON body; cesium would read JSON as an empty glTF.
    if (glb.size() < 4 || std::memcmp(glb.data(), "glTF", 4) != 0) {
        out.error = "not a GLB (no glTF magic)";
        return out;
    }
    CesiumGltfReader::GltfReaderOptions options;
    // Filament decodes the JPEGs itself; cesium's decoded copy would be thrown away.
    options.decodeEmbeddedImages = false;
    static const CesiumGltfReader::GltfReader reader;
    CesiumGltfReader::GltfReaderResult read = reader.readGltf(glb, options);
    if (!read.model || !read.errors.empty()) {
        out.error = read.errors.empty() ? std::string("GLB without a model") : read.errors.front();
        return out;
    }
    CesiumGltf::Model& model = *read.model;
    // cesium drops the extension names even when a stream fails to decode; the failed view keeps
    // its extension and still points at the fallback buffer, which has no bytes.
    for (const CesiumGltf::BufferView& view : model.bufferViews) {
        if (view.hasExtension<CesiumGltf::ExtensionBufferViewExtMeshoptCompression>()) {
            out.error = "EXT_meshopt_compression: a compressed stream did not decode";
            return out;
        }
    }
    // cesium's decoders take the extension they handle off extensionsRequired; a name left is one
    // it does not decode, such as KHR_meshopt_compression, whose views would still point into the
    // empty fallback buffer.
    if (!model.extensionsRequired.empty()) {
        out.error =
            "GLB requires " + model.extensionsRequired.front() + ", which the reader cannot decode";
        return out;
    }

    const CesiumGltf::Scene* scene = choose_scene(model);
    const std::vector<int32_t> roots = scene != nullptr ? scene->nodes : std::vector<int32_t>{};
    std::vector<CesiumGltf::Node> buildings;
    // Only scene roots are walked: Mapbox's landmark nodes have no children.
    for (const int32_t index : roots) {
        const CesiumGltf::Node* node = CesiumGltf::Model::getSafe(&model.nodes, index);
        const CesiumGltf::Mesh* mesh =
            node != nullptr ? CesiumGltf::Model::getSafe(&model.meshes, node->mesh) : nullptr;
        if (mesh == nullptr) continue;
        std::optional<Mat4d> to_frame;
        if (node->matrix.size() == 16) {
            Mat4d m{};
            std::copy(node->matrix.begin(), node->matrix.end(), m.begin());
            to_frame = landmark_node_to_frame(frame, m);
        }
        if (!to_frame) {
            ++out.nodes_skipped;
            continue;
        }
        if (node->extras.count(kFootprintIdKey) != 0) {
            append_footprint_triangles(model, *mesh, *to_frame, out.footprint_triangles);
            continue;
        }
        CesiumGltf::Node& kept = buildings.emplace_back();
        kept.mesh = node->mesh;
        kept.matrix.assign(to_frame->begin(), to_frame->end());
    }
    out.buildings_kept = static_cast<uint32_t>(buildings.size());

    // One scene of the kept buildings, then everything only the dropped nodes used goes.
    model.nodes = std::move(buildings);
    model.scenes.assign(1, CesiumGltf::Scene{});
    model.scenes[0].name = "Default Scene";
    for (int32_t i = 0; i < static_cast<int32_t>(model.nodes.size()); ++i) {
        model.scenes[0].nodes.push_back(i);
    }
    model.scene = 0;
    model.skins.clear();  // skins and animations name nodes by index
    model.animations.clear();
    if (!keep_materials) {
        // Clay replaces every material at upload. removeUnusedTextures does not visit
        // occlusionTexture, Mapbox's only texture slot, so the references go first (and original
        // mode, which keeps them, must not call it).
        for (CesiumGltf::Material& material : model.materials) {
            material.occlusionTexture.reset();
            material.normalTexture.reset();
            material.emissiveTexture.reset();
            if (material.pbrMetallicRoughness) {
                material.pbrMetallicRoughness->baseColorTexture.reset();
                material.pbrMetallicRoughness->metallicRoughnessTexture.reset();
            }
        }
        GltfUtilities::removeUnusedTextures(model);
        GltfUtilities::removeUnusedSamplers(model);
        GltfUtilities::removeUnusedImages(model);
    }
    GltfUtilities::removeUnusedMeshes(model);
    GltfUtilities::removeUnusedAccessors(model);
    GltfUtilities::removeUnusedBufferViews(model);
    // Before compactBuffers: the meshopt fallback buffer has a byteLength but no bytes, which
    // compactBuffer cannot shrink.
    GltfUtilities::removeUnusedBuffers(model);
    if (const char* problem = kept_model_problem(model)) return LandmarkTile{.error = problem};
    GltfUtilities::compactBuffers(model);

    out.model = std::move(model);
    out.ok = true;
    return out;
}

namespace {

// The edge function of (ax, ay) -> (bx, by) at (px, py): zero on the edge. A compiler that fuses
// a * b - c * d into one multiply-add keeps the rounding of one product only, so a point exactly
// on an edge came out as +-1 ulp instead of 0 on aarch64, and the diagonal of a footprint split
// in two counted as outside both halves; anything within 1e-12 of the products' size is zero.
double edge_function(double ax, double ay, double bx, double by, double px, double py) {
    const double lhs = (bx - ax) * (py - ay);
    const double rhs = (by - ay) * (px - ax);
    const double e = lhs - rhs;
    return std::abs(e) <= 1e-12 * (std::abs(lhs) + std::abs(rhs)) ? 0.0 : e;
}

}  // namespace

bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y) {
    for (const std::array<double, 6>& t : triangles) {
        const double area2 = edge_function(t[0], t[1], t[2], t[3], t[4], t[5]);
        if (!(std::abs(area2) > 0.0)) continue;  // a degenerate triangle covers nothing
        // Edge functions of either winding; zero on an edge, so a point on the diagonal of a
        // footprint split in two counts as inside.
        const double e0 = edge_function(t[0], t[1], t[2], t[3], x, y);
        const double e1 = edge_function(t[2], t[3], t[4], t[5], x, y);
        const double e2 = edge_function(t[4], t[5], t[0], t[1], x, y);
        if ((e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) || (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0)) {
            return true;
        }
    }
    return false;
}

}

namespace overlume::testing {

MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14,
                                               uint32_t y14, double origin_height_m,
                                               bool keep_materials) {
    namespace mb = overlume::mapbox;
    const mb::TileFrame frame = mb::make_tile_frame({mb::kLandmarkZoom, x14, y14}, origin_height_m);
    const mb::LandmarkTile tile =
        mb::read_landmark_tile(std::as_bytes(std::span(glb)), frame, keep_materials);
    const CesiumGltf::Model& m = tile.model;
    const auto count = [](const auto& items) { return static_cast<uint32_t>(items.size()); };
    MapboxLandmarkProbe p;
    p.ok = tile.ok;
    p.error = tile.error;
    p.buildings_kept = tile.buildings_kept;
    p.nodes_skipped = tile.nodes_skipped;
    p.scenes = count(m.scenes);
    p.nodes = count(m.nodes);
    p.meshes = count(m.meshes);
    p.accessors = count(m.accessors);
    p.buffer_views = count(m.bufferViews);
    p.buffers = count(m.buffers);
    p.images = count(m.images);
    p.textures = count(m.textures);
    p.samplers = count(m.samplers);
    p.materials = count(m.materials);
    p.footprint_triangles = count(tile.footprint_triangles);
    p.meshopt_extension_left = m.isExtensionUsed("EXT_meshopt_compression") ||
                               m.isExtensionRequired("EXT_meshopt_compression");
    for (const CesiumGltf::Buffer& buffer : m.buffers) {
        p.buffer_bytes += buffer.cesium.data.size();
        p.meshopt_extension_left =
            p.meshopt_extension_left ||
            buffer.hasExtension<CesiumGltf::ExtensionBufferExtMeshoptCompression>();
    }
    for (const CesiumGltf::BufferView& view : m.bufferViews) {
        p.meshopt_extension_left =
            p.meshopt_extension_left ||
            view.hasExtension<CesiumGltf::ExtensionBufferViewExtMeshoptCompression>();
    }
    for (const CesiumGltf::Node& node : m.nodes) {
        std::array<double, 16> matrix{};
        std::copy_n(node.matrix.begin(), std::min<size_t>(16, node.matrix.size()), matrix.begin());
        p.node_matrices.push_back(matrix);
    }
    return p;
}

bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles,
                                       double x, double y) {
    return overlume::mapbox::inside_any_footprint(triangles, x, y);
}

}
