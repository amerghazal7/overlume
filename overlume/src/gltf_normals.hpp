// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// gltf_normals.hpp — load-time fix for environment geometry with no vertex
// normals (VM: "buildings render flat" defect). Baked chunks
// (scripts/bake_environment.py's trimesh export) carry POSITION only, no
// NORMAL; gltfio has no built-in fallback for that (verified: no
// flat-normal-generation code in libgltfio_core.a, and glTF spec leaves it
// to the client -- see glTF 2.0 spec §3.7.2.1), so buildings without this
// fix get a degenerate default shading normal and read as unlit.
//
// ensure_flat_normals() is the single load-time hook both ingestion paths
// (BakedEnvironmentSource::update() in environment.cpp,
// StreamRendererResources::prepareInLoadThread() in environment_stream.cpp)
// call right before handing bytes to gltfio's AssetLoader::createAsset().
//
// It is a NO-OP -- returns the input unchanged -- for every primitive that
// already carries a NORMAL attribute, which per the committed
// environment_ion_fixture_*/*.b3dm fixtures is every real streamed tile
// today. It only ever ADDS data (a new accessor + bufferView + normal
// bytes appended to the bin chunk); nothing existing is rewritten in
// place, so geometry that already has normals passes through byte-for-byte.
//
// Computes area-weighted flat/per-face normals (glTF's own suggested
// fallback for missing NORMAL) directly from POSITION + indices, using
// yaml-cpp (already linked into overlume for theme.cpp/
// environment.cpp -- JSON is a subset of YAML flow style, verified against
// the real fixture chunks) to read and round-trip the glTF JSON -- no new
// dependency. Any input this function doesn't recognize (not a GLB,
// unsupported accessor/bufferView shape, parse failure) is returned
// unchanged, same fail-open convention as open_baked_environment_source().
#pragma once

#include <cstdint>
#include <vector>

namespace overlume {

std::vector<uint8_t> ensure_flat_normals(std::vector<uint8_t> glb_bytes);

}  // namespace overlume
