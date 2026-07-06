#pragma once
/** @file mesh_loader.hpp @brief OBJ+MTL loader for the robot proxy mesh. */

#include <cstddef>
#include <string>
#include <vector>

namespace micropilot::rendering
{
/** Triangle soup with baked per-triangle colors, ready for Reprojector::upload_robot_mesh. */
struct RobotMesh
{
    std::vector<float> verts;   ///< n_tris * 9: three rig-frame xyz per triangle
    std::vector<float> cols;    ///< n_tris * 3: one baked RGB per triangle
    std::size_t n_tris = 0;
};

/**
 * Load a Wavefront OBJ (+ its MTL, resolved relative to the OBJ) into a RobotMesh.
 *
 * - transform: 12 floats [R(9 row-major) | t(3)], OBJ->rig; applied to positions,
 *   R alone to normals.
 * - n-gon faces are fan-triangulated; negative OBJ indices are supported.
 * - Color = material Kd (emissive lights: max(Kd, clamp(Ke,0,1))) shaded with a
 *   fixed lambert term |dot(n, L)| (winding-robust). Missing MTL/material -> gray.
 *
 * Throws std::runtime_error if the OBJ cannot be opened or contains no faces.
 */
RobotMesh load_obj_mesh(const std::string& obj_path, const float* transform);
}  // namespace micropilot::rendering
