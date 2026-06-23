#pragma once
/** @file types.hpp @brief Plain-old-data parameter structs for the reprojector. */

namespace micropilot::rendering
{
/** @brief Pinhole camera: row-major K and R, center t. R columns = (right,down,fwd). */
struct CameraParams
{
    float K[9];
    float R[9];
    float t[3];
    int width;
    int height;
};

/** @brief Bowl proxy surface: flat floor radius R0, parabolic wall k, clamp Rmax. */
struct BowlParams
{
    float R0;
    float k;
    float Rmax;
};
}  // namespace micropilot::rendering
