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
    // Seam crossfade width in source-image pixels: a camera's weight fades to 0
    // over this distance from its image border. Size it to a good fraction of
    // the camera-overlap width or the seam shows as a hard line.
    float feather_margin = 30.0f;
    // Fill surface-hit pixels that no camera covers (the blind ring around the
    // robot) by propagating surrounding scene colors, instead of leaving them
    // invalid for the consumer's sky fill. Runs before the robot proxy overlay.
    bool fill_blind_zone = false;
};
}  // namespace micropilot::rendering
