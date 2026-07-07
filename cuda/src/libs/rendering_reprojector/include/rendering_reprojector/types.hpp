#pragma once
/** @file types.hpp @brief Plain-old-data parameter structs for the reprojector. */

namespace micropilot::rendering
{
/** @brief Pinhole camera: row-major K and R, center t. R columns = (right,down,fwd).
 *  dist = plumb_bob [k1 k2 p1 p2 k3] applied when projecting INTO this camera
 *  (real lenses publish raw/distorted images); all-zero = pure pinhole (sim). */
struct CameraParams
{
    float K[9];
    float R[9];
    float t[3];
    int width;
    int height;
    float dist[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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
