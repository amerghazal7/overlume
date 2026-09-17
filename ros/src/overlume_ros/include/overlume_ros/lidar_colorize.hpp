#pragma once
/** @file lidar_colorize.hpp
 *  @brief VM-094 (unified-engine migration Task 5): camera-colorized lidar
 *  for HYBRID mode.
 *
 *  Pure camera math only -- no ROS types, no rclcpp::Node. The PointCloud2
 *  subscription, the cloud-frame -> rig-frame `pointcloud_transform`
 *  application, and the final rig -> map anchoring by the node's ego pose
 *  all live in overlume_node.cpp (Task 5 Step 3's "wire the node
 *  adapter"), the same "pure math here, ROS wrapper at the call site" split
 *  camera_ingest.hpp's own header comment describes for IngestState/
 *  CameraIngest -- kept out of this function so its tests (test_lidar_colorize.cpp)
 *  stay pure camera math with no ego-pose or rclcpp fixture.
 *
 *  Reuses Task 2's overlume::bowl::ProjectToCameraUv (bowl_projection.hpp) --
 *  the SAME rig-frame projection the bowl bake itself uses, per Decision 3's
 *  "same toolchain, one copy not two" precedent. Colorizes from
 *  camera_ingest.hpp's own last-ingested RGB buffers (CameraIngest::
 *  fill_camera_rgb_buffers()) -- not a second camera ingest path.
 */

#include <cstdint>
#include <vector>

#include "overlume/scene.h"

namespace overlume::ros
{

// For each input point, tries every configured camera (cameras.extrinsics/
// intrinsics/cam_width/cam_height, ascending camera index) via
// overlume::bowl::ProjectToCameraUv (Task 2); the FIRST camera whose UV lands
// inside that camera's image (ProjectToCameraUv's own positive-depth +
// distortion-field + pixel-bounds checks) wins -- a nearest-neighbor pixel
// sample from `camera_rgb_buffers[cam_idx]` (tightly-packed WxHx3 RGB8, the
// SAME buffer camera_ingest.cpp hands to set_camera_frame() -- no second
// ingest, no new subscription). A null entry in `camera_rgb_buffers` (that
// camera has never delivered an image, or is unconfigured) is treated as
// "this camera covers nothing" -- the point tries the next configured
// camera instead of dereferencing null.
//
// **Named, deliberate fidelity regression vs. the CUDA reference**
// (reprojector.hpp:30-36's own doc): the CUDA node colorizes ON DEVICE from
// EVERY already-uploaded camera image, distortion-aware and
// feather-weighted ("same blend as the bowl kernel"); first-match is
// cheaper and reuses zero new library blend code (Decision 5's "zero new
// library rendering code" charter), at the cost of visible seams at
// camera-boundary points the CUDA node blends smoothly.
//
// A point no configured camera covers is DROPPED from the output vector
// entirely -- matching reprojector.hpp:30-36's own "points no camera
// covers are skipped by the splat pass", NOT point_cloud.cpp's `a==0`
// "no producer color, use theme neutral" sentinel (an unrelated case: a
// point some OTHER producer emits with no color data at all). Every point
// this function DOES return therefore always has rgba's alpha byte == 255.
//
// Input points are RIG-frame (the cloud AFTER pointcloud_transform,
// cloud->rig -- Decision 3's frame convention, matching the rig-frame
// `cameras` extrinsics). Output PointCloudPoint positions are the SAME
// rig-frame values; the CALLER transforms rig->map by the node's own ego
// pose before set_scene() (scene.h documents PointCloudPoint's frame as
// map) -- kept out of this function per this file's own header comment.
std::vector<overlume::PointCloudPoint> ColorizeFromCameras(
    const std::vector<overlume::Vec3>& lidar_points_rig_frame, const overlume::BowlConfig& cameras,
    const std::vector<const uint8_t*>& camera_rgb_buffers);

}  // namespace overlume::ros
