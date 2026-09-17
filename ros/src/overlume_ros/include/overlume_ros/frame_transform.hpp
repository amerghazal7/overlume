// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
/** @file frame_transform.hpp
 *  @brief map<-header.frame_id lookup, shared by every Epic 2 adapter
 *  (VM-020). Wraps the tf2_ros::Buffer OverlumeNode already owns --
 *  no new listener, no new thread.
 *
 *  Spec §4.1/§5 ("everything renders in the map frame") is not automatic:
 *  `/sim/ground_truth/boxes` publishes in `base_link`, not `map`, and
 *  copying its coordinates straight through would pile every box at the
 *  map origin while the ego is 100+ m away. Rule, binding for every
 *  adapter: `frame_id == "map"` (or empty) -> identity, zero lookup cost.
 *  Otherwise one lookup per MESSAGE (never per marker). On failure: the
 *  caller drops the whole message and counts it (dropped_no_tf) -- this
 *  class never fabricates an identity transform on lookup failure, because
 *  that renders wrong AND looks plausible.
 */

#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

namespace overlume::ros {

class FrameTransformer {
public:
    // flatten_z (default ON): the HD map is a 2D plane today, so
    // publisher-supplied z (dynamic-object bbox centers,
    // live TF altitude) renders as floating geometry. Every adapter zeroes
    // the z of the points it STORES while this is true (the transform math
    // itself is untouched). Flip to false when the HD-map layer grows real
    // 3D coordinates -- exposed as the node's `flatten_z` parameter.
    explicit FrameTransformer(const tf2_ros::Buffer& buffer, std::string target_frame = "map",
                              bool flatten_z = true)
        : buffer_(buffer), target_frame_(std::move(target_frame)), flatten_z_(flatten_z) {}

    bool flatten_z() const { return flatten_z_; }

    // map <- header.frame_id at the message stamp; falls back to
    // tf2::TimePointZero (latest available) if the exact stamp isn't in the
    // buffer yet (bag playback jitter). Returns false on
    // tf2::TransformException -- caller drops the whole message and counts
    // it (dropped_no_tf), never a partial transform.
    bool lookup(const std_msgs::msg::Header& header, tf2::Transform& out) const;

private:
    const tf2_ros::Buffer& buffer_;
    std::string target_frame_;
    bool flatten_z_ = true;
};

}  // namespace overlume::ros
