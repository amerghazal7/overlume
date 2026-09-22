// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "overlume/scene.h"

namespace overlume::ros {

struct StampedTwist {
    double t;
    double vx, vy;
    double wz;
};

bool twist_at(const std::deque<StampedTwist>& twists, double t, StampedTwist& out);

bool rig_delta(const std::deque<StampedTwist>& twists, double t_from, double t_ref, double& th,
               double& px, double& py);

void compensation_delta_4x4(const std::deque<StampedTwist>& twists, double t_cam, double t_ref,
                            double out_delta_row_major[16]);

constexpr double kOrthonormalizeWarnThresholdRad = 0.05;
overlume::CameraExtrinsics OrthonormalizeExtrinsics(const overlume::CameraExtrinsics& in,
                                                    double* out_max_correction_rad);

class IngestState {
public:
    IngestState(uint32_t camera_count, std::vector<overlume::CameraExtrinsics> extrinsics);

    bool record_camera_info(uint32_t cam_idx, const overlume::CameraIntrinsics& in, uint32_t width,
                            uint32_t height);
    bool all_info_ready() const;
    uint32_t camera_count() const { return camera_count_; }
    const overlume::CameraExtrinsics& extrinsics(uint32_t i) const { return cams_[i].extrinsics; }
    const overlume::CameraIntrinsics& intrinsics(uint32_t i) const { return cams_[i].intrinsics; }
    uint32_t width(uint32_t i) const { return cams_[i].width; }
    uint32_t height(uint32_t i) const { return cams_[i].height; }

    uint64_t record_image_stamp(uint32_t cam_idx, double stamp_sec);
    bool has_stamp(uint32_t cam_idx) const { return cams_[cam_idx].has_stamp; }
    double stamp(uint32_t cam_idx) const { return cams_[cam_idx].stamp; }
    bool newest_stamp(double& out_t_max) const;

    void store_rgb(uint32_t cam_idx, const uint8_t* data, uint32_t width, uint32_t height);
    const uint8_t* rgb(uint32_t cam_idx) const;

private:
    struct PerCam {
        overlume::CameraExtrinsics extrinsics{};
        overlume::CameraIntrinsics intrinsics{};
        uint32_t width = 0, height = 0;
        bool info_ready = false;
        double stamp = 0.0;
        bool has_stamp = false;
        uint64_t frame_id = 0;
        std::vector<uint8_t> rgb;
    };
    uint32_t camera_count_;
    std::vector<PerCam> cams_;
};

class CameraIngest {
public:
    CameraIngest(rclcpp_lifecycle::LifecycleNode* node, uint32_t camera_count,
                 std::vector<std::string> image_topics, std::vector<std::string> info_topics,
                 std::string odom_topic, std::vector<overlume::CameraExtrinsics> extrinsics);

    void set_renderer(overlume::VisualRenderer* r) { renderer_ = r; }
    void set_bowl_enabled(bool enabled) { bowl_enabled_ = enabled; }
    void set_hybrid_enabled(bool enabled) { hybrid_enabled_ = enabled; }
    void set_max_sync_latency(double seconds) { max_sync_latency_ = seconds; }

    bool all_info_ready() const { return state_.all_info_ready(); }
    void fill_bowl_intrinsics(std::vector<overlume::CameraExtrinsics>& out_ext,
                              std::vector<overlume::CameraIntrinsics>& out_in,
                              std::vector<uint32_t>& out_w, std::vector<uint32_t>& out_h) const;
    void fill_camera_rgb_buffers(std::vector<const uint8_t*>& out) const;
    void mark_bowl_config_applied() {
        config_applied_ = true;
        info_dirty_ = false;
    }
    bool config_applied() const { return config_applied_; }
    bool consume_info_dirty();

    void update_motion_deltas();

private:
    rclcpp_lifecycle::LifecycleNode* node_;
    IngestState state_;
    overlume::VisualRenderer* renderer_ = nullptr;
    bool bowl_enabled_ = false;
    bool hybrid_enabled_ = false;
    bool config_applied_ = false;
    bool info_dirty_ = false;
    double max_sync_latency_ = 0.12;

    std::string odom_topic_;
    std::deque<StampedTwist> twists_;
    std::mutex odom_mtx_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> img_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
};

}
