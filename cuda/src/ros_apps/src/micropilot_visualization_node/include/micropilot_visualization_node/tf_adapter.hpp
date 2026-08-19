#pragma once
/** @file tf_adapter.hpp
 *  @brief map->base_link TF lookup -> mpviz::EgoState, with EMA-smoothed
 *  finite-difference speed (Epic 1 Task 4 / VM-012).
 *
 *  Deliberately thin: this class owns none of the ROS machinery (no
 *  Buffer/TransformListener of its own) -- VisualizationNode owns those
 *  (Files list, Task 4 Step 10: "own tf2_ros::Buffer/TransformListener")
 *  because constructing a TransformListener needs the node's own
 *  NodeInterfaces. TfAdapter just holds a reference to that Buffer plus the
 *  small bit of state (previous position/timestamp, smoothed speed) that
 *  the finite-difference+EMA math needs across ticks.
 *
 *  Speed source (spec §7, docs/superpowers/specs/2026-08-18-visual-mode-design.md:264,
 *  added after this task's plan text but binding per this review round):
 *  ego speed PREFERS `/robot/feedback/robot_speed_mps` (Float32) when the
 *  node has received at least one sample from it, falling back to the TF
 *  finite-difference+EMA below only until the first sample arrives. The
 *  node (not this class -- same ROS-machinery-lives-on-the-node split as
 *  Buffer/TransformListener above) owns the subscription and forwards
 *  each sample via set_robot_speed_mps().
 */

#include <optional>
#include <string>

#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>

#include "visual_renderer/scene.h"

namespace micropilot::visualization_app
{

class TfAdapter
{
public:
    /// alpha: EMA smoothing factor in (0,1] for the finite-differenced
    /// speed -- smoothed += alpha * (raw - smoothed). Small default (0.2)
    /// kills single-sample TF noise without adding latency-tuning
    /// complexity (a Kalman filter would be gold-plating for a HUD speed
    /// readout, not a control input).
    explicit TfAdapter(tf2_ros::Buffer& buffer, std::string map_frame = "map",
                       std::string base_frame = "base_link", double alpha = 0.2);

    /// Look up map->base_link "now" (tf2::TimePointZero -- latest available),
    /// finite-difference against the previous successful lookup, and
    /// EMA-smooth the result. tf2::TransformException (no transform yet,
    /// covers both LookupException and ExtrapolationException) is caught
    /// here -> returns ego.valid = 0, never throws and never fabricates a
    /// clay box parked at the origin.
    mpviz::EgoState update();

    /// Forward the latest /robot/feedback/robot_speed_mps sample (node-owned
    /// subscription callback calls this). Once set, update() reports this
    /// value instead of the TF-diff/EMA fallback (spec §7).
    void set_robot_speed_mps(double mps) { topic_speed_mps_ = mps; }

private:
    tf2_ros::Buffer& buffer_;
    std::string map_frame_;
    std::string base_frame_;
    double alpha_;

    bool have_prev_{false};
    mpviz::Vec3 prev_pos_{};
    rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
    double smoothed_speed_{0.0};

    // nullopt until the first /robot/feedback/robot_speed_mps sample
    // arrives; the TF-diff/EMA above keeps running underneath regardless
    // (cheap, and keeps it warm in case a later epic adds staleness
    // reversion) but is only reported while this is unset.
    std::optional<double> topic_speed_mps_;
};

}  // namespace micropilot::visualization_app
