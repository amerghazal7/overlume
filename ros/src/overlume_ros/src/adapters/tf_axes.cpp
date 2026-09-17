#include "overlume_ros/adapters/tf_axes.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace overlume_node
{
namespace
{

constexpr double kAxisLengthM = 0.5;

struct Axis
{
    tf2::Vector3 dir;
    float color[3];
};

const Axis kAxes[3] = {
    {tf2::Vector3(1, 0, 0), {1.0f, 0.0f, 0.0f}},  // X, red
    {tf2::Vector3(0, 1, 0), {0.0f, 1.0f, 0.0f}},  // Y, green
    {tf2::Vector3(0, 0, 1), {0.0f, 0.0f, 1.0f}},  // Z, blue
};

}  // namespace

TfAxesAdapter::TfAxesAdapter(const ProfileRow& row, const tf2_ros::Buffer& buffer,
                             std::string target_frame)
    : row_(row), buffer_(buffer), target_frame_(std::move(target_frame))
{
}

void TfAxesAdapter::fill(overlume::ros::SceneAssembly& out, double sim_time_sec)
{
    point_storage_.clear();
    axis_markers_.clear();

    const std::vector<std::string> frames = buffer_.getAllFrameNames();
    // Fixed capacity FIRST -- every GenericMarker::points below is a raw
    // pointer into point_storage_, which must never reallocate mid-loop
    // (same reasoning as every other adapter's/golden.cpp scene builder's
    // own comment).
    point_storage_.reserve(frames.size() * 6);  // 3 axes * 2 points each
    axis_markers_.reserve(frames.size() * 3);

    for (const auto& frame : frames)
    {
        geometry_msgs::msg::TransformStamped msg;
        try
        {
            // Always "latest available" -- this is a live debug view of
            // the CURRENT tf tree, not a message-stamped lookup (unlike
            // FrameTransformer::lookup(), which has a specific stamp to
            // try first).
            msg = buffer_.lookupTransform(target_frame_, frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException&)
        {
            // A TF tree mid-startup (a frame registered but not yet
            // resolvable against target_frame_) is normal, not an error.
            ++stats_.dropped_no_tf;
            continue;
        }
        tf2::Transform xform;
        tf2::fromMsg(msg.transform, xform);

        const tf2::Vector3 origin = xform * tf2::Vector3(0, 0, 0);
        for (const Axis& axis : kAxes)
        {
            const tf2::Vector3 tip = xform * (axis.dir * kAxisLengthM);
            const size_t offset = point_storage_.size();
            point_storage_.push_back(overlume::Vec3{origin.x(), origin.y(), origin.z()});
            point_storage_.push_back(overlume::Vec3{tip.x(), tip.y(), tip.z()});

            overlume::GenericMarker m{};
            m.primitive = overlume::MarkerPrimitive::LINE_LIST;
            m.points = point_storage_.data() + offset;
            m.point_count = 2;
            m.color[0] = axis.color[0];
            m.color[1] = axis.color[1];
            m.color[2] = axis.color[2];
            m.color[3] = 1.0f;
            // Stamped to THIS call's sim_time_sec, always -- see this
            // file's own header comment: a live-regenerated debug layer
            // must never read as stale.
            m.last_update_sec = sim_time_sec;
            axis_markers_.push_back(m);
        }
    }
    ++stats_.msgs;  // one "tick" of live generation

    for (const auto& m : axis_markers_) out.markers.push_back(m);
}

}  // namespace overlume_node
