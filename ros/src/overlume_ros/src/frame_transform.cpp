#include "overlume_ros/frame_transform.hpp"

#include <rclcpp/time.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace overlume::ros
{

bool FrameTransformer::lookup(const std_msgs::msg::Header& header, tf2::Transform& out) const
{
    if (header.frame_id.empty() || header.frame_id == target_frame_) {
        out.setIdentity();
        return true;
    }

    geometry_msgs::msg::TransformStamped msg;
    try {
        // tf2_ros::fromRclcpp() converts to tf2::TimePoint so this selects
        // the timeout-free BufferCore::lookupTransform(TimePoint) overload.
        // Passing rclcpp::Time directly instead selects Buffer's *timeout*
        // overload (buffer.hpp), whose body unconditionally RCLCPP_ERRORs
        // via checkAndErrorDedicatedThreadPresent() when the buffer has no
        // dedicated thread -- true here -- regardless of timeout==0.
        msg = buffer_.lookupTransform(target_frame_, header.frame_id,
                                       tf2_ros::fromRclcpp(rclcpp::Time(header.stamp)));
    } catch (const tf2::TransformException&) {
        try {
            // Bag-playback jitter: the exact stamp may not be in the buffer
            // yet even though the transform itself is available. Fall back
            // to "latest available" rather than dropping a message that a
            // slightly later lookup would have succeeded on.
            msg = buffer_.lookupTransform(target_frame_, header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    tf2::fromMsg(msg.transform, out);
    return true;
}

}  // namespace overlume::ros
