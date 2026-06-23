/** @file rendering_node.cpp
 *  @brief ROS2 LifecycleNode wrapping the micropilot_rendering CUDA reprojector.
 */

#include "micropilot_rendering_node/rendering_node.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace micropilot::rendering_app
{

RenderingNode::RenderingNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("rendering_node", options)
{
    RCLCPP_INFO(get_logger(), "RenderingNode constructed — awaiting configure transition.");
}

// ── Lifecycle: on_configure ──────────────────────────────────────────────────
RenderingNode::CallbackReturn RenderingNode::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_configure() called.");

    // ── declare + read parameters ────────────────────────────────────────────
    n_cameras_ = declare_parameter<int>("n_cameras", 4);
    out_width_ = declare_parameter<int>("out_width", 640);
    out_height_ = declare_parameter<int>("out_height", 480);

    bowl_.R0 = static_cast<float>(declare_parameter<double>("bowl_R0", 6.0));
    bowl_.k = static_cast<float>(declare_parameter<double>("bowl_k", 0.08));
    bowl_.Rmax = static_cast<float>(declare_parameter<double>("bowl_Rmax", 20.0));

    // Virtual camera: 12-float row-major [R(3x3 row-major) | t(3)].
    // Default: identity rotation, camera 4 m above origin looking down.
    auto vcam_vec = declare_parameter<std::vector<double>>(
        "virtual_pose",
        {1, 0, 0,   // R row 0
         0, 0, -1,  // R row 1  (y maps to -z → looking down)
         0, 1, 0,   // R row 2
         0.0, -4.0, 0.0});  // t (camera 4 m above ground in rig frame)

    if (vcam_vec.size() != 12)
    {
        RCLCPP_ERROR(get_logger(), "virtual_pose must be 12 floats [R(9) | t(3)], got %zu",
                     vcam_vec.size());
        return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < 9; ++i) vcam_.R[i] = static_cast<float>(vcam_vec[i]);
    for (int i = 0; i < 3; ++i) vcam_.t[i] = static_cast<float>(vcam_vec[9 + i]);
    vcam_.width = out_width_;
    vcam_.height = out_height_;

    // Virtual camera intrinsics — simple pinhole, 60° VFOV
    float fy = (out_height_ / 2.0f) / std::tan(30.0f * M_PI / 180.0f);
    float fx = fy;
    vcam_.K[0] = fx;  vcam_.K[1] = 0;   vcam_.K[2] = out_width_ / 2.0f;
    vcam_.K[3] = 0;   vcam_.K[4] = fy;  vcam_.K[5] = out_height_ / 2.0f;
    vcam_.K[6] = 0;   vcam_.K[7] = 0;   vcam_.K[8] = 1;

    // ── camera extrinsics from parameter ────────────────────────────────────
    // Flat list of N*12 floats: per-camera [R(9 row-major) | t(3)] in rig frame.
    // Note: in production, replace with a tf2 lookup:
    //   tf2_ros::Buffer tf_buffer(get_clock());
    //   auto tf = tf_buffer.lookupTransform(rig_frame, cam_frame, tf2::TimePointZero);
    //   // fill cam_params_[i].R / .t from tf.transform.rotation / .translation
    std::vector<double> ext_default;
    // Build a default ring of 4 cameras around z-axis (identity rotation = forward)
    for (int i = 0; i < n_cameras_; ++i)
    {
        float angle = 2.0f * M_PI * i / n_cameras_;
        // Camera i: positioned on ring, looking outward
        // R = rotation so camera looks outward at angle
        float ca = std::cos(angle), sa = std::sin(angle);
        // Row-major: right=(sin,cos,0), down=(0,0,1), fwd=(cos,-sin,0) ... simplified identity
        std::vector<double> r = {
            (double)ca, (double)(-sa), 0,
            0,          0,             -1,
            (double)sa, (double)ca,    0,
            0.55 * ca, 0.55 * sa, 0.55};  // t: radius 0.55, height 0.55
        ext_default.insert(ext_default.end(), r.begin(), r.end());
    }

    auto ext_vec = declare_parameter<std::vector<double>>("camera_extrinsics", ext_default);

    if (static_cast<int>(ext_vec.size()) != n_cameras_ * 12)
    {
        RCLCPP_ERROR(get_logger(),
                     "camera_extrinsics must have n_cameras*12 = %d floats, got %zu",
                     n_cameras_ * 12, ext_vec.size());
        return CallbackReturn::FAILURE;
    }

    cam_params_.resize(n_cameras_);
    for (int i = 0; i < n_cameras_; ++i)
    {
        auto& cp = cam_params_[i];
        int base = i * 12;
        for (int j = 0; j < 9; ++j) cp.R[j] = static_cast<float>(ext_vec[base + j]);
        for (int j = 0; j < 3; ++j) cp.t[j] = static_cast<float>(ext_vec[base + 9 + j]);
        // width/height filled when CameraInfo arrives; default to out size
        cp.width = out_width_;
        cp.height = out_height_;
        // Default intrinsics (overwritten by CameraInfo subscription)
        cp.K[0] = fx; cp.K[1] = 0;  cp.K[2] = out_width_ / 2.0f;
        cp.K[3] = 0;  cp.K[4] = fy; cp.K[5] = out_height_ / 2.0f;
        cp.K[6] = 0;  cp.K[7] = 0;  cp.K[8] = 1;
    }

    // ── build reprojector ────────────────────────────────────────────────────
    try
    {
        reprojector_ = std::make_unique<micropilot::rendering::Reprojector>(out_width_, out_height_);
        reprojector_->set_cameras(cam_params_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Reprojector init failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    // ── publishers (created in configure, activated in on_activate) ──────────
    pub_image_ = create_publisher<sensor_msgs::msg::Image>("/rendering/image", 1);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/rendering/camera_info", 1);

    // ── explicit topic names (optional) ─────────────────────────────────────
    // If provided (size == n_cameras), the node subscribes to these exact topics
    // instead of the default "/camera/camN/..." pattern — used to point at real
    // camera drivers (e.g. "/fl_camera/raw_images" + "/fl_camera/camera_info").
    image_topics_ = declare_parameter<std::vector<std::string>>(
        "image_topics", std::vector<std::string>{});
    info_topics_ = declare_parameter<std::vector<std::string>>(
        "info_topics", std::vector<std::string>{});
    if ((!image_topics_.empty() && static_cast<int>(image_topics_.size()) != n_cameras_) ||
        (!info_topics_.empty() && static_cast<int>(info_topics_.size()) != n_cameras_))
    {
        RCLCPP_ERROR(get_logger(),
                     "image_topics/info_topics, when set, must have n_cameras=%d entries",
                     n_cameras_);
        return CallbackReturn::FAILURE;
    }

    // ── per-camera state ─────────────────────────────────────────────────────
    per_cam_.resize(n_cameras_);
    img_dirty_.assign(n_cameras_, false);

    RCLCPP_INFO(get_logger(),
                "on_configure() succeeded. n_cameras=%d out=%dx%d bowl(R0=%.2f k=%.3f Rmax=%.1f)",
                n_cameras_, out_width_, out_height_, bowl_.R0, bowl_.k, bowl_.Rmax);
    return CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_activate ───────────────────────────────────────────────────
RenderingNode::CallbackReturn RenderingNode::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_activate() called.");
    pub_image_->on_activate();
    pub_info_->on_activate();

    // ── image subscriptions ──────────────────────────────────────────────────
    img_subs_.resize(n_cameras_);
    info_subs_.resize(n_cameras_);

    for (int i = 0; i < n_cameras_; ++i)
    {
        std::string img_topic = image_topics_.empty()
                                    ? "/camera/cam" + std::to_string(i) + "/image_raw"
                                    : image_topics_[i];
        std::string info_topic = info_topics_.empty()
                                     ? "/camera/cam" + std::to_string(i) + "/camera_info"
                                     : info_topics_[i];

        // Capture index by value
        img_subs_[i] = create_subscription<sensor_msgs::msg::Image>(
            img_topic, rclcpp::SensorDataQoS(),
            [this, i](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                cv_bridge::CvImagePtr cv_img;
                try
                {
                    cv_img = cv_bridge::toCvCopy(msg, "rgb8");
                }
                catch (const cv_bridge::Exception& e)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "cv_bridge error cam %d: %s", i, e.what());
                    return;
                }
                // Convert uint8 rgb8 → float32, normalize [0,1]
                cv::Mat float_img;
                cv_img->image.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

                std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);
                per_cam_[i].image = float_img;
                img_dirty_[i] = true;
            });

        info_subs_[i] = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, rclcpp::SensorDataQoS(),
            [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                // I3: guard cam_params_ width/height/K writes with the per-camera mutex.
                std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);
                auto& cp = cam_params_[i];
                cp.width = static_cast<int>(msg->width);
                cp.height = static_cast<int>(msg->height);
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        cp.K[r * 3 + c] = static_cast<float>(msg->k[r * 3 + c]);
                per_cam_[i].info_ready = true;
            });
    }

    // ── render timer @ 30 Hz ─────────────────────────────────────────────────
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(33ms, [this]() { timer_callback(); });

    RCLCPP_INFO(get_logger(), "on_activate() succeeded. Subscribed to %d cameras.", n_cameras_);
    return CallbackReturn::SUCCESS;
}

// ── Timer callback ───────────────────────────────────────────────────────────
void RenderingNode::timer_callback()
{
    // Check all cameras have an image
    for (int i = 0; i < n_cameras_; ++i)
    {
        std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);
        if (!per_cam_[i].image.has_value()) return;
    }

    // Build NHWC float buffer (N, H, W, C=3) — use first camera's size.
    // Invariant: every image plane uploaded is (H, W); CamDev w/h/K must also
    // reflect (W, H). Each cam's K is rescaled from its CameraInfo resolution
    // to (W, H) so the kernel bilinear-sample bounds and focal lengths agree.
    int H, W;
    {
        std::lock_guard<std::mutex> lk(*per_cam_[0].mtx);
        H = per_cam_[0].image->rows;
        W = per_cam_[0].image->cols;
    }

    std::vector<float> nhwc(n_cameras_ * H * W * 3);
    // I1/I2: rescaled camera params — only width/height/K change; R/t are taken
    // from the stored cam_params_ extrinsics and are NOT mutated here.
    std::vector<micropilot::rendering::CameraParams> scaled_params(n_cameras_);
    bool any_dirty = false;

    for (int i = 0; i < n_cameras_; ++i)
    {
        // I3: hold the per-camera lock while reading both image and cam_params_[i]
        // so width/height/K are consistent with the image that was received.
        std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);

        // Build rescaled CameraParams: copy extrinsics, then overwrite w/h/K.
        scaled_params[i] = cam_params_[i];
        int info_w = cam_params_[i].width;
        int info_h = cam_params_[i].height;
        float sx = (info_w > 0) ? static_cast<float>(W) / info_w : 1.0f;
        float sy = (info_h > 0) ? static_cast<float>(H) / info_h : 1.0f;
        scaled_params[i].width  = W;
        scaled_params[i].height = H;
        scaled_params[i].K[0] *= sx;  // fx
        scaled_params[i].K[2] *= sx;  // cx
        scaled_params[i].K[4] *= sy;  // fy
        scaled_params[i].K[5] *= sy;  // cy

        if (!img_dirty_[i]) continue;
        any_dirty = true;

        const cv::Mat& m = per_cam_[i].image.value();
        int cam_H = m.rows, cam_W = m.cols;

        // Resize if needed
        cv::Mat resized;
        if (cam_H != H || cam_W != W)
            cv::resize(m, resized, cv::Size(W, H));
        else
            resized = m;

        // Copy into NHWC layout: [i, row, col, channel]
        size_t cam_offset = static_cast<size_t>(i) * H * W * 3;
        std::memcpy(nhwc.data() + cam_offset, resized.ptr<float>(),
                    static_cast<size_t>(H) * W * 3 * sizeof(float));
        img_dirty_[i] = false;
    }

    if (any_dirty)
    {
        reprojector_->set_cameras(scaled_params);
        reprojector_->upload_images(nhwc.data(), n_cameras_, H, W);
    }

    // ── render ───────────────────────────────────────────────────────────────
    std::vector<float> out_rgba(static_cast<size_t>(out_width_) * out_height_ * 4);
    reprojector_->render_bowl(vcam_, bowl_, out_rgba.data());

    // ── convert float RGBA → rgb8 ─────────────────────────────────────────
    cv::Mat out_rgb(out_height_, out_width_, CV_8UC3);
    for (int row = 0; row < out_height_; ++row)
    {
        for (int col = 0; col < out_width_; ++col)
        {
            int idx = (row * out_width_ + col) * 4;
            float r = std::min(1.0f, std::max(0.0f, out_rgba[idx + 0]));
            float g = std::min(1.0f, std::max(0.0f, out_rgba[idx + 1]));
            float b = std::min(1.0f, std::max(0.0f, out_rgba[idx + 2]));
            out_rgb.at<cv::Vec3b>(row, col) = {
                static_cast<uint8_t>(r * 255),
                static_cast<uint8_t>(g * 255),
                static_cast<uint8_t>(b * 255)};
        }
    }

    // ── publish image ─────────────────────────────────────────────────────
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", out_rgb).toImageMsg();
    img_msg->header.stamp = now();
    img_msg->header.frame_id = "rendering_virtual_cam";
    pub_image_->publish(*img_msg);

    // ── publish CameraInfo ────────────────────────────────────────────────
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = img_msg->header;
    info_msg.width = static_cast<uint32_t>(out_width_);
    info_msg.height = static_cast<uint32_t>(out_height_);
    info_msg.distortion_model = "plumb_bob";
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            info_msg.k[r * 3 + c] = static_cast<double>(vcam_.K[r * 3 + c]);
    pub_info_->publish(info_msg);
}

// ── Lifecycle: teardown helpers ──────────────────────────────────────────────
void RenderingNode::teardown_active()
{
    timer_.reset();
    img_subs_.clear();
    info_subs_.clear();
}

RenderingNode::CallbackReturn RenderingNode::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_deactivate() called.");
    teardown_active();
    pub_image_->on_deactivate();
    pub_info_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

RenderingNode::CallbackReturn RenderingNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_cleanup() called.");
    teardown_active();
    reprojector_.reset();
    per_cam_.clear();
    img_dirty_.clear();
    cam_params_.clear();
    pub_image_.reset();
    pub_info_.reset();
    return CallbackReturn::SUCCESS;
}

RenderingNode::CallbackReturn RenderingNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() called.");
    teardown_active();
    reprojector_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::rendering_app
