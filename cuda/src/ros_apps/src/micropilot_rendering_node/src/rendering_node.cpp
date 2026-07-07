/** @file rendering_node.cpp
 *  @brief ROS2 LifecycleNode wrapping the micropilot_rendering CUDA reprojector.
 */

#include "micropilot_rendering_node/rendering_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "rendering_reprojector/mesh_loader.hpp"

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
    // Frame-sync window (s): render only when all cameras have a new frame whose
    // header stamps span <= this. ~10 fps cameras -> ~0.10 s period.
    max_sync_latency_ = declare_parameter<double>("max_sync_latency", 0.12);
    // Sky fill for unseen-above-bowl pixels (keeps the horizon natural, not black).
    auto sky = declare_parameter<std::vector<double>>("sky_color", {0.53, 0.70, 0.92});
    if (sky.size() == 3)
        for (int i = 0; i < 3; ++i) sky_color_[i] = static_cast<float>(sky[i]);

    bowl_.R0 = static_cast<float>(declare_parameter<double>("bowl_R0", 6.0));
    bowl_.k = static_cast<float>(declare_parameter<double>("bowl_k", 0.08));
    bowl_.Rmax = static_cast<float>(declare_parameter<double>("bowl_Rmax", 20.0));
    // Seam crossfade width (px on source images) — see BowlParams::feather_margin.
    bowl_.feather_margin =
        static_cast<float>(declare_parameter<double>("feather_margin", 30.0));
    // Fill the uncovered blind ring around the robot from surrounding scene
    // colors (GPU, pre-robot-overlay) instead of leaving it for the sky fill.
    bowl_.fill_blind_zone = declare_parameter<bool>("fill_blind_zone", true);
    // Auto-generated per-camera self-view masks (robot mesh rastered from each
    // camera pose). OFF by default: correct masks require the rig origin to be
    // exactly the body center — verify with the mask-overlay preview before
    // enabling (a wrong origin masks REAL scene, which is far worse than the
    // body smear it removes).
    self_view_masks_ = declare_parameter<bool>("self_view_masks", false);

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

    // Virtual camera intrinsics — pinhole with configurable vertical FOV.
    // Narrower FOV reduces wide-angle edge distortion (more realistic); the
    // auto-tuner sets this alongside the pose.
    double vfov_deg = declare_parameter<double>("virtual_vfov_deg", 60.0);
    float fy = (out_height_ / 2.0f) / std::tan(static_cast<float>(vfov_deg) * 0.5f * M_PI / 180.0f);
    float fx = fy;
    vcam_.K[0] = fx;  vcam_.K[1] = 0;   vcam_.K[2] = out_width_ / 2.0f;
    vcam_.K[3] = 0;   vcam_.K[4] = fy;  vcam_.K[5] = out_height_ / 2.0f;
    vcam_.K[6] = 0;   vcam_.K[7] = 0;   vcam_.K[8] = 1;

    // ── virtual-camera presets ────────────────────────────────────────────────
    // Preset 1 (config) is the just-built pose expressed as look-points:
    // eye = camera center t, target = t + forward (R column 2). look_at()
    // reproduces this exact rotation, so presets share one representation and
    // switching is a smoothstep tween of the look-points (see advance_tween()).
    presets_[0] = LookPoint{{vcam_.t[0], vcam_.t[1], vcam_.t[2]},
                            {vcam_.t[0] + vcam_.R[2], vcam_.t[1] + vcam_.R[5],
                             vcam_.t[2] + vcam_.R[8]}};
    // Presets 2-5: rig frame is x-forward, y-left, z-up (same as virtual_pose).
    // reverse_follow: the config view mirrored 180° about the robot's vertical
    // axis (negate x,y; keep height) — placed symmetrically opposite preset 1
    // across the robot at the origin, looking back the other way.
    presets_[1] = LookPoint{
        {-presets_[0].eye[0], -presets_[0].eye[1], presets_[0].eye[2]},
        {-presets_[0].target[0], -presets_[0].target[1], presets_[0].target[2]}};
    presets_[2] = LookPoint{{0.0f, 4.0f, 2.5f}, {0.0f, 0.0f, 0.5f}};    // left_side
    presets_[3] = LookPoint{{0.0f, -4.0f, 2.5f}, {0.0f, 0.0f, 0.5f}};   // right_side
    presets_[4] = LookPoint{{0.0f, 0.0f, 8.0f}, {0.0f, 0.001f, 0.0f}};  // top_down
    cur_ = src_ = dst_ = presets_[0];
    tween_t_ = 1.0;  // start settled on the config preset

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

    // ── robot proxy mesh (optional) ──────────────────────────────────────────
    // No real camera sees the robot, so the virtual view composites a known 3D
    // proxy. robot_model_path = "" disables it; load failure is non-fatal
    // (render without the robot rather than take the node down).
    auto robot_path = declare_parameter<std::string>("robot_model_path", "");
    // OBJ->rig [R(9 row-major)|t(3)]. Default matches the M02P Blender export:
    // obj x-fwd, y-up, z-right -> rig x-fwd, y-left, z-up.
    auto robot_tf = declare_parameter<std::vector<double>>(
        "robot_model_transform", {1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0});
    if (!robot_path.empty())
    {
        if (robot_tf.size() != 12)
        {
            RCLCPP_ERROR(get_logger(),
                         "robot_model_transform must be 12 floats [R(9)|t(3)], got %zu",
                         robot_tf.size());
            return CallbackReturn::FAILURE;
        }
        float T[12];
        for (int i = 0; i < 12; ++i) T[i] = static_cast<float>(robot_tf[i]);
        try
        {
            robot_mesh_ = micropilot::rendering::load_obj_mesh(robot_path, T);
            have_robot_mesh_ = true;
            reprojector_->upload_robot_mesh(robot_mesh_.verts.data(), robot_mesh_.cols.data(),
                                            robot_mesh_.n_tris);
            RCLCPP_INFO(get_logger(), "robot proxy: %zu triangles from %s",
                        robot_mesh_.n_tris, robot_path.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(),
                         "robot model load failed, rendering without robot: %s", e.what());
        }
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

    // ── preset-switch service ────────────────────────────────────────────────
    // ~/set_virtual_cam -> /rendering_node/set_virtual_cam. Switches the virtual
    // camera among the 5 presets with an eased tween (see on_set_virtual_cam).
    set_vcam_srv_ = create_service<SetVirtualCam>(
        "~/set_virtual_cam",
        std::bind(&RenderingNode::on_set_virtual_cam, this, std::placeholders::_1,
                  std::placeholders::_2));

    // ── free-look input + vcam telemetry ─────────────────────────────────────
    // ~/set_look: 6 floats [eye xyz | target xyz] in the rig frame, applied
    // immediately (orbiting streams continuous poses; a tween would lag them).
    // Generic runtime pose input — used by tools/vcam_ws_bridge.py.
    set_look_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/set_look", 10,
        std::bind(&RenderingNode::on_set_look, this, std::placeholders::_1));
    // ~/vcam_state: [eye xyz | target xyz | active_preset (0 = free look)],
    // published each render tick as telemetry for external UIs.
    pub_vcam_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/vcam_state", 1);

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
    pub_vcam_state_->on_activate();

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
                per_cam_[i].stamp = rclcpp::Time(msg->header.stamp);
                per_cam_[i].have_new = true;
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
                // plumb_bob distortion from the driver (real lenses publish raw
                // images); absent/empty D leaves the zeros = pinhole (sim).
                for (std::size_t j = 0; j < 5; ++j)
                    cp.dist[j] = j < msg->d.size() ? static_cast<float>(msg->d[j]) : 0.0f;
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
void RenderingNode::generate_self_masks()
{
    // Rasterize the robot mesh FROM each real camera's calibrated pose
    // (distortion-aware): nonzero pixels see the robot's own body, get skipped
    // during reprojection, and the blind-zone fill paints them from the scene.
    // Runs once, lazily, after every camera's CameraInfo arrived (real dims/K/D).
    int W = cam_params_[0].width, H = cam_params_[0].height;
    micropilot::rendering::Reprojector mask_rend(W, H);
    mask_rend.upload_robot_mesh(robot_mesh_.verts.data(), robot_mesh_.cols.data(),
                                robot_mesh_.n_tris);
    micropilot::rendering::BowlParams flat{};  // no env uploaded; robot layer only
    std::vector<unsigned char> masks(static_cast<size_t>(n_cameras_) * H * W, 0);
    std::vector<float> rgba(static_cast<size_t>(W) * H * 4);
    long total = 0;
    for (int i = 0; i < n_cameras_; ++i)
    {
        micropilot::rendering::CameraParams vc = cam_params_[i];
        vc.width = W;
        vc.height = H;
        mask_rend.render_bowl(vc, flat, rgba.data());
        cv::Mat m(H, W, CV_8U);
        for (int p = 0; p < W * H; ++p)
            m.data[p] = rgba[static_cast<size_t>(p) * 4 + 3] == 1.0f ? 255 : 0;
        // dilate a little: calibration is cm-accurate, not mm-accurate
        cv::dilate(m, m, cv::getStructuringElement(cv::MORPH_RECT, {9, 9}));
        std::memcpy(masks.data() + static_cast<size_t>(i) * H * W, m.data,
                    static_cast<size_t>(H) * W);
        total += cv::countNonZero(m);
    }
    reprojector_->upload_self_masks(masks.data(), n_cameras_, H, W);
    RCLCPP_INFO(get_logger(), "self-view masks: %ld body pixels masked across %d cameras",
                total, n_cameras_);
}

void RenderingNode::timer_callback()
{
    // ── lazy self-view mask generation (needs all CameraInfo; one-shot) ──────
    if (self_view_masks_ && !self_masks_done_ && have_robot_mesh_ && bowl_.fill_blind_zone)
    {
        bool infos = true;
        for (int i = 0; i < n_cameras_; ++i)
        {
            std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);
            infos = infos && per_cam_[i].info_ready;
        }
        if (infos)
        {
            try
            {
                generate_self_masks();
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(get_logger(), "self-view mask generation failed: %s", e.what());
            }
            self_masks_done_ = true;  // one attempt either way
        }
    }

    // Ease the virtual camera toward the selected preset (no-op once settled).
    advance_tween();

    // vcam telemetry — published before the frame-sync gate so external UIs
    // keep receiving pose updates even while waiting for camera frames.
    std_msgs::msg::Float64MultiArray state;
    state.data = {cur_.eye[0],    cur_.eye[1],    cur_.eye[2],
                  cur_.target[0], cur_.target[1], cur_.target[2],
                  static_cast<double>(active_preset_)};
    pub_vcam_state_->publish(state);

    // ── frame-sync gate ───────────────────────────────────────────────────────
    // Render only when EVERY camera has delivered a NEW frame since the last
    // render, and those frames' header stamps fall within max_sync_latency_.
    // This avoids stitching temporally-misaligned async frames (the cameras run
    // ~10 fps and arrive independently), which otherwise causes heavy flicker.
    rclcpp::Time t_min, t_max;
    for (int i = 0; i < n_cameras_; ++i)
    {
        std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);
        if (!per_cam_[i].image.has_value() || !per_cam_[i].have_new) return;  // wait
        const rclcpp::Time& s = per_cam_[i].stamp;
        if (i == 0) { t_min = s; t_max = s; }
        else { if (s < t_min) t_min = s; if (s > t_max) t_max = s; }
    }
    if ((t_max - t_min).seconds() > max_sync_latency_)
    {
        // Frames not yet aligned within the window — wait for a fresher, tighter set.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "camera frames span %.3fs > max_sync_latency %.3fs; skipping render",
                             (t_max - t_min).seconds(), max_sync_latency_);
        return;
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
            float r, g, b;
            if (out_rgba[idx + 3] < 0.5f)  // genuinely-unseen (above the bowl) -> sky, not black
            {
                r = sky_color_[0]; g = sky_color_[1]; b = sky_color_[2];
            }
            else
            {
                r = std::min(1.0f, std::max(0.0f, out_rgba[idx + 0]));
                g = std::min(1.0f, std::max(0.0f, out_rgba[idx + 1]));
                b = std::min(1.0f, std::max(0.0f, out_rgba[idx + 2]));
            }
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

    // Consume the synced set: require a fresh frame from every camera before the
    // next render (so the published rate tracks the synchronized camera rate).
    for (int i = 0; i < n_cameras_; ++i)
    {
        std::lock_guard<std::mutex> lk(*per_cam_[i].mtx);
        per_cam_[i].have_new = false;
    }
}

// ── Virtual-camera presets / eased switching ─────────────────────────────────
namespace
{
float smoothstep(float s)
{
    s = std::min(1.0f, std::max(0.0f, s));
    return s * s * (3.0f - 2.0f * s);
}
void cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}
void normalize(float v[3])
{
    float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-9f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
}  // namespace

// Mirror of tpsprojector.transforms.look_at: CV camera (R columns = right,
// down, fwd) looking from eye toward target with world-up +z. Falls back to a
// +y up vector when looking nearly straight up/down (top-down preset).
void RenderingNode::look_at(const float eye[3], const float target[3], float R_out[9])
{
    float f[3] = {target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]};
    normalize(f);
    float up[3] = {0.0f, 0.0f, 1.0f};
    float right[3];
    cross(f, up, right);
    if (std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]) < 1e-6f)
    {
        up[0] = 0.0f; up[1] = 1.0f; up[2] = 0.0f;
        cross(f, up, right);
    }
    normalize(right);
    float down[3];
    cross(f, right, down);
    // Row-major; columns are (right, down, fwd).
    R_out[0] = right[0]; R_out[1] = down[0]; R_out[2] = f[0];
    R_out[3] = right[1]; R_out[4] = down[1]; R_out[5] = f[1];
    R_out[6] = right[2]; R_out[7] = down[2]; R_out[8] = f[2];
}

void RenderingNode::apply_lookpoint(const LookPoint& lp)
{
    look_at(lp.eye, lp.target, vcam_.R);
    for (int i = 0; i < 3; ++i) vcam_.t[i] = lp.eye[i];
}

void RenderingNode::advance_tween()
{
    if (tween_t_ >= 1.0) return;  // settled — nothing to do
    // Timer fires at 33 ms; ~0.5 s transition -> step 0.033/0.5 per tick.
    tween_t_ = std::min(1.0, tween_t_ + 0.033 / 0.5);
    float w = smoothstep(static_cast<float>(tween_t_));
    for (int i = 0; i < 3; ++i)
    {
        cur_.eye[i] = src_.eye[i] + (dst_.eye[i] - src_.eye[i]) * w;
        cur_.target[i] = src_.target[i] + (dst_.target[i] - src_.target[i]) * w;
    }
    apply_lookpoint(cur_);
}

void RenderingNode::on_set_virtual_cam(const std::shared_ptr<SetVirtualCam::Request> req,
                                       std::shared_ptr<SetVirtualCam::Response> res)
{
    // ponytail: no lock — the node runs on a single-threaded executor
    // (rclcpp::spin in main.cpp), so this callback and timer_callback() never
    // overlap. Add a mutex here if it ever moves to a MultiThreadedExecutor.
    const int p = req->preset;
    if (p < 1 || p > static_cast<int>(presets_.size()))
    {
        res->success = false;
        res->active = "invalid preset (expected 1.." + std::to_string(presets_.size()) + ")";
        RCLCPP_WARN(get_logger(), "set_virtual_cam: rejected preset %d", p);
        return;
    }
    src_ = cur_;
    dst_ = presets_[p - 1];
    tween_t_ = 0.0;  // begin the eased transition
    active_preset_ = p;
    res->success = true;
    res->active = kPresetNames[p - 1];
    RCLCPP_INFO(get_logger(), "set_virtual_cam: -> preset %d (%s)", p, kPresetNames[p - 1]);
}

void RenderingNode::on_set_look(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Same single-threaded-executor note as on_set_virtual_cam: never overlaps
    // timer_callback(), so no lock.
    if (msg->data.size() != 6)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "set_look expects 6 floats [eye xyz | target xyz], got %zu",
                             msg->data.size());
        return;
    }
    LookPoint lp;
    for (int i = 0; i < 3; ++i) lp.eye[i] = static_cast<float>(msg->data[i]);
    for (int i = 0; i < 3; ++i) lp.target[i] = static_cast<float>(msg->data[3 + i]);
    cur_ = src_ = dst_ = lp;
    tween_t_ = 1.0;  // cancel any in-flight preset tween
    apply_lookpoint(cur_);
    active_preset_ = 0;  // free look
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
    pub_vcam_state_->on_deactivate();
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
    pub_vcam_state_.reset();
    set_vcam_srv_.reset();
    set_look_sub_.reset();
    return CallbackReturn::SUCCESS;
}

RenderingNode::CallbackReturn RenderingNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() called.");
    teardown_active();
    reprojector_.reset();
    set_vcam_srv_.reset();
    set_look_sub_.reset();
    return CallbackReturn::SUCCESS;
}

}  // namespace micropilot::rendering_app
