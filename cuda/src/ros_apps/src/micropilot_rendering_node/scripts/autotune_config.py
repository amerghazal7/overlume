#!/usr/bin/env python3
"""Auto-tune the rendering-node config from the live cameras.

Run this whenever the cameras / their mounting change, or when deploying on a
new robot. It:
  1. captures one synchronized set of the N camera frames + intrinsics,
  2. loads camera->rig extrinsics from a calibration source,
  3. geometrically derives a behind+above virtual pose whose top ray clears the
     horizon (no black sky border) for the chosen FOV,
  4. sweeps bowl params and picks the one that minimises overlap disagreement
     (how much the cameras disagree where they overlap — a proxy for ghosting /
     seams; lower = more realistic) subject to a coverage floor,
  5. writes a node params YAML and a montage PNG for visual confirmation.

Usage:
  python3 autotune_config.py \
      --source carla:/home/ag7/micropilot/micropilot_sim/config/carla_interface_config.yaml \
      --out /path/to/params.yaml [--frames cams.npz] [--vfov 50] [--no-capture]

  --source calib:/path/sensors_extrinsic_calib.yaml  uses camera_to_ego 4x4 mats
           (must already be in a z-up, ground-at-z=0 rig frame; use --ground-offset).
Requires ROS2 sourced (for live capture) and the built tpscuda module on PYTHONPATH.
"""
import argparse
import os
import sys
import numpy as np
import yaml
import cv2

REPO = "/home/ag7/Documents/TPSProjector"
sys.path.insert(0, REPO)
sys.path.insert(0, f"{REPO}/cuda/install/libs/rendering_reprojector/python")
from tpsprojector.transforms import look_at

DEF_NAMES = ["fl_camera", "fm_camera", "fr_camera", "bl_camera", "bm_camera", "br_camera"]
CARLA_NAME_MAP = {  # live name -> CARLA spawn name (if different); identity by default
}
CALIB_NAME_MAP = {  # live topic name -> real-robot calib entry (cams_calib.yaml keys)
    "fl_camera": "front_left_camera",
    "fm_camera": "front_camera",
    "fr_camera": "front_right_camera",
    "bl_camera": "back_left_camera",
    "bm_camera": "back_camera",
    "br_camera": "back_right_camera",
}


# ── extrinsics sources ───────────────────────────────────────────────────────
def _R(a, ax):
    c, s = np.cos(a), np.sin(a)
    if ax == "z": return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    if ax == "y": return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def extrinsics_carla(path, names):
    """CARLA spawn_point -> camera->rig [R(3x3), t]. LH->RH (negate y) + CV optical."""
    by = {c["name"]: c for c in yaml.safe_load(open(path))["cameras"]}
    R_body_opt = np.array([[0, 0, 1.], [-1, 0, 0], [0, -1, 0]])
    out = []
    for n in names:
        sp = by[CARLA_NAME_MAP.get(n, n)]["spawn_point"]
        t = np.array([sp["x"], -sp["y"], sp["z"]])
        roll, pitch, yaw = np.radians([sp["roll"], -sp["pitch"], -sp["yaw"]])
        R = (_R(yaw, "z") @ _R(pitch, "y") @ _R(roll, "x")) @ R_body_opt
        out.append((R, t))
    return out


def extrinsics_calib(path, names, ground_offset, name_map):
    """sensors_extrinsic_calib.yaml camera_to_ego 4x4 -> [R, t] (+ optional z offset).

    The calib ego frame IS rig-convention (x-fwd / y-left / z-up; cameras come
    out upright with headings/sides matching their names) but its ORIGIN is the
    VERTICAL CENTER of the robot body, NOT base_link — so camera z's are
    negative as parsed and a ground offset is needed. For m2o1 / cams_calib.yaml
    the body bottom sits 0.22 m above ground and the body is ~1.05 m tall, so
    --ground-z 0.75 (= 0.22 + body/2); the metric sweep minimum (~0.80) agrees
    to within a few cm. Do NOT axis-flip: a diag(1,-1,-1) "fix" renders the
    world upside-down while keeping cameras mutually consistent (2026-07-07)."""
    cal = yaml.safe_load(open(path))
    out = []
    for n in names:
        M = np.array(cal[name_map[n]]["camera_to_ego"], float)
        R, t = M[:3, :3], M[:3, 3].copy()
        t[2] += ground_offset
        out.append((R, t))
    return out


# ── live capture ─────────────────────────────────────────────────────────────
def capture_live(names, win=0.12, timeout_s=20.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo

    class Grab(Node):
        def __init__(self):
            super().__init__("autotune_grab")
            self.img = {n: None for n in names}
            self.st = {n: None for n in names}
            self.K = {n: None for n in names}
            self.D = {n: None for n in names}
            for n in names:
                self.create_subscription(Image, f"/{n}/raw_images",
                                         lambda m, nn=n: self._i(m, nn), qos_profile_sensor_data)
                self.create_subscription(CameraInfo, f"/{n}/camera_info",
                                         lambda m, nn=n: self._k(m, nn), qos_profile_sensor_data)

        def _i(self, m, n):
            self.img[n] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()
            self.st[n] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        def _k(self, m, n):
            self.K[n] = np.array(m.k, float).reshape(3, 3)
            self.D[n] = np.array(m.d, float) if len(m.d) else None  # plumb_bob [k1 k2 p1 p2 k3]

        def ready(self):
            if any(self.img[n] is None or self.K[n] is None for n in names):
                return False
            ss = [self.st[n] for n in names]
            return max(ss) - min(ss) <= win

    rclpy.init()
    g = Grab()
    end = g.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while rclpy.ok() and g.get_clock().now().nanoseconds < end and not g.ready():
        rclpy.spin_once(g, timeout_sec=0.05)
    ok = g.ready()
    imgs = {n: g.img[n] for n in names}
    Ks = {n: g.K[n] for n in names}
    Ds = {n: g.D[n] for n in names}
    rclpy.shutdown()
    if not ok:
        raise RuntimeError("cameras not all synced within window")
    return imgs, Ks, Ds


# ── rendering helpers (tpscuda) ──────────────────────────────────────────────
def make_cam(K, R, t, w, h, dist=None):
    cam = dict(K=np.asarray(K, "f4").ravel(), R=np.asarray(R, "f4").ravel(),
               t=np.asarray(t, "f4"), width=int(w), height=int(h))
    if dist is not None and np.any(np.asarray(dist)):
        cam["dist"] = np.asarray(dist, "f4").ravel()[:5]
    return cam


def vcam(eye, target, vfov, ow, oh):
    p = look_at(eye, target, world_up=(0, 0, 1))
    fy = (oh / 2.0) / np.tan(np.radians(vfov) / 2.0)
    K = np.array([fy, 0, ow / 2.0, 0, fy, oh / 2.0, 0, 0, 1.0], "f4")
    return dict(K=K, R=p.R.astype("f4").ravel(), t=np.asarray(p.t, "f4"), width=ow, height=oh), p


def driving_pose(eye_back, eye_height, vfov, sky_frac):
    """Teleop chase-cam: behind+above, tilted so the HORIZON sits ~sky_frac from the
    top of the frame. The operator still sees ahead to the horizon, but only a thin
    sky strip remains (no wasted half-frame of sky). Geometric relation:
      sky fraction of frame = (vfov/2 - downpitch) / vfov  =>  downpitch = (vfov/2)(1 - 2*sky_frac)
    Target placed on the ground ahead to realise that downpitch."""
    phi = vfov / 2.0
    theta = np.radians(max(phi * (1.0 - 2.0 * sky_frac), 1.0))  # downpitch (deg->rad)
    target_x = eye_height / np.tan(theta) - eye_back
    return [-eye_back, 0.0, eye_height], [target_x, 0.0, 0.0]


def overlap_score(tps, cams, imgs_f, V, R0, k, Rmax, ow, oh):
    """Render each camera alone; return (coverage, overlap_disagreement)."""
    frames, valids = [], []
    for i in range(len(cams)):
        tps.set_cameras([cams[i]])
        tps.upload_images(imgs_f[i:i + 1])
        out = tps.render_bowl(V, R0, k, Rmax)
        frames.append(out[..., :3])
        valids.append(out[..., 3] > 0.5)
    F = np.stack(frames)          # (N,H,W,3)
    Vd = np.stack(valids)         # (N,H,W)
    cnt = Vd.sum(0)
    coverage = float((cnt >= 1).mean())
    ov = cnt >= 2
    if ov.sum() == 0:
        return coverage, 1e9
    # per-pixel mean over valid cams, then mean abs deviation over valid cams
    Vf = Vd[..., None].astype(np.float32)
    msum = (F * Vf).sum(0)
    mean = msum / np.maximum(cnt[..., None], 1)
    dev = (np.abs(F - mean) * Vf).sum(0) / np.maximum(cnt[..., None], 1)
    disagree = float(dev.mean(-1)[ov].mean())
    return coverage, disagree


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="carla:/home/ag7/micropilot/micropilot_sim/config/carla_interface_config.yaml")
    ap.add_argument("--names", default=",".join(DEF_NAMES))
    ap.add_argument("--out", default=f"{REPO}/cuda/src/ros_apps/src/micropilot_rendering_node/config/default_params.yaml")
    ap.add_argument("--frames", default="")     # npz to skip live capture
    ap.add_argument("--montage", default="/tmp/autotune_montage.png")
    ap.add_argument("--out-width", type=int, default=1280)
    ap.add_argument("--out-height", type=int, default=720)
    ap.add_argument("--vfov", type=float, default=65.0)        # forward-aware driving FOV
    ap.add_argument("--eye-back", type=float, default=6.0)
    ap.add_argument("--eye-height", type=float, default=4.0)
    ap.add_argument("--sky-frac", type=float, default=0.10)    # fraction of frame left as sky (horizon near top)
    ap.add_argument("--sky", default="0.53,0.70,0.92")         # sky fill RGB for unseen region
    # Bowl: modest wall by default so distant objects "stand up" (option C); tunable.
    # Set --bowl-k 0 / large --bowl-r0 for a flat floor (less seam, squashed far objects).
    ap.add_argument("--bowl-r0", type=float, default=12.0)
    ap.add_argument("--bowl-k", type=float, default=0.06)
    ap.add_argument("--bowl-rmax", type=float, default=35.0)
    # Seam crossfade width in source-image px; should span a good fraction of the
    # camera overlap (1440px @ 120deg HFOV, >=50deg overlap -> ~480px ~= 30deg).
    ap.add_argument("--feather", type=float, default=480.0)
    ap.add_argument("--ground-offset", type=float, default=0.0)
    ap.add_argument("--odom-topic", default="",
                    help="nav_msgs/Odometry topic for the node's ego-motion time "
                         "compensation ('' disables); real robots with free-running "
                         "cameras need this while driving (e.g. /fixposition/odometry_enu)")
    ap.add_argument("--coverage-floor", type=float, default=0.90)
    ap.add_argument("--max-sync-latency", type=float, default=0.12)
    ap.add_argument("--ground-z", default="auto",
                    help="global camera-height offset (m) correcting rig-origin-vs-ground "
                         "error; 'auto' sweeps for the overlap-disagreement minimum")
    ap.add_argument("--robot-model", default="/home/ag7/Downloads/M02P.obj",
                    help="robot proxy OBJ path written to robot_model_path ('' disables)")
    ap.add_argument("--robot-transform",
                    default="1.0,0,0,0,0,-1.0,0,1.0,0,0,0,0",
                    help="12 floats [R(9)|t(3)] OBJ->rig for robot_model_transform "
                         "(1.0x scale: visually calibrated against the camera-projected body)")
    a = ap.parse_args()
    names = a.names.split(",")

    # 1. frames + K (--frames caches the capture: loads if the npz exists,
    # otherwise captures live and saves — lets param sweeps reuse one frame set)
    if a.frames and os.path.exists(a.frames):
        d = np.load(a.frames)
        imgs = {n: d[f"img_{n}"] for n in names}
        Ks = {n: d[f"K_{n}"] for n in names}
        Ds = {n: (d[f"D_{n}"] if f"D_{n}" in d.files else None) for n in names}
    else:
        imgs, Ks, Ds = capture_live(names)
        if a.frames:
            np.savez_compressed(
                a.frames,
                **{f"img_{n}": imgs[n] for n in names},
                **{f"K_{n}": Ks[n] for n in names},
                **{f"D_{n}": Ds[n] for n in names if Ds[n] is not None})
            print(f"WROTE frames -> {a.frames}")
    H, W = imgs[names[0]].shape[:2]

    # 2. extrinsics
    kind, path = a.source.split(":", 1)
    if kind == "carla":
        ext = extrinsics_carla(path, names)
    elif kind == "calib":
        ext = extrinsics_calib(path, names, a.ground_offset,
                               {n: CALIB_NAME_MAP.get(n, n) for n in names})
    else:
        raise SystemExit(f"unknown --source kind {kind}")

    import tpscuda
    OW, OH = a.out_width, a.out_height
    cams = [make_cam(Ks[n], R, t, W, H, Ds[n]) for n, (R, t) in zip(names, ext)]
    imgs_f = np.stack([imgs[n].astype("f4") / 255.0 for n in names])
    tps = tpscuda.Reprojector(OW, OH)

    # 3. teleop driving pose (horizon near the top, minimal sky)
    eye, target = driving_pose(a.eye_back, a.eye_height, a.vfov, a.sky_frac)
    V, pose = vcam(eye, target, a.vfov, OW, OH)
    print(f"pose eye={np.round(eye,2).tolist()} target={np.round(target,2).tolist()} "
          f"vfov={a.vfov} sky_frac={a.sky_frac}")

    # 3b. ground-z calibration. The rig frame assumes ground at z=0, but the
    # CARLA vehicle pivot is only ~ground (suspension, mesh origin). A dz error
    # shifts each camera's ground reprojection by dz*d/h — cameras at DIFFERENT
    # heights shift differently, splitting lane lines at the seams. Sweep a
    # global camera-height offset on the captured frames and keep the overlap-
    # disagreement minimum (coarse 1 cm, then 2 mm refine around the best).
    if a.ground_z == "auto":
        # Score ground alignment on a TOP-DOWN view: dz shifts near-ground
        # reprojection strongly and far-field weakly, so a forward view (mostly
        # wall content) lets distortion noise dominate the sweep. Top-down is
        # ground-dominated — the split-lane symptom is exactly what it measures.
        V_ground, _ = vcam([0.0, 0.0, 5.0], [0.01, 0.0, 0.0], 60.0, OW, OH)
        def dis_at(dz):
            cams_dz = [make_cam(Ks[n], R, t + np.array([0.0, 0.0, dz]), W, H, Ds[n])
                       for n, (R, t) in zip(names, ext)]
            _, dis = overlap_score(tps, cams_dz, imgs_f, V_ground,
                                   a.bowl_r0, a.bowl_k, a.bowl_rmax, OW, OH)
            return dis
        if kind == "calib":
            # calib ego origins sit ABOVE the ground (e.g. m2o1's lidar frame).
            # Bound by ROBOT PHYSICS: every camera between 0.25 m and the robot
            # top (1.30 m). This excludes the degenerate high-dz attractor where
            # raised cameras lose parallax and game the disagreement metric.
            min_tz = min(t[2] for _, t in ext)
            max_tz = max(t[2] for _, t in ext)
            lo, hi, coarse = 0.25 - min_tz, 1.30 - max_tz, 0.02
        else:
            lo, hi, coarse = -0.08, 0.08, 0.01  # CARLA pivots are near-ground
        best_dz, best_dis = 0.0, dis_at(0.0)
        for dz in np.arange(lo, hi + coarse / 2, coarse):
            dis = dis_at(dz)
            if dis < best_dis:
                best_dz, best_dis = dz, dis
        for dz in np.arange(best_dz - 0.008, best_dz + 0.0081, 0.002):
            dis = dis_at(dz)
            if dis < best_dis:
                best_dz, best_dis = dz, dis
        ground_z = float(best_dz)
        print(f"ground-z calibration: dz={ground_z:+.3f} m "
              f"(disagreement {best_dis:.4f} vs {dis_at(0.0):.4f} at 0)")
    else:
        ground_z = float(a.ground_z)
        print(f"ground-z offset (manual): dz={ground_z:+.3f} m")
    ext = [(R, t + np.array([0.0, 0.0, ground_z])) for (R, t) in ext]
    cams = [make_cam(Ks[n], R, t, W, H, Ds[n]) for n, (R, t) in zip(names, ext)]

    # 4. bowl montage (informational): flat -> walled. The CHOSEN bowl is the
    # tunable --bowl-* (default a modest wall so distant objects stand up); the
    # printed "seam" (overlap disagreement) is a quality readout, not the decider,
    # because flat minimises seams but squashes far objects — a style tradeoff.
    chosen_bowl = (a.bowl_r0, a.bowl_k, a.bowl_rmax)
    candidates = sorted(set([
        (30.0, 0.012, 80.0), (15.0, 0.03, 40.0), chosen_bowl,
        (10.0, 0.10, 30.0), (8.0, 0.15, 25.0), (6.0, 0.20, 22.0),
    ]))
    tiles, results = [], []
    for (R0, k, Rmax) in candidates:
        cov, dis = overlap_score(tps, cams, imgs_f, V, R0, k, Rmax, OW, OH)
        results.append((R0, k, Rmax, cov, dis))
        # full blended render for the montage
        tps.set_cameras(cams)
        tps.upload_images(imgs_f)
        out = tps.render_bowl(V, R0, k, Rmax)
        bgr = cv2.cvtColor((np.clip(out[..., :3], 0, 1) * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
        cv2.putText(bgr, f"R0={R0} k={k} Rmax={Rmax} cov={cov*100:.0f}% seam={dis:.3f}",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        tiles.append(cv2.resize(bgr, (OW // 2, OH // 2)))
        print(f"R0={R0} k={k} Rmax={Rmax}: coverage {cov*100:.1f}% disagreement {dis:.4f}")

    R0, k, Rmax = chosen_bowl
    chosen_dis = next((r[4] for r in results if (r[0], r[1], r[2]) == chosen_bowl), float("nan"))
    print(f"CHOSEN bowl (tunable via --bowl-*): R0={R0} k={k} Rmax={Rmax} (disagreement {chosen_dis:.4f})")

    # montage
    cols = 3
    rows = (len(tiles) + cols - 1) // cols
    th, tw = tiles[0].shape[:2]
    grid = np.zeros((rows * th, cols * tw, 3), np.uint8)
    for i, t in enumerate(tiles):
        rr, cc = divmod(i, cols)
        grid[rr*th:(rr+1)*th, cc*tw:(cc+1)*tw] = t
    cv2.imwrite(a.montage, grid)

    # 5. write config
    ext_flat = []
    for (R, t) in ext:
        ext_flat += [float(v) for v in np.asarray(R).reshape(-1)] + [float(v) for v in t]
    vpose = [float(v) for v in list(pose.R.reshape(-1)) + list(pose.t)]
    params = {"/**": {"ros__parameters": {
        "n_cameras": len(names),
        "out_width": OW, "out_height": OH,
        "max_sync_latency": a.max_sync_latency,
        "odom_topic": a.odom_topic,
        "bowl_R0": float(R0), "bowl_k": float(k), "bowl_Rmax": float(Rmax),
        "feather_margin": float(a.feather),
        "fill_blind_zone": True,
        "robot_model_path": a.robot_model,
        "robot_model_transform": [float(x) for x in a.robot_transform.split(",")],
        "virtual_vfov_deg": float(a.vfov),
        "sky_color": [float(x) for x in a.sky.split(",")],
        "image_topics": [f"/{n}/raw_images" for n in names],
        "info_topics": [f"/{n}/camera_info" for n in names],
        "camera_extrinsics": ext_flat,
        "virtual_pose": vpose,
    }}}
    with open(a.out, "w") as f:
        f.write("# Auto-generated by autotune_config.py — re-run when cameras/mounting change.\n")
        f.write(f"# source: {a.source}\n")
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    print(f"WROTE config -> {a.out}")
    print(f"WROTE montage -> {a.montage}")


if __name__ == "__main__":
    main()
