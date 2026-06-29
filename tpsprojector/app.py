"""TPSProjector prototype application.

:class:`Engine` is the testable per-frame core: given a cinematic shot it
synthesizes the virtual view, composites the robot proxy, renders the
ground-truth reference, and computes validation metrics. :func:`main` is the
thin pygame display shell around it (preset switching with eased tweens, the
``[synthesized | ground-truth | diff]`` panel, and a free-orbit debug mode).

Run:  ``python -m tpsprojector.app``
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass

import numpy as np

from .camera import PinholeCamera
from .depth_renderer import synthetic_frames
from .presets import PRESET_NAMES, Shot, get_preset, tween
from .robot import RobotProxy, composite
from .surface import BowlSurface
from .transforms import Pose, look_at
from .validate import diff_heatmap, psnr, ssim
from .world.rig import make_ring_rig, tilt_for_body_edge
from .world.scene import default_scene

RENDER_MODES = ("hybrid", "depth", "bowl")

# The reprojection core is the C++ CUDA library, exposed via the `tpscuda`
# pybind module (built under cuda/install/…/python). Make it importable.
_CUDA_PY = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "cuda", "install",
    "libs", "rendering_reprojector", "python"))
if os.path.isdir(_CUDA_PY) and _CUDA_PY not in sys.path:
    sys.path.insert(0, _CUDA_PY)


def _cam_dict(cam: PinholeCamera) -> dict:
    """Pack a PinholeCamera into the {K,R,t,width,height} dict tpscuda expects."""
    return dict(K=np.asarray(cam.K, "f4").ravel(),
                R=np.asarray(cam.pose.R, "f4").ravel(),
                t=np.asarray(cam.pose.t, "f4"),
                width=cam.width, height=cam.height)


@dataclass
class RenderResult:
    synth: np.ndarray      # synthesized view with robot composited
    truth: np.ndarray      # ground-truth view (robot composited for parity)
    diff: np.ndarray       # error heatmap over the environment region
    valid: np.ndarray      # pixels that resolved to a camera
    robot_mask: np.ndarray
    psnr: float
    ssim: float


class Engine:
    def __init__(self, scene, cameras, frames, surface, robot, fov_deg,
                 width, height, mode="hybrid", splat_radius=1):
        import tpscuda  # lazy: only the CUDA path needs the GPU module present

        self.scene = scene
        self.cameras = cameras
        self.frames = frames                       # CameraFrames (image+depth)
        self.images = [f.image for f in frames]
        self.surface = surface
        self.robot = robot
        self.fov_deg = fov_deg
        self.width = width
        self.height = height
        self.mode = mode
        self.splat_radius = splat_radius
        self.tilt_deg = 0.0
        self.sky_color = np.array([0.45, 0.6, 0.8])  # fills genuinely-unseen sky

        # ── core reprojector: the C++ CUDA library via its pybind bindings ──
        # Calibration and images are static, so upload them once; only the
        # virtual pose changes per frame (see _render_env).
        self.reprojector = tpscuda.Reprojector(width, height)
        self.reprojector.set_cameras([_cam_dict(c) for c in cameras])
        self.reprojector.upload_images(
            np.stack([np.asarray(im, "f4") for im in self.images]))  # (N,H,W,3)
        # Depth feeds the depth/hybrid modes; inf marks "no return" (matches the
        # parity tests). Bowl mode ignores it.
        depth = np.stack([np.where(np.isfinite(f.depth), f.depth, np.inf).astype("f4")
                          for f in frames])
        self.reprojector.upload_depth(depth)

    @classmethod
    def from_defaults(cls, width=320, height=240, n_cameras=6, rig_fov_deg=85.0,
                      mount_radius=0.25, mount_height=0.55, body_radius=0.5,
                      tilt_deg=None, cam_width=320, cam_height=240, mode="hybrid"):
        scene = default_scene()
        # Realistic mounting: tilt each camera down just enough that its nearest
        # visible ground reaches the robot body edge (body boundary at the bottom
        # of frame, no blind ground ring), but no more. Computed from geometry.
        vfov = PinholeCamera.from_fov(cam_width, cam_height, rig_fov_deg,
                                      Pose.identity()).vfov_deg()
        # mount_height may be a scalar or a per-camera list (cameras at different
        # heights). Each camera's body-edge tilt follows from its own height.
        heights = np.atleast_1d(np.asarray(mount_height, dtype=float))
        if heights.size == 1:
            heights = np.repeat(heights, n_cameras)
        if tilt_deg is None:
            tilt_deg = [tilt_for_body_edge(h, mount_radius, body_radius, vfov)
                        for h in heights]
        cameras = make_ring_rig(n=n_cameras, hfov_deg=rig_fov_deg,
                                radius=mount_radius, mount_height=list(heights),
                                tilt_deg=tilt_deg, width=cam_width,
                                height=cam_height)
        # Per-camera ground-truth depth (synthetic now; real model/LIDAR later).
        frames = synthetic_frames(scene, cameras)
        # Bowl is the fallback geometry that fills depth disocclusion holes.
        surface = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
        robot = RobotProxy.default(footprint_radius=body_radius)
        eng = cls(scene, cameras, frames, surface, robot, fov_deg=70.0,
                  width=width, height=height, mode=mode)
        eng.tilt_deg = tilt_deg
        return eng

    def virtual_camera(self, shot: Shot) -> PinholeCamera:
        return PinholeCamera.from_fov(self.width, self.height, self.fov_deg,
                                      shot.pose())

    def _render_env(self, vc: PinholeCamera):
        """Render the environment per the active mode via the CUDA reprojector.

        Returns ``(rgb, valid)`` where ``valid`` is the alpha>0.5 coverage mask.
        bowl/depth/hybrid map 1:1 onto the C++ kernels; the hybrid composite
        (depth where valid, bowl fallback) is done inside the kernel.
        """
        s = self.surface
        vcam = _cam_dict(vc)
        if self.mode == "bowl":
            out = self.reprojector.render_bowl(vcam, s.R0, s.k, s.Rmax)
        elif self.mode == "depth":
            out = self.reprojector.render_depth(vcam, self.splat_radius)
        else:  # hybrid
            out = self.reprojector.render_hybrid(vcam, s.R0, s.k, s.Rmax,
                                                 self.splat_radius)
        return out[..., :3].astype(float), out[..., 3] > 0.5

    def synthesize(self, shot: Shot) -> RenderResult:
        vc = self.virtual_camera(shot)
        env, valid = self._render_env(vc)
        # Genuinely-unseen pixels (above the horizon / outside all coverage) read
        # as a sky color rather than black holes.
        env = np.where(valid[:, :, None], env, self.sky_color)
        robot_rgb, robot_depth = self.robot.render(vc)
        robot_mask = np.isfinite(robot_depth)

        synth = composite(env, robot_rgb, robot_depth)
        truth_env, _ = self.scene.render(vc)
        truth = composite(truth_env, robot_rgb, robot_depth)

        # measure projection quality on the environment only (exclude robot)
        env_mask = valid & ~robot_mask
        metric_psnr = psnr(env, truth_env, env_mask)
        metric_ssim = ssim(env, truth_env)
        diff = diff_heatmap(env, truth_env)
        diff[~valid] = 0.0

        return RenderResult(
            synth=np.clip(synth, 0, 1), truth=np.clip(truth, 0, 1), diff=diff,
            valid=valid, robot_mask=robot_mask,
            psnr=metric_psnr, ssim=metric_ssim)


# --------------------------------------------------------------------------
# pygame display shell (interactive; not exercised by the unit tests)
# --------------------------------------------------------------------------

def main():  # pragma: no cover
    import pygame
    from . import gl_present
    from .gl_context import use_window_context

    W, H = 960, 720
    pygame.init()
    pygame.display.set_mode((W, H), pygame.OPENGL | pygame.DOUBLEBUF)
    pygame.display.set_caption("TPSProjector — GL live")
    use_window_context()                       # GL backend renders into this window's context
    clock = pygame.time.Clock()

    eng = Engine.from_defaults(width=W, height=H)

    cur = get_preset(PRESET_NAMES[0]); src = dst = cur; t = 1.0
    orbit = False
    az, el, dist = np.radians(180.0), np.radians(28.0), 4.5

    def orbit_shot():
        ex = dist * np.cos(el) * np.cos(az)
        ey = dist * np.cos(el) * np.sin(az)
        ez = dist * np.sin(el) + 0.5
        return Shot(eye=[ex, ey, ez], target=[0.0, 0.0, 0.3])

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_o:
                    orbit = not orbit
                elif e.key == pygame.K_b:
                    eng.mode = "bowl"
                elif e.key == pygame.K_d:
                    eng.mode = "depth"
                elif e.key == pygame.K_h:
                    eng.mode = "hybrid"
                elif pygame.K_1 <= e.key <= pygame.K_9:
                    idx = e.key - pygame.K_1
                    if idx < len(PRESET_NAMES):
                        src, dst, t, orbit = cur, get_preset(PRESET_NAMES[idx]), 0.0, False

        keys = pygame.key.get_pressed()
        if orbit:
            if keys[pygame.K_LEFT]:  az -= 0.04
            if keys[pygame.K_RIGHT]: az += 0.04
            if keys[pygame.K_UP]:    el = min(el + 0.03, np.radians(85))
            if keys[pygame.K_DOWN]:  el = max(el - 0.03, np.radians(5))
            if keys[pygame.K_EQUALS]: dist = max(dist - 0.1, 1.5)
            if keys[pygame.K_MINUS]:  dist += 0.1
            cur = orbit_shot()
        else:
            if t < 1.0:
                t = min(1.0, t + 0.05); cur = tween(src, dst, t)
            else:
                cur = dst

        # CUDA reprojection (env) + Python robot composite + metrics, then present.
        res = eng.synthesize(cur)
        gl_present.present_array(np.clip(res.synth, 0, 1))
        hud_extra = f" PSNR={res.psnr:5.2f} SSIM={res.ssim:4.2f}"

        pygame.display.flip()
        clock.tick(0)   # uncapped, to see real fps
        pygame.display.set_caption(
            f"TPSProjector — mode={eng.mode} (cuda) "
            f"{'ORBIT' if orbit else 'preset'} fps={clock.get_fps():4.1f}{hud_extra}  "
            f"[1-4]preset [b/d/h]mode [o]rbit [esc]")

    pygame.quit()


if __name__ == "__main__":  # pragma: no cover
    main()
