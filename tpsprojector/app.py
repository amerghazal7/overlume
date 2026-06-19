"""TPSProjector prototype application.

:class:`Engine` is the testable per-frame core: given a cinematic shot it
synthesizes the virtual view, composites the robot proxy, renders the
ground-truth reference, and computes validation metrics. :func:`main` is the
thin pygame display shell around it (preset switching with eased tweens, the
``[synthesized | ground-truth | diff]`` panel, and a free-orbit debug mode).

Run:  ``python -m tpsprojector.app``
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .camera import PinholeCamera
from .presets import PRESET_NAMES, Shot, get_preset, tween
from .renderer import NumpyRenderer
from .robot import RobotProxy, composite
from .surface import BowlSurface
from .transforms import look_at
from .validate import diff_heatmap, psnr, ssim
from .world.rig import make_ring_rig
from .world.scene import default_scene


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
    def __init__(self, scene, cameras, images, surface, robot, fov_deg,
                 width, height, renderer=None):
        self.scene = scene
        self.cameras = cameras
        self.images = images
        self.surface = surface
        self.robot = robot
        self.fov_deg = fov_deg
        self.width = width
        self.height = height
        self.renderer = renderer or NumpyRenderer()

    @classmethod
    def from_defaults(cls, width=320, height=240, n_cameras=6, fov_deg=75.0,
                      cam_width=320, cam_height=240):
        scene = default_scene()
        cameras = make_ring_rig(n=n_cameras, hfov_deg=fov_deg,
                                width=cam_width, height=cam_height)
        images = [scene.render(c)[0] for c in cameras]
        surface = BowlSurface(R0=8.0, k=0.05, Rmax=25.0)
        robot = RobotProxy.default()
        return cls(scene, cameras, images, surface, robot, fov_deg=70.0,
                   width=width, height=height)

    def virtual_camera(self, shot: Shot) -> PinholeCamera:
        return PinholeCamera.from_fov(self.width, self.height, self.fov_deg,
                                      shot.pose())

    def synthesize(self, shot: Shot) -> RenderResult:
        vc = self.virtual_camera(shot)
        env, valid = self.renderer.render(self.images, self.cameras,
                                          self.surface, vc)
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

    W, H = 360, 270
    eng = Engine.from_defaults(width=W, height=H)

    pygame.init()
    screen = pygame.display.set_mode((W * 3, H + 40))
    pygame.display.set_caption("TPSProjector prototype")
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()

    cur = get_preset(PRESET_NAMES[0])
    src = cur
    dst = cur
    t = 1.0  # tween progress (1 = settled)
    show_validation = True

    # free-orbit debug state
    orbit = False
    az, el, dist = np.radians(180.0), np.radians(30.0), 5.0

    def orbit_shot():
        ex = dist * np.cos(el) * np.cos(az)
        ey = dist * np.cos(el) * np.sin(az)
        ez = dist * np.sin(el) + 0.5
        return Shot(eye=[ex, ey, ez], target=[0.0, 0.0, 0.3])

    def to_surf(arr):
        a = (np.clip(arr, 0, 1) * 255).astype(np.uint8)
        return pygame.surfarray.make_surface(np.transpose(a, (1, 0, 2)))

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_v:
                    show_validation = not show_validation
                elif e.key == pygame.K_o:
                    orbit = not orbit
                elif pygame.K_1 <= e.key <= pygame.K_9:
                    idx = e.key - pygame.K_1
                    if idx < len(PRESET_NAMES):
                        src, dst, t, orbit = cur, get_preset(PRESET_NAMES[idx]), 0.0, False

        keys = pygame.key.get_pressed()
        if orbit:
            if keys[pygame.K_LEFT]:
                az -= 0.04
            if keys[pygame.K_RIGHT]:
                az += 0.04
            if keys[pygame.K_UP]:
                el = min(el + 0.03, np.radians(85))
            if keys[pygame.K_DOWN]:
                el = max(el - 0.03, np.radians(5))
            if keys[pygame.K_EQUALS]:
                dist = max(dist - 0.1, 1.5)
            if keys[pygame.K_MINUS]:
                dist += 0.1
            cur = orbit_shot()
        else:
            if t < 1.0:
                t = min(1.0, t + 0.05)
                cur = tween(src, dst, t)
            else:
                cur = dst

        res = eng.synthesize(cur)
        screen.fill((20, 20, 24))
        screen.blit(to_surf(res.synth), (0, 0))
        if show_validation:
            screen.blit(to_surf(res.truth), (W, 0))
            screen.blit(to_surf(res.diff), (W * 2, 0))
            labels = ["synthesized", "ground truth", "diff (cold=good)"]
            for i, lab in enumerate(labels):
                screen.blit(font.render(lab, True, (220, 220, 220)), (i * W + 6, 4))

        hud = ("ORBIT" if orbit else "preset") + \
            f"  PSNR={res.psnr:5.2f}dB  SSIM={res.ssim:4.2f}  cover={res.valid.mean():.0%}"
        keys_help = "[1-4] presets  [v] validation  [o] orbit+arrows/+-  [esc] quit"
        screen.blit(font.render(hud, True, (255, 230, 140)), (6, H + 4))
        screen.blit(font.render(keys_help, True, (160, 160, 170)), (6, H + 22))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":  # pragma: no cover
    main()
