import numpy as np

from tpsprojector.camera import PinholeCamera
from tpsprojector.robot import RobotProxy, composite
from tpsprojector.transforms import look_at


def test_robot_mesh_is_near_origin_and_stands_up():
    robot = RobotProxy.default()
    v = robot.vertices
    assert np.all(np.abs(v[:, 0]) < 2.0)
    assert np.all(np.abs(v[:, 1]) < 2.0)
    assert v[:, 2].min() >= -1e-9      # sits on the ground
    assert v[:, 2].max() > 0.2         # has height


def test_robot_visible_from_behind():
    robot = RobotProxy.default()
    cam = PinholeCamera.from_fov(120, 120, 70.0,
                                 look_at(eye=[-2.0, 0.0, 1.5], target=[0.0, 0.0, 0.25]))
    rgb, depth = robot.render(cam)
    assert np.isfinite(depth).any()             # robot drawn
    # drawn pixels cluster around the image center
    ys, xs = np.where(np.isfinite(depth))
    assert abs(xs.mean() - 60) < 30
    assert abs(ys.mean() - 60) < 40


def test_composite_robot_over_environment():
    env = np.zeros((4, 4, 3))
    env[..., 1] = 1.0  # green environment
    robot_rgb = np.zeros((4, 4, 3))
    robot_rgb[..., 0] = 1.0  # red robot
    robot_depth = np.full((4, 4), np.inf)
    robot_depth[1:3, 1:3] = 2.0  # robot occupies a 2x2 block

    out = composite(env, robot_rgb, robot_depth)
    # block is red (robot), rest stays green (environment)
    np.testing.assert_allclose(out[1, 1], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(out[0, 0], [0.0, 1.0, 0.0])
