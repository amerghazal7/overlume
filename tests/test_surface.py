import numpy as np

from tpsprojector.surface import BowlSurface, FlatSurface


# ---------------- FlatSurface ----------------

def test_flat_downward_ray_hits_ground():
    surf = FlatSurface()
    o = np.array([[0.0, 1.0, 5.0]])
    d = np.array([[0.0, 0.0, -1.0]])
    hits, valid = surf.intersect(o, d)
    assert valid[0]
    np.testing.assert_allclose(hits[0], [0.0, 1.0, 0.0], atol=1e-6)


def test_flat_upward_ray_misses():
    surf = FlatSurface()
    o = np.array([[0.0, 0.0, 5.0]])
    d = np.array([[0.0, 0.0, 1.0]])
    hits, valid = surf.intersect(o, d)
    assert not valid[0]


def test_flat_oblique_ray():
    surf = FlatSurface()
    o = np.array([[0.0, 0.0, 2.0]])
    d = np.array([[1.0, 0.0, -1.0]])  # 45 deg down-forward
    hits, valid = surf.intersect(o, d)
    assert valid[0]
    np.testing.assert_allclose(hits[0], [2.0, 0.0, 0.0], atol=1e-6)


# ---------------- BowlSurface profile ----------------

def test_bowl_profile_flat_inside_floor_radius():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    assert bowl.height(0.0) == 0.0
    assert bowl.height(5.0) == 0.0
    assert bowl.height(10.0) == 0.0


def test_bowl_profile_rises_on_wall():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    # k*(r-R0)^2 = 0.04 * 25 = 1.0 at r=15
    np.testing.assert_allclose(bowl.height(15.0), 1.0, atol=1e-9)


def test_bowl_profile_tangent_at_floor_radius():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    eps = 1e-5
    slope = (bowl.height(10.0 + eps) - bowl.height(10.0)) / eps
    assert abs(slope) < 1e-3  # tangent to flat floor


def test_bowl_profile_clamps_beyond_rmax():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    hmax = bowl.height(30.0)
    assert bowl.height(40.0) == hmax


# ---------------- BowlSurface intersection ----------------

def test_bowl_floor_hit_inside_radius():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    o = np.array([[0.0, 0.0, 3.0]])
    d = np.array([[0.0, 0.0, -1.0]])
    hits, valid = bowl.intersect(o, d)
    assert valid[0]
    np.testing.assert_allclose(hits[0], [0.0, 0.0, 0.0], atol=1e-4)


def test_bowl_wall_hit_for_horizontal_ray():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    o = np.array([[0.0, 0.0, 1.0]])
    d = np.array([[1.0, 0.0, 0.0]])  # horizontal, z stays 1
    # wall: 0.04*(r-10)^2 = 1 -> r = 15
    hits, valid = bowl.intersect(o, d)
    assert valid[0]
    np.testing.assert_allclose(hits[0, 0], 15.0, atol=1e-3)
    np.testing.assert_allclose(hits[0, 2], 1.0, atol=1e-3)


def test_bowl_straight_up_ray_misses():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    o = np.array([[0.0, 0.0, 1.0]])
    d = np.array([[0.0, 0.0, 1.0]])
    hits, valid = bowl.intersect(o, d)
    assert not valid[0]


def test_bowl_vectorized_batch():
    bowl = BowlSurface(R0=10.0, k=0.04, Rmax=30.0)
    o = np.array([[0.0, 0.0, 3.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    d = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    hits, valid = bowl.intersect(o, d)
    np.testing.assert_array_equal(valid, [True, True, False])
    assert hits.shape == (3, 3)
