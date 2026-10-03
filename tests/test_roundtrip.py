import numpy as np
from arm5dof import RobotConfig, fk, ik, pitch_from_rotation, solve

CFG = RobotConfig()


def test_fk_ik_roundtrip():
    rng = np.random.default_rng(0)
    b = CFG.bounds
    for _ in range(1000):
        q = rng.uniform(b[:, 0], b[:, 1])
        T = fk(q, CFG)
        p, R = T[:3, 3], T[:3, :3]
        sols = ik(p, pitch_from_rotation(p, R), R, CFG)
        assert sols, "no solution for a reachable pose"
        err = min(np.linalg.norm(fk(s, CFG) - T) for s in sols)
        assert err < 1e-6


def test_home_pose_matrix():
    T = np.array([[0, 0, 1, 0.0711], [0, -1, 0, 0.0], [1, 0, 0, 0.08], [0, 0, 0, 1.0]])
    p, R = T[:3, 3], T[:3, :3]
    sols = ik(p, pitch_from_rotation(p, R), R, CFG)
    assert sols
    assert np.linalg.norm(fk(sols[0], CFG)[:3, 3] - p) < 1e-3


def test_unreachable_returns_none():
    assert solve(np.array([0.60, 0.0, 0.10]), np.zeros(5)) is None
