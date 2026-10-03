"""Forward kinematics (standard DH: Rz(theta) Tz(d) Tx(a) Rx(alpha))."""
import numpy as np
from .config import RobotConfig


def dh(alpha, a, d, theta) -> np.ndarray:
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca,      d],
                     [0.0,     0.0,      0.0,    1.0]])


def dh_table(q, cfg: RobotConfig):
    t1, t2, t3, t4, t5 = q
    return [(np.pi / 2, 0.0,    cfg.d1, t1),
            (0.0,       cfg.a2, 0.0,    t2 + cfg.phi),
            (0.0,       cfg.a3, 0.0,    t3 - cfg.phi),
            (np.pi / 2, 0.0,    0.0,    t4 + np.pi / 2),
            (0.0,       0.0,    cfg.d5, t5)]


def fk(q, cfg: RobotConfig = RobotConfig()) -> np.ndarray:
    """4x4 base->tool transform for joint vector q (rad)."""
    T = np.eye(4)
    for row in dh_table(q, cfg):
        T = T @ dh(*row)
    return T


def link_points(q, cfg: RobotConfig = RobotConfig()) -> np.ndarray:
    """Origins of base, joint frames 1-5 (tool tip last) - used for collision checks."""
    T, pts = np.eye(4), [np.zeros(3)]
    for row in dh_table(q, cfg):
        T = T @ dh(*row)
        pts.append(T[:3, 3].copy())
    return np.array(pts)
