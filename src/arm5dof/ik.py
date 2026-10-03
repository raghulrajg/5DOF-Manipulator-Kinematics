"""Closed-form inverse kinematics.

Structure: J1 yaw, J2/J3/J4 parallel pitch joints, J5 tool roll.
Target = tool-tip position + tool-axis pitch (+ optional full rotation for J5).
`pitch` is the elevation of the tool axis measured in the vertical plane of the
target heading (0 = horizontal, -pi/2 = pointing straight down).
"""
import numpy as np
from .config import RobotConfig
from .kinematics import fk


def _wrap(q):
    return (q + np.pi) % (2 * np.pi) - np.pi


def ik(p, pitch, R=None, cfg: RobotConfig = RobotConfig(), tol=1e-9):
    """Return list of joint vectors (rad) that reach (p, pitch[, R]) within limits."""
    x, y, z = p
    lo, hi = cfg.bounds[:, 0], cfg.bounds[:, 1]
    lt = np.radians(cfg.limit_tol_deg)
    sols = []
    for flip in (0, 1):                               # shoulder front / back
        th1 = _wrap(np.arctan2(y, x) + flip * np.pi)
        r = np.hypot(x, y) * (1 if flip == 0 else -1)
        pb = pitch if flip == 0 else np.pi - pitch    # keep tool direction consistent
        wx = r - cfg.d5 * np.cos(pb)                  # wrist point in arm plane
        wy = (z - cfg.d1) - cfg.d5 * np.sin(pb)
        c = (wx**2 + wy**2 - cfg.a2**2 - cfg.a3**2) / (2 * cfg.a2 * cfg.a3)
        if abs(c) > 1 + tol:                          # out of workspace
            continue
        for sgn in (1, -1):                           # elbow branches
            t3p = sgn * np.arccos(np.clip(c, -1, 1))
            q2 = np.arctan2(wy, wx) - np.arctan2(cfg.a3 * np.sin(t3p),
                                                 cfg.a2 + cfg.a3 * np.cos(t3p))
            th2, th3 = q2 - cfg.phi, t3p + cfg.phi
            th4 = pb - (th2 + th3)
            th5 = 0.0
            if R is not None:
                R04 = fk([th1, th2, th3, th4, 0.0], cfg)[:3, :3]
                M = R04.T @ R
                th5 = np.arctan2(M[1, 0], M[0, 0])
            q = _wrap(np.array([th1, th2, th3, th4, th5]))
            if np.all(q >= lo - lt) and np.all(q <= hi + lt):
                sols.append(np.clip(q, lo, hi))       # snap tiny overshoot onto limit
    return sols


def pitch_from_rotation(p, R) -> float:
    """Tool-axis pitch implied by a target pose (p, R)."""
    h = np.arctan2(p[1], p[0])
    return float(np.arctan2(R[2, 2], np.cos(h) * R[0, 2] + np.sin(h) * R[1, 2]))
