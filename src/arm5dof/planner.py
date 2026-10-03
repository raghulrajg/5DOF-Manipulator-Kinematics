"""Constrained solution selection on top of the analytic IK."""
import numpy as np
from .config import RobotConfig
from .ik import ik, pitch_from_rotation
from .kinematics import link_points


def is_valid(q, cfg: RobotConfig = RobotConfig()) -> bool:
    m = np.radians(cfg.margin_deg)
    b = cfg.bounds
    inside = np.all(q >= b[:, 0] + m) and np.all(q <= b[:, 1] - m)
    above_floor = np.all(link_points(q, cfg)[:, 2] >= cfg.z_min)
    return bool(inside and above_floor)


def solve(p, q_current, pitches=None, R=None, cfg: RobotConfig = RobotConfig()):
    """Best valid solution (closest to q_current) or None.

    pitches : candidate approach pitches (rad). Default sweeps -90..0 deg.
              If R is given and pitches is None, the pitch implied by R is used.
    R       : optional 3x3 target rotation (sets J5). If None, J5 is parked mid-range.
    """
    q_current = np.asarray(q_current, dtype=float)
    if pitches is None:
        pitches = ([pitch_from_rotation(p, R)] if R is not None
                   else np.radians(np.arange(-90, 1, 5)))
    mid5 = 0.5 * (cfg.bounds[4, 0] + cfg.bounds[4, 1])
    best, best_cost = None, np.inf
    for pitch in pitches:
        for q in ik(p, pitch, R, cfg):
            if R is None:
                q[4] = mid5
            if not is_valid(q, cfg):
                continue
            cost = np.linalg.norm(q - q_current)
            if cost < best_cost:
                best, best_cost = q, cost
    return best
