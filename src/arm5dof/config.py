"""Robot parameters. Edit here (or pass your own RobotConfig) - nothing is hard-coded elsewhere."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class RobotConfig:
    # link geometry (m)
    d1: float = 0.04
    a2: float = 0.199
    a3: float = 0.14
    d5: float = 0.126
    # DH angle offset on joints 2 and 3 (+phi on J2, -phi on J3)
    phi_deg: float = 168.4
    # joint limits (deg): J1..J5
    bounds_deg: tuple = ((-90, 90), (-180, 0), (0, 180), (-90, 90), (0, 180))
    # constraints
    z_min: float = 0.0            # floor / table height in base frame (m)
    margin_deg: float = 3.0       # keep this far from joint limits in solve()
    limit_tol_deg: float = 0.5    # ik() snaps overshoot up to this much onto the limit

    @property
    def phi(self) -> float:
        return float(np.radians(self.phi_deg))

    @property
    def bounds(self) -> np.ndarray:
        return np.radians(np.array(self.bounds_deg, dtype=float))
