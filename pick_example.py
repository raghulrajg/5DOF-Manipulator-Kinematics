import numpy as np
from src.arm5dof import RobotConfig, fk, ik, pitch_from_rotation, solve

cfg = RobotConfig()

# 1) full pose matrix (e.g. from FoundationPose -> grasp pose in robot base frame)
T = np.array([[0, 0, 1, 0.0711],
              [0,-1, 0, 0.0   ],
              [1, 0, 0, 0.08  ],
              [0, 0, 0, 1     ]])
p, R = T[:3, 3], T[:3, :3]
for q in ik(p, pitch_from_rotation(p, R), R, cfg):
    print("IK (deg):", np.degrees(q).round(3))

# 2) position only, constrained + closest to the current joint state
# q_now = np.radians([0, -90, 90, 0, 90])
# q = solve(np.array([0.18, 0.05, 0.10]), q_now, cfg=cfg)
# print("constrained (deg):", None if q is None else np.degrees(q).round(2))
