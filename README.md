# arm5dof

Closed-form forward / inverse kinematics for a custom 5-DOF arm (standard DH),
with joint-limit and floor constraints.

## Install
```bash
pip install -e .            # from this folder
pip install -e ".[dev]"     # + pytest
```

## Use
```python
import numpy as np
from arm5dof import fk, ik, pitch_from_rotation, solve

T = fk(np.radians([10, -60, 80, -20, 45]))
p, R = T[:3, 3], T[:3, :3]
sols = ik(p, pitch_from_rotation(p, R), R)          # all valid solutions (rad)
q = solve(p, q_current=np.zeros(5), R=R)            # best constrained solution or None
```

## Robot parameters
All geometry, joint limits, floor height and margins live in `arm5dof/config.py`
(`RobotConfig`). Pass a custom instance via `cfg=`.

## Notes
* 5 DOF => target = position + tool pitch (+ roll via J5). The tool axis must lie
  in the vertical plane through the base axis and the target.
* `pitch`: elevation of the tool axis (0 = horizontal, -pi/2 = straight down).
* Home pose (all zeros) sits on joint limits and is near the folded-elbow singularity.

## Test
```bash
pytest
```
