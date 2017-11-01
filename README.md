### Manipulators kinematics

## Using as library

$ git clone git@github.com:kirillin/youbot_arm_kinematics.git

# Import and initialization for Python

```python
from youbot_arm_kinematics.kinematics import Kinematics
from youbot_arm_kinematics.dh_parameters import YouBotDHParameters

ks = Kinematics(YouBotDHParameters.DH_A, YouBotDHParameters.DH_ALPHA, YouBotDHParameters.DH_D, YouBotDHParameters.DH_THETA)
```

# Using for Python

For forward kinematics problem:

```python
xyz, qtn, rpy, h  = ks.forward(q)
```

where `xyz = [x, y, z]`, `qtn = [a, b, c, d]`, `rpy = [roll, pitch, yaw]`, `h` - matrix homogeneous transformation.

For inverse kinematics problem:

```python
qs, solves = ks.inverse(xyz, rpy)
```
where `qs` - matrix with configurations, `solves` - achievable configurations.

You can see detail in nodes `scripts/fk_test` and `scripts/ik_test`.
