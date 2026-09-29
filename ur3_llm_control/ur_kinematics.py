"""Offline UR3/UR3e kinematics (pure numpy, no ROS).

Used ONLY for:
  * the offline self-check of scene.yaml (are all pick/place poses reachable
    with the tool pointing down, inside joint limits?)
  * the FakeMotionBackend used by the unit tests.

The real robot motion is always planned and executed by MoveIt 2.
The chain below mirrors ur_description/urdf/ur_macro.xacro (Humble branch):
world -> base_link -> base_link_inertia (rz=pi) -> shoulder ... -> tool0.
"""
import math

import numpy as np

# default_kinematics.yaml from ur_description (x, y, z, roll, pitch, yaw)
_PARAMS = {
    'ur3e': {
        'shoulder': (0, 0, 0.15185, 0, 0, 0),
        'upper_arm': (0, 0, 0, math.pi / 2, 0, 0),
        'forearm': (-0.24355, 0, 0, 0, 0, 0),
        'wrist_1': (-0.2132, 0, 0.13105, 0, 0, 0),
        'wrist_2': (0, -0.08535, 0, math.pi / 2, 0, 0),
        'wrist_3': (0, 0.0921, 0, math.pi / 2, math.pi, math.pi),
    },
    'ur3': {
        'shoulder': (0, 0, 0.1519, 0, 0, 0),
        'upper_arm': (0, 0, 0, math.pi / 2, 0, 0),
        'forearm': (-0.24365, 0, 0, 0, 0, 0),
        'wrist_1': (-0.21325, 0, 0.11235, 0, 0, 0),
        'wrist_2': (0, -0.08535, 0, math.pi / 2, 0, 0),
        'wrist_3': (0, 0.0819, 0, math.pi / 2, math.pi, math.pi),
    },
}

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
# joint_limits.yaml: +-2pi except elbow (+-pi)
JOINT_LOWER = np.array([-2 * math.pi, -2 * math.pi, -math.pi,
                        -2 * math.pi, -2 * math.pi, -2 * math.pi])
JOINT_UPPER = -JOINT_LOWER


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _tf(x, y, z, r, p, yw):
    t = np.eye(4)
    t[:3, :3] = _rpy(r, p, yw)
    t[:3, 3] = (x, y, z)
    return t


def _rz(q):
    t = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    t[:2, :2] = [[c, -s], [s, c]]
    return t


class URKinematics:
    """Forward / numerical inverse kinematics of tool0 in the world frame."""

    def __init__(self, ur_type='ur3e'):
        if ur_type not in _PARAMS:
            raise ValueError(f'unsupported ur_type {ur_type}')
        p = _PARAMS[ur_type]
        self._origins = [_tf(*p[k]) for k in (
            'shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3')]
        self._base = _tf(0, 0, 0, 0, 0, math.pi)             # base_link_inertia
        self._tool = (_tf(0, 0, 0, 0, -math.pi / 2, -math.pi / 2)   # flange
                      @ _tf(0, 0, 0, math.pi / 2, 0, math.pi / 2))  # tool0

    def fk(self, q):
        t = self._base.copy()
        for origin, qi in zip(self._origins, q):
            t = t @ origin @ _rz(qi)
        return t @ self._tool

    # ------------------------------------------------------------------
    def _error(self, q, pos, down_only):
        t = self.fk(q)
        e_pos = pos - t[:3, 3]
        z_axis = t[:3, 2]
        # tool z must point to world -z; yaw about the vertical is free
        e_rot = np.cross(z_axis, np.array([0.0, 0.0, -1.0])) if down_only else np.zeros(3)
        return np.concatenate([e_pos, e_rot])

    def ik_down(self, pos, seeds=None, iters=300, tol=1e-4):
        """Tool pointing straight down at `pos` (x, y, z) in world.

        Returns joint vector within limits or None."""
        pos = np.asarray(pos, dtype=float)
        if seeds is None:
            seeds = []
            for pan in np.linspace(-math.pi, math.pi, 9):
                seeds.append([pan, -1.2, 1.3, -1.7, -1.57, 0.0])
                seeds.append([pan, -2.0, -1.3, -1.4, 1.57, 0.0])
        for seed in seeds:
            q = np.array(seed, dtype=float)
            for _ in range(iters):
                e = self._error(q, pos, True)
                if np.linalg.norm(e) < tol:
                    break
                jac = np.zeros((6, 6))
                for j in range(6):
                    dq = np.zeros(6)
                    dq[j] = 1e-6
                    jac[:, j] = (self._error(q + dq, pos, True) - e) / -1e-6
                lam = 1e-3
                step = jac.T @ np.linalg.solve(jac @ jac.T + lam * np.eye(6), e)
                q = q + np.clip(step, -0.3, 0.3)
            if (np.linalg.norm(self._error(q, pos, True)) < tol
                    and np.all(q >= JOINT_LOWER) and np.all(q <= JOINT_UPPER)):
                return q
        return None
