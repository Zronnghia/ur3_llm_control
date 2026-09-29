"""Pure-python motion backend (no ROS).  Checks every target with the UR
numerical IK (tool pointing down, joint limits) and tracks attachments, so the
full pipeline can be tested without Gazebo/MoveIt."""
from .robot_skills import MotionBackend
from .skill_spec import Status
from .ur_kinematics import URKinematics


class FakeMotionBackend(MotionBackend):
    def __init__(self, scene, fail_on=None):
        self.scene = scene
        self.kin = URKinematics(scene.ur_type)
        self.tool = None                  # (x, y, z) of tool0
        self.attached = None
        self.calls = []
        self.fail_on = fail_on or set()   # e.g. {'move_pose_down'} to inject failures

    def _reach(self, xyz, name):
        self.calls.append((name, tuple(round(v, 4) for v in xyz)))
        if name in self.fail_on or self.kin.ik_down(xyz) is None:
            return Status.PLANNING_FAILED
        self.tool = tuple(xyz)
        return Status.SUCCESS

    def move_joints(self, joints):
        self.calls.append(('move_joints', tuple(joints)))
        if 'move_joints' in self.fail_on:
            return Status.PLANNING_FAILED
        self.tool = tuple(self.kin.fk(joints)[:3, 3])
        return Status.SUCCESS

    def move_pose_down(self, xyz):
        return self._reach(xyz, 'move_pose_down')

    def move_vertical(self, z):
        if self.tool is None:
            return Status.FAILED
        return self._reach((self.tool[0], self.tool[1], z), 'move_vertical')

    def attach(self, obj):
        self.calls.append(('attach', obj))
        if self.attached is not None:
            return Status.FAILED
        self.attached = obj
        return Status.SUCCESS

    def detach(self, obj, center_xyz):
        self.calls.append(('detach', obj, tuple(center_xyz)))
        if self.attached != obj:
            return Status.FAILED
        self.attached = None
        return Status.SUCCESS
