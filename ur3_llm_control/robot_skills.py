"""Robot skills.

Every skill is a short, deterministic sequence of MoveIt 2 motions and returns
a Status string.  All geometry (heights, approach distances, orientations)
lives here and in scene.yaml – never in the LLM output.

The motion layer is abstract (MotionBackend) so the same skills run against
  * MoveItBackend  (ROS 2 / MoveIt 2 / Gazebo, see moveit_backend.py)
  * FakeMotionBackend (pure python, used by the unit tests)
"""
from .scene_model import SceneConfig, WorldState
from .skill_spec import Status


class MotionBackend:
    """Interface implemented by the MoveIt backend and the fake backend."""

    def move_joints(self, joints):                       # -> Status
        raise NotImplementedError

    def move_pose_down(self, xyz):                       # free-space plan, tool down
        raise NotImplementedError

    def move_vertical(self, z):                          # straight-line Cartesian z
        raise NotImplementedError

    def attach(self, obj):                               # virtual gripper closes
        raise NotImplementedError

    def detach(self, obj, center_xyz):                   # virtual gripper opens
        raise NotImplementedError


class RobotSkills:
    def __init__(self, scene: SceneConfig, backend: MotionBackend, world: WorldState):
        self.scene = scene
        self.backend = backend
        self.world = world
        self.m = scene.motion

    # ----------------------------------------------------------- primitives
    def home(self):
        return self.backend.move_joints(self.m['home_joints'])

    def move_above(self, obj):
        if obj not in self.scene.objects:
            return Status.INVALID_OBJECT
        if obj == self.world.held:
            return Status.INVALID_STATE
        x, y = self.world.positions[obj]
        return self.backend.move_pose_down((x, y, self.scene.approach_tool_z()))

    def move_to_zone(self, zone, allow_buffer=False):
        if not (zone in self.scene.zones or (allow_buffer and zone in self.scene.buffers)):
            return Status.INVALID_ZONE
        x, y = self.scene.slot_xy(zone)
        return self.backend.move_pose_down((x, y, self.scene.approach_tool_z()))

    def close_gripper(self, obj):
        st = self.backend.attach(obj)
        if st == Status.SUCCESS:
            self.world.held = obj
        return st

    def open_gripper(self, obj, slot):
        x, y = self.scene.slot_xy(slot)
        # Gazebo's SetEntityState teleports the cube instantly (no ROS
        # control loop moves it there); landing it exactly flush with the
        # table (zero gap) makes the ODE physics solver resolve a sudden
        # zero-clearance contact in one step, which can show up as a visible
        # jolt/settle-bounce. A tiny drop height lets it fall those few mm
        # under gravity instead, which the solver handles smoothly.
        drop = 0.0015
        center = (x, y, self.scene.cube_center_z() + drop)
        st = self.backend.detach(obj, center)
        if st == Status.SUCCESS:
            self.world.positions[obj] = (x, y)
            self.world.held = None
        return st

    # ---------------------------------------------------------- main skills
    def pick(self, obj):
        if obj not in self.scene.objects:
            return Status.INVALID_OBJECT
        if self.world.held is not None:
            return Status.INVALID_STATE
        for action in (lambda: self.move_above(obj),
                       lambda: self.backend.move_vertical(self.scene.grasp_tool_z()),
                       lambda: self.close_gripper(obj),
                       lambda: self.backend.move_vertical(self.scene.approach_tool_z())):
            st = action()
            if st != Status.SUCCESS:
                return st
        return Status.SUCCESS

    def place(self, obj, zone, allow_buffer=False):
        if obj not in self.scene.objects:
            return Status.INVALID_OBJECT
        if not (zone in self.scene.zones or (allow_buffer and zone in self.scene.buffers)):
            return Status.INVALID_ZONE
        if self.world.held != obj:
            return Status.INVALID_STATE
        occupant = self.world.occupant(zone, exclude=obj)
        if occupant is not None:           # executor should have cleared it
            return Status.INVALID_STATE
        for action in (lambda: self.move_to_zone(zone, allow_buffer),
                       lambda: self.backend.move_vertical(self.scene.place_tool_z()),
                       lambda: self.open_gripper(obj, zone),
                       lambda: self.backend.move_vertical(self.scene.approach_tool_z())):
            st = action()
            if st != Status.SUCCESS:
                return st
        return Status.SUCCESS
