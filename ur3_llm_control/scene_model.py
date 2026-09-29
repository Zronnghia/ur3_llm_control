"""Scene configuration + world state (where each cube is, what is held).

Pure python – no ROS import – so it can be unit-tested anywhere.
"""
import copy
import math

import yaml


class SceneConfig:
    def __init__(self, data: dict):
        self.raw = data
        self.frame_id = data.get('frame_id', 'world')
        self.ur_type = data.get('ur_type', 'ur3e')
        self.planning_group = data.get('planning_group', 'ur_manipulator')
        self.ee_link = data.get('ee_link', 'tool0')
        self.table = data['table']
        self.table_top = float(self.table['height'])
        self.cube_size = float(data.get('cube_size', 0.04))
        self.objects = data['objects']
        self.zones = data['zones']
        self.buffers = data.get('buffer_slots', {}) or {}
        self.motion = data['motion']
        self._check()

    @classmethod
    def from_file(cls, path):
        with open(path, 'r', encoding='utf-8') as f:
            return cls(yaml.safe_load(f))

    def _check(self):
        names = list(self.objects) + list(self.zones) + list(self.buffers)
        if len(names) != len(set(names)):
            raise ValueError('object / zone / buffer names must be unique')
        tx, ty = self.table['center']
        sx, sy = self.table['size']
        for name, xy in self.all_locations().items():
            if not (tx - sx / 2 <= xy[0] <= tx + sx / 2 and ty - sy / 2 <= xy[1] <= ty + sy / 2):
                raise ValueError(f'{name} at {xy} is outside the table')
        if len(self.motion['home_joints']) != 6:
            raise ValueError('motion.home_joints needs 6 values')

    # -------------------------------------------------------------- helpers
    @property
    def object_names(self):
        return list(self.objects.keys())

    @property
    def zone_names(self):
        return list(self.zones.keys())

    def all_locations(self):
        locs = {n: tuple(o['xy']) for n, o in self.objects.items()}
        locs.update({n: tuple(z['xy']) for n, z in self.zones.items()})
        locs.update({n: tuple(b['xy']) for n, b in self.buffers.items()})
        return locs

    def slot_xy(self, slot):
        if slot in self.zones:
            return tuple(self.zones[slot]['xy'])
        if slot in self.buffers:
            return tuple(self.buffers[slot]['xy'])
        raise KeyError(slot)

    def cube_center_z(self):
        return self.table_top + self.cube_size / 2.0

    def grasp_tool_z(self):
        """tool0 height when 'gripping' a cube resting on the table."""
        return self.table_top + self.cube_size + float(self.motion['grasp_gap'])

    def place_tool_z(self):
        return self.grasp_tool_z() + float(self.motion['place_clearance'])

    def approach_tool_z(self):
        return self.grasp_tool_z() + float(self.motion['approach_height'])


class WorldState:
    """Symbolic state tracked by the executor (positions are ground truth
    for the simulator as well: cubes are only moved by our skills)."""

    def __init__(self, scene: SceneConfig):
        self.scene = scene
        self.positions = {n: tuple(o['xy']) for n, o in scene.objects.items()}
        self.held = None

    def copy(self):
        return copy.deepcopy(self)

    def slot_of(self, obj):
        """zone/buffer name the object currently sits in, else None."""
        if obj == self.held:
            return None
        ox, oy = self.positions[obj]
        for slot in list(self.scene.zones) + list(self.scene.buffers):
            sx, sy = self.scene.slot_xy(slot)
            if math.hypot(ox - sx, oy - sy) < 0.02:
                return slot
        return None

    def occupant(self, slot, exclude=None):
        for obj in self.positions:
            if obj != exclude and obj != self.held and self.slot_of(obj) == slot:
                return obj
        return None

    def free_buffer(self):
        for b in self.scene.buffers:
            if self.occupant(b) is None:
                return b
        return None

    def summary(self):
        out = {}
        for obj in self.positions:
            if obj == self.held:
                out[obj] = 'held by gripper'
            else:
                out[obj] = self.slot_of(obj) or 'initial position'
        return out
