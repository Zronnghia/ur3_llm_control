"""Offline sanity check of scene.yaml (no ROS needed).

    python3 -m ur3_llm_control.check_scene [path/to/scene.yaml]
    ros2 run ur3_llm_control check_scene

Verifies that every pick / place / approach pose is reachable with the tool
pointing down and inside joint limits, that slots do not overlap and that
home is a tool-down pose above the table.  MoveIt still does the real
(collision-aware) planning at run time; this only catches bad layouts early.
"""
import itertools
import math
import sys

import numpy as np

from .paths import config_path
from .scene_model import SceneConfig
from .ur_kinematics import URKinematics


def default_scene_path():
    return config_path('scene.yaml')


def check(scene: SceneConfig, ur_type=None):
    problems = []
    kin = URKinematics(ur_type or scene.ur_type)
    heights = {'grasp': scene.grasp_tool_z(), 'place': scene.place_tool_z(),
               'approach': scene.approach_tool_z()}
    for name, (x, y) in scene.all_locations().items():
        for hname, z in heights.items():
            if kin.ik_down((x, y, z)) is None:
                problems.append(f'{name}: {hname} pose ({x:.3f}, {y:.3f}, {z:.3f}) unreachable')
        r = math.hypot(x, y)
        if r < 0.15:
            problems.append(f'{name}: too close to the robot base (r={r:.3f} m)')
    slots = list(scene.zones) + list(scene.buffers) + list(scene.objects)
    min_gap = scene.cube_size * 1.8
    for a, b in itertools.combinations(slots, 2):
        (ax, ay), (bx, by) = scene.all_locations()[a], scene.all_locations()[b]
        if math.hypot(ax - bx, ay - by) < min_gap:
            problems.append(f'{a} and {b} are closer than {min_gap:.3f} m')
    t = kin.fk(scene.motion['home_joints'])
    if t[2, 2] > -0.95:
        problems.append('home pose: tool is not pointing down')
    if t[2, 3] < scene.approach_tool_z():
        problems.append('home pose is lower than the approach height')
    tx, ty = scene.table['center']
    sx, _ = scene.table['size']
    if tx - sx / 2 < 0.09:
        problems.append('table overlaps the robot base (x_min < 0.09 m)')
    return problems, t


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else default_scene_path()
    scene = SceneConfig.from_file(path)
    np.set_printoptions(precision=3, suppress=True)
    all_ok = True
    for ur in ('ur3', 'ur3e'):
        problems, home = check(scene, ur)
        print(f'[{ur}] home tool0 position: {home[:3, 3]}')
        for p in problems:
            print(f'[{ur}] PROBLEM: {p}')
        print(f'[{ur}] ' + ('OK' if not problems else f'{len(problems)} problem(s)'))
        all_ok &= not problems if ur == scene.ur_type else True
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
