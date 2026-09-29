"""Test the LLM part WITHOUT ROS / Gazebo (real 9Router, simulated arm).

    python3 -m ur3_llm_control.offline_cli "Move the blue cube to zone C."
    python3 -m ur3_llm_control.offline_cli            # interactive

The arm is the FakeMotionBackend (numerical IK reachability only), so this is
a quick way to check prompts, validator and executor logic on any machine.
"""
import argparse
import sys

import yaml

from .fake_backend import FakeMotionBackend
from .paths import config_path
from .llm_planner import LLMError, LLMPlanner, make_client_from_config
from .pipeline import CommandPipeline
from .robot_skills import RobotSkills
from .scene_model import SceneConfig, WorldState
from .student_task import describe
from .task_validator import TaskValidator


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('command', nargs='*')
    ap.add_argument('--scene', default=config_path('scene.yaml'))
    ap.add_argument('--student', default=config_path('student_config.yaml'))
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args(argv)

    scene = SceneConfig.from_file(a.scene)
    with open(a.student, encoding='utf-8') as f:
        student = yaml.safe_load(f)
    world = WorldState(scene)
    skills = RobotSkills(scene, FakeMotionBackend(scene), world)
    try:
        client = make_client_from_config(student.get('llm', {}))
    except LLMError as e:
        print(e)
        return 1
    planner = LLMPlanner(client, scene, TaskValidator(scene), student['student_name'],
                         student['student_id'], student.get('llm', {}).get('max_attempts', 2))
    pipe = CommandPipeline(planner, skills, world)
    print(describe(student['student_name'], student['student_id']))
    if a.command:
        rep = pipe.run(' '.join(a.command), dry_run=a.dry_run)
        return 0 if rep['status'] in ('TASK SUCCESS', 'DRY RUN') else 2
    while True:
        try:
            cmd = input('\nCommand> ').strip()
        except (EOFError, KeyboardInterrupt):
            return 0
        if cmd in ('quit', 'exit', 'q'):
            return 0
        if cmd:
            pipe.run(cmd, dry_run=a.dry_run)


if __name__ == '__main__':
    sys.exit(main())
