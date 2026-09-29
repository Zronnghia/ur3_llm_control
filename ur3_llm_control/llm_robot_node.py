"""ROS 2 node: natural-language control of the UR3/UR3e.

  ros2 run ur3_llm_control llm_robot_node                 # interactive prompt
  ros2 run ur3_llm_control llm_robot_node --ros-args -p command:="Put the red cube in zone B."
  ros2 run ur3_llm_control send_command "Move the blue cube to zone C."   # via topic

Topics
  sub  /llm_robot/command  std_msgs/String   natural-language command
  pub  /llm_robot/plan     std_msgs/String   validated JSON plan
  pub  /llm_robot/result   std_msgs/String   JSON execution report
  pub  /llm_robot/markers  visualization_msgs/MarkerArray   zone labels for RViz
"""
import json
import os
import queue
import sys
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .fake_backend import FakeMotionBackend
from .llm_planner import LLMError, LLMPlanner, make_client_from_config
from .pipeline import CommandPipeline
from .robot_skills import RobotSkills
from .scene_model import SceneConfig, WorldState
from .skill_spec import Status
from .student_task import describe
from .task_validator import TaskValidator


def _share(*parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ur3_llm_control'), *parts)


def out(msg=''):
    print(msg, flush=True)


class LLMRobotNode(Node):
    def __init__(self):
        super().__init__('llm_robot_node')
        self.declare_parameter('scene_file', _share('config', 'scene.yaml'))
        self.declare_parameter('student_file', _share('config', 'student_config.yaml'))
        self.declare_parameter('backend', 'moveit')        # moveit | fake
        self.declare_parameter('use_gazebo', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('command', '')
        self.declare_parameter('interactive', True)
        self.declare_parameter('home_on_start', True)
        self.declare_parameter('llm_model', '')             # overrides student_config.yaml

        gp = self.get_parameter
        self.scene = SceneConfig.from_file(gp('scene_file').value)
        with open(gp('student_file').value, 'r', encoding='utf-8') as f:
            student = yaml.safe_load(f)
        llm_cfg = dict(student.get('llm', {}))
        if gp('llm_model').value:
            llm_cfg['model'] = gp('llm_model').value

        self.cb = ReentrantCallbackGroup()
        self.world = WorldState(self.scene)
        if gp('backend').value == 'fake':
            self.backend = FakeMotionBackend(self.scene)
        else:
            from .moveit_backend import MoveItBackend
            self.backend = MoveItBackend(self, self.scene, self.cb, gp('use_gazebo').value)
        self.skills = RobotSkills(self.scene, self.backend, self.world)
        validator = TaskValidator(self.scene)
        client = make_client_from_config(llm_cfg)
        self.planner = LLMPlanner(client, self.scene, validator, student['student_name'],
                                  student['student_id'], llm_cfg.get('max_attempts', 2), log=out)
        self.pipeline = CommandPipeline(self.planner, self.skills, self.world, log=out)
        self.student_info = describe(student['student_name'], student['student_id'])
        self.llm_desc = f"{client.model} @ {client.url}"

        self.commands = queue.Queue()
        self.plan_pub = self.create_publisher(String, '/llm_robot/plan', 10)
        self.result_pub = self.create_publisher(String, '/llm_robot/result', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/llm_robot/markers', 1)
        self.create_subscription(String, '/llm_robot/command',
                                 lambda m: self.commands.put(m.data), 10,
                                 callback_group=self.cb)
        self.create_timer(1.0, self._publish_markers, callback_group=self.cb)

    # --------------------------------------------------------------- setup
    def prepare_robot(self):
        if isinstance(self.backend, FakeMotionBackend):
            return True
        out('[setup] waiting for MoveIt 2 (move_group) ...')
        missing = self.backend.wait_for_servers(timeout=180.0)
        if missing:
            out(f'[setup] ERROR: not available: {missing}. Is sim.launch.py running?')
            return False
        if not self.backend.setup_scene(self.world):
            out('[setup] ERROR: could not update the MoveIt planning scene')
            return False
        if self.get_parameter('use_gazebo').value:
            self.backend.reset_gazebo(self.world)
        out('[setup] planning scene ready (floor, table, 3 cubes)')
        if self.get_parameter('home_on_start').value:
            st = self.skills.home()
            out(f'[setup] home() ... {st}')
            if st != Status.SUCCESS:
                return False
        return True

    def _publish_markers(self):
        arr = MarkerArray()
        for i, (name, z) in enumerate(self.scene.zones.items()):
            m = Marker()
            m.header.frame_id = self.scene.frame_id
            m.ns, m.id, m.type, m.action = 'zones', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y = map(float, z['xy'])
            m.pose.position.z = self.scene.table_top + 0.12
            m.pose.orientation.w = 1.0
            m.scale.z = 0.03
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = z['label']
            arr.markers.append(m)
            p = Marker()
            p.header.frame_id = self.scene.frame_id
            p.ns, p.id, p.type, p.action = 'zone_pads', i, Marker.CUBE, Marker.ADD
            p.pose.position.x, p.pose.position.y = map(float, z['xy'])
            p.pose.position.z = self.scene.table_top + 0.001
            p.pose.orientation.w = 1.0
            p.scale.x = p.scale.y = float(z['size'])
            p.scale.z = 0.002
            p.color.r, p.color.g, p.color.b, p.color.a = map(float, z['rgba'])
            arr.markers.append(p)
        self.marker_pub.publish(arr)

    # ----------------------------------------------------------- run a cmd
    def handle_command(self, command):
        report = self.pipeline.run(command, dry_run=self.get_parameter('dry_run').value)
        if report.get('plan'):
            self.plan_pub.publish(String(data=json.dumps({'plan': report['plan']})))
        self.result_pub.publish(String(data=json.dumps(report, ensure_ascii=False,
                                                       default=str)))
        return report


def _stdin_reader(q, stop, ready):
    while not stop.is_set():
        ready.wait()          # do not prompt while a command is being executed
        ready.clear()
        try:
            line = input('\nCommand> ')
        except (EOFError, KeyboardInterrupt):
            q.put('quit')
            return
        if line.strip():
            q.put(line.strip())
        else:
            ready.set()       # empty line: prompt again


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LLMRobotNode()
    except (LLMError, OSError, KeyError, ValueError) as e:
        print(f'[llm_robot_node] configuration error: {e}', file=sys.stderr)
        rclpy.shutdown()
        return 1
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    out('=' * 60)
    out('UR3 LLM CONTROL')
    out(node.student_info)
    out(f'LLM (9Router): {node.llm_desc}')
    out('=' * 60)
    code = 0
    stop = threading.Event()
    ready = threading.Event()
    ready.set()
    try:
        if not node.prepare_robot():
            code = 1
        else:
            one_shot = node.get_parameter('command').value
            if one_shot:
                rep = node.handle_command(one_shot)
                code = 0 if rep['status'] in ('TASK SUCCESS', 'DRY RUN') else 2
            else:
                if node.get_parameter('interactive').value and sys.stdin.isatty():
                    out("Type a command ('state' shows the world state, 'quit' exits). "
                        "Commands on /llm_robot/command are accepted too.")
                    threading.Thread(target=_stdin_reader, args=(node.commands, stop, ready),
                                     daemon=True).start()
                else:
                    out('Waiting for commands on /llm_robot/command ...')
                while rclpy.ok():
                    try:
                        cmd = node.commands.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    if cmd.lower() in ('quit', 'exit', 'q'):
                        break
                    if cmd.lower() == 'state':
                        out(json.dumps(node.world.summary(), indent=2))
                    else:
                        node.handle_command(cmd)
                    ready.set()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
