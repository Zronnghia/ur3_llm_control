"""Unit + integration tests that need no ROS (run with: python3 -m pytest test)."""
import http.server
import json
import os
import threading

import pytest

from ur3_llm_control.check_scene import check
from ur3_llm_control.fake_backend import FakeMotionBackend
from ur3_llm_control.llm_planner import (LLMError, LLMPlanner, OpenAICompatibleClient,
                                         build_system_prompt, extract_json)
from ur3_llm_control.pipeline import CommandPipeline
from ur3_llm_control.robot_skills import RobotSkills
from ur3_llm_control.scene_model import SceneConfig, WorldState
from ur3_llm_control.skill_executor import PlanExpansionError, expand_plan
from ur3_llm_control.skill_spec import Status
from ur3_llm_control.student_task import compute_p, zone_assignment
from ur3_llm_control.task_validator import TaskValidator
from ur3_llm_control.world_gen import generate

CFG = os.path.join(os.path.dirname(__file__), '..', 'config', 'scene.yaml')


@pytest.fixture
def scene():
    return SceneConfig.from_file(CFG)


@pytest.fixture
def world(scene):
    return WorldState(scene)


def P(*steps):
    out = []
    for s in steps:
        name, *args = s
        d = {'skill': name}
        if name in ('pick', 'move_above'):
            d['object'] = args[0]
        elif name == 'place':
            d['object'], d['zone'] = args
        elif name == 'move_to_zone':
            d['zone'] = args[0]
        out.append(d)
    return {'plan': out}


class FakeLLM:
    """Scripted LLM: returns the given answers in order, records prompts."""

    def __init__(self, *answers):
        self.answers = list(answers)
        self.model, self.url = 'fake', 'fake://'
        self.seen = []

    def chat(self, messages):
        self.seen.append(messages)
        a = self.answers.pop(0)
        if isinstance(a, Exception):
            raise a
        return a if isinstance(a, str) else json.dumps(a)


# ================================================================ student
@pytest.mark.parametrize('sid,p,a,b,c', [
    ('23020123', 5, 'blue', 'yellow', 'red'),     # example of the assignment
    ('23020104', 4, 'blue', 'red', 'yellow'),
    ('23020100', 0, 'red', 'yellow', 'blue'),
    ('23020106', 0, 'red', 'yellow', 'blue'),
    ('23020107', 1, 'red', 'blue', 'yellow'),
    ('23020108', 2, 'yellow', 'red', 'blue'),
    ('23020109', 3, 'yellow', 'blue', 'red'),
])
def test_student_mapping(sid, p, a, b, c):
    assert compute_p(sid) == p
    assert zone_assignment(sid) == {'zone_a': f'{a}_cube', 'zone_b': f'{b}_cube',
                                    'zone_c': f'{c}_cube'}


def test_student_id_invalid():
    with pytest.raises(ValueError):
        compute_p('2A')
    assert compute_p(23020199) == 99 % 6      # int accepted


# ================================================================== scene
def test_scene_reachable_both_robots(scene):
    for ur in ('ur3', 'ur3e'):
        problems, home = check(scene, ur)
        assert problems == [], problems
        assert home[2, 2] < -0.99             # home: tool points down


def test_scene_rejects_object_off_table(scene):
    bad = dict(scene.raw)
    bad['objects'] = dict(bad['objects'], red_cube={'color': 'red', 'xy': [0.9, 0],
                                                    'rgba': [1, 0, 0, 1]})
    with pytest.raises(ValueError):
        SceneConfig(bad)


def test_world_file_is_valid_xml(scene):
    import xml.dom.minidom
    doc = xml.dom.minidom.parseString(generate(scene))
    names = {m.getAttribute('name') for m in doc.getElementsByTagName('model')}
    assert {'red_cube', 'yellow_cube', 'blue_cube', 'zone_a', 'zone_b', 'zone_c',
            'work_table'} <= names
    assert 'libgazebo_ros_state.so' in generate(scene)


# ============== geometry invariants MoveIt enforces at run time ============
BASE_MESH_MIN_Z = -0.0008    # lowest vertex of the UR3 / UR3e base collision mesh


def test_held_cube_clears_the_table(scene):
    """A cube held at its resting height must not touch the table box."""
    shrink = float(scene.motion['attached_shrink'])
    grip_offset = scene.motion['grasp_gap'] + scene.cube_size / 2.0
    bottom = scene.grasp_tool_z() - grip_offset - (scene.cube_size - shrink) / 2.0
    clearance = bottom - scene.table_top
    assert clearance > 0, 'held cube rests exactly on the table -> MoveIt sees a collision'
    assert clearance >= float(scene.motion['position_tolerance']), (
        f'clearance {clearance} must exceed position_tolerance')


def _backend_src():
    return open(os.path.join(os.path.dirname(__file__), '..', 'ur3_llm_control',
                             'moveit_backend.py'), encoding='utf-8').read()


def test_backend_adds_floor_below_robot_base():
    """MoveIt's robot model (ur_moveit.launch.py) has NO ground_plane link, so the
    planning scene must contain a floor -- and it must not touch the base."""
    import ast
    src = _backend_src()
    body = src.split('def setup_scene')[1].split('def reset_gazebo')[0]
    assert "_collision_object('floor'" in body, 'setup_scene must add a floor'
    consts = {n.targets[0].id: n.value.value for n in ast.parse(src).body
              if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant)
              and n.targets[0].id.startswith('FLOOR_')}
    floor_top = -consts['FLOOR_GAP']
    assert floor_top < BASE_MESH_MIN_Z, 'floor would overlap the robot base link'
    assert consts['FLOOR_GAP'] < 0.02, 'floor too far below the real ground'


def _include_args(src, launch_name):
    """launch_arguments dict text of the IncludeLaunchDescription of launch_name."""
    marker = f"'/{launch_name}'"
    assert marker in src, f'{launch_name} is not included'
    return src.split(marker)[1].split('}.items())')[0]


def test_sim_launch_moveit_include():
    """ur_moveit.launch.py (humble) declares no `use_fake_hardware`; use_sim_time
    is what selects joint_trajectory_controller, and ur_type is mandatory."""
    src = open(os.path.join(os.path.dirname(__file__), '..', 'launch',
                            'sim.launch.py'), encoding='utf-8').read()
    args = _include_args(src, 'ur_moveit.launch.py')
    assert "'use_fake_hardware'" not in args
    assert "'use_sim_time': 'true'" in args
    assert "'ur_type'" in args
    gz = _include_args(src, 'gazebo.launch.py')
    assert "'world'" in gz


def test_launch_files_compile():
    import py_compile
    d = os.path.join(os.path.dirname(__file__), '..', 'launch')
    for name in ('sim.launch.py', 'llm_robot.launch.py'):
        py_compile.compile(os.path.join(d, name), doraise=True)


# ============================================================== validator
def test_valid_basic_plan(scene, world):
    r = TaskValidator(scene).validate(
        P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_b'), ('home',)), world)
    assert r.ok and len(r.plan) == 3 and not r.warnings


def test_normalises_case_and_spaces(scene, world):
    raw = {'plan': [{'skill': 'Pick', 'object': 'Red Cube'},
                    {'skill': 'place', 'object': 'red-cube', 'zone': 'Zone B'}]}
    r = TaskValidator(scene).validate(raw, world)
    assert r.ok
    assert r.plan[1] == {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'}
    assert r.plan[-1] == {'skill': 'home'} and r.warnings     # home appended


@pytest.mark.parametrize('raw,needle', [
    ([], 'JSON object'),
    ({'steps': []}, 'missing "plan"'),
    ({'plan': []}, 'no plan'),
    ({'plan': [], 'error': 'no green cube'}, 'no green cube'),
    (P(('fly', 'red_cube')), 'not allowed'),
    ({'plan': [{'skill': 'move_joints', 'joints': [0, 1, 2, 3, 4, 5]}]}, 'not allowed'),
    ({'plan': [{'skill': 'home', 'joint_positions': [0] * 6}]}, 'low-level'),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube', 'trajectory': []}]}, 'low-level'),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube', 'speed': 2}]}, 'unexpected'),
    (P(('pick', 'green_cube')), 'INVALID_OBJECT'),
    (P(('pick', 'red')), 'INVALID_OBJECT'),
    (P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_d')), 'INVALID_ZONE'),
    (P(('pick', 'red_cube'), ('place', 'red_cube', 'buffer_1')), 'INVALID_ZONE'),
    ({'plan': [{'skill': 'pick'}]}, 'needs string parameter'),
    ({'plan': [{'skill': 'pick', 'object': 3}]}, 'needs string parameter'),
    ({'plan': ['pick red']}, 'must be an object'),
    (P(('place', 'red_cube', 'zone_a')), 'holds nothing'),
    (P(('pick', 'red_cube'), ('pick', 'blue_cube')), 'already holding'),
    (P(('pick', 'red_cube'), ('place', 'blue_cube', 'zone_a')), 'gripper holds red_cube'),
    (P(('pick', 'red_cube'), ('home',)), 'still holding'),
    (P(('pick', 'red_cube'), ('move_above', 'red_cube')), 'in the gripper'),
    ({'plan': [{'skill': 'home'}] * 21}, 'too long'),
])
def test_validator_rejects(scene, world, raw, needle):
    r = TaskValidator(scene).validate(raw, world)
    assert not r.ok
    assert any(needle in e for e in r.errors), r.errors


# ================================================================ parsing
@pytest.mark.parametrize('text', [
    '{"plan": [{"skill": "home"}]}',
    '```json\n{"plan": [{"skill": "home"}]}\n```',
    'Sure! Here is the plan:\n{"plan": [{"skill": "home"}]}\nHope it helps',
    '<think>{"plan": "draft"}</think>{"plan": [{"skill": "home"}]}',
])
def test_extract_json(text):
    assert extract_json(text) == {'plan': [{'skill': 'home'}]}


def test_extract_json_fails():
    with pytest.raises(ValueError):
        extract_json('I cannot do that')


def test_prompt_contains_whitelist_and_personal_task(scene):
    p = build_system_prompt(scene, 'Nguyen Van An', '23020123')
    for token in ('pick(object)', 'place(object, zone)', 'home()', 'red_cube', 'zone_c',
                  'P = last two digits mod 6 = 5', 'blue_cube -> zone_a',
                  'yellow_cube -> zone_b', 'red_cube -> zone_c'):
        assert token in p
    assert 'buffer' not in p                      # buffers are hidden from the LLM


# ================================================================ planner
def make_planner(scene, llm, attempts=2):
    return LLMPlanner(llm, scene, TaskValidator(scene), 'Nguyen Van An', '23020123',
                      max_attempts=attempts, log=lambda *_: None)


def test_planner_retries_after_rejection(scene, world):
    llm = FakeLLM(P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_9')),
                  P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_b'), ('home',)))
    r, _ = make_planner(scene, llm).plan('put red in B', world)
    assert r.ok and len(llm.seen) == 2
    assert 'rejected by the validator' in llm.seen[1][-1]['content']
    assert 'Do NOT substitute' in llm.seen[1][-1]['content']


def test_planner_gives_up(scene, world):
    llm = FakeLLM('not json', 'still not json')
    r, _ = make_planner(scene, llm).plan('dance', world)
    assert not r.ok and len(llm.seen) == 2


def test_planner_llm_refusal_is_final(scene, world):
    llm = FakeLLM({'plan': [], 'error': 'there is no green cube'}, 'SHOULD NOT BE USED')
    r, _ = make_planner(scene, llm).plan('move the green cube', world)
    assert not r.ok and len(llm.seen) == 1 and 'green' in r.errors[0]


def test_user_prompt_contains_state(scene, world):
    llm = FakeLLM(P(('home',)))
    make_planner(scene, llm).plan('go home', world)
    assert 'red_cube: initial position' in llm.seen[0][1]['content']


# ========================================================= expand / executor
def test_expand_inserts_buffer_when_zone_occupied(scene, world):
    world.positions['yellow_cube'] = scene.slot_xy('zone_a')        # A occupied
    plan = TaskValidator(scene).validate(
        P(('pick', 'blue_cube'), ('place', 'blue_cube', 'zone_a'), ('home',)), world).plan
    steps = expand_plan(plan, world)
    assert steps[0] == {'skill': 'pick', 'object': 'yellow_cube', 'auto': True}
    assert steps[1]['zone'] == 'buffer_1' and steps[1]['auto']
    assert steps[2:] == plan


def test_reorder_avoids_buffer(scene, world):
    """red sits in B; plan asks yellow->B first, then red->C: run red->C first."""
    world.positions['red_cube'] = scene.slot_xy('zone_b')
    plan = P(('pick', 'yellow_cube'), ('place', 'yellow_cube', 'zone_b'),
             ('pick', 'red_cube'), ('place', 'red_cube', 'zone_c'), ('home',))['plan']
    steps = expand_plan(plan, world)
    assert not any(s.get('auto') for s in steps)
    assert [s.get('object') for s in steps] == ['red_cube', 'red_cube', 'yellow_cube',
                                                'yellow_cube', None]


def test_reorder_keeps_non_pair_plans(scene, world):
    plan = P(('move_above', 'red_cube'), ('pick', 'red_cube'),
             ('place', 'red_cube', 'zone_a'), ('home',))['plan']
    assert expand_plan(plan, world) == plan


def test_expand_skips_object_already_in_place(scene, world):
    world.positions['red_cube'] = scene.slot_xy('zone_c')
    plan = P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_c'), ('home',))['plan']
    steps = expand_plan(plan, world)
    assert steps[0]['skip'] and steps[1]['skip'] and not steps[2].get('skip')


def test_expand_no_free_buffer(scene, world):
    world.positions['red_cube'] = scene.slot_xy('buffer_1')
    world.positions['yellow_cube'] = scene.slot_xy('buffer_2')
    world.positions['blue_cube'] = scene.slot_xy('zone_a')
    plan = P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_a'), ('home',))['plan']
    with pytest.raises(PlanExpansionError):
        expand_plan(plan, world)


def run_pipeline(scene, world, answer, command='cmd', fail_on=None):
    backend = FakeMotionBackend(scene, fail_on=fail_on)
    skills = RobotSkills(scene, backend, world)
    lines = []
    answers = answer if isinstance(answer, tuple) else (answer,)
    pipe = CommandPipeline(make_planner(scene, FakeLLM(*answers)), skills, world,
                           log=lines.append)
    return pipe.run(command), '\n'.join(str(x) for x in lines), backend


def test_pipeline_basic_output_format(scene, world):
    rep, text, backend = run_pipeline(
        scene, world, P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_b'), ('home',)),
        'Put the red cube in zone B.')
    assert rep['status'] == 'TASK SUCCESS'
    for token in ('USER COMMAND:', 'Put the red cube in zone B.', 'LLM PLAN:',
                  'pick(red_cube)', 'place(red_cube, zone_b)', 'home()', 'EXECUTION:',
                  'TASK SUCCESS'):
        assert token in text
    assert text.count('SUCCESS') >= 4
    assert world.slot_of('red_cube') == 'zone_b' and world.held is None
    assert backend.attached is None
    kinds = [c[0] for c in backend.calls]
    assert kinds == ['move_pose_down', 'move_vertical', 'attach', 'move_vertical',
                     'move_pose_down', 'move_vertical', 'detach', 'move_vertical',
                     'move_joints']


def test_pipeline_advanced_arrange_by_student_id(scene, world):
    """23020123 -> P=5 -> A=blue, B=yellow, C=red."""
    answer = P(('pick', 'blue_cube'), ('place', 'blue_cube', 'zone_a'),
               ('pick', 'yellow_cube'), ('place', 'yellow_cube', 'zone_b'),
               ('pick', 'red_cube'), ('place', 'red_cube', 'zone_c'), ('home',))
    rep, text, _ = run_pipeline(scene, world, answer,
                                'Arrange all objects according to my student ID.')
    assert rep['status'] == 'TASK SUCCESS', text
    assert {z: world.occupant(z) for z in scene.zones} == zone_assignment('23020123')


def test_pipeline_swap_uses_buffer(scene, world):
    """red in A, blue in B -> ask red->B and blue->A (cyclic conflict)."""
    world.positions['red_cube'] = scene.slot_xy('zone_a')
    world.positions['blue_cube'] = scene.slot_xy('zone_b')
    answer = P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_b'),
               ('pick', 'blue_cube'), ('place', 'blue_cube', 'zone_a'), ('home',))
    rep, text, _ = run_pipeline(scene, world, answer, 'swap red and blue')
    assert rep['status'] == 'TASK SUCCESS', text
    assert '[auto]' in text
    assert world.occupant('zone_b') == 'red_cube'
    assert world.occupant('zone_a') == 'blue_cube'


def test_pipeline_rejects_invalid_plan_without_moving(scene, world):
    rep, text, backend = run_pipeline(
        scene, world, ({'plan': [{'skill': 'set_joints', 'joints': [0] * 6}]},
                       {'plan': [{'skill': 'home', 'joint_positions': [0] * 6}]}))
    assert rep['status'] == 'TASK REJECTED'
    assert 'low-level robot control is forbidden' in text
    assert 'nothing was executed' in text
    assert backend.calls == []


def test_pipeline_planning_failure_stops(scene, world):
    rep, text, backend = run_pipeline(
        scene, world, P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_b'), ('home',)),
        fail_on={'move_pose_down'})
    assert rep['status'] == 'TASK FAILED'
    assert rep['steps'][0][1] == Status.PLANNING_FAILED
    assert rep['steps'][1][1] == 'NOT EXECUTED' and rep['steps'][2][1] == 'NOT EXECUTED'
    assert world.held is None


def test_pipeline_llm_unreachable(scene, world):
    rep, text, _ = run_pipeline(scene, world, LLMError('cannot reach 9Router'))
    assert rep['status'] == 'TASK REJECTED' and 'LLM ERROR' in text


def test_skills_guard_invalid_inputs(scene, world):
    sk = RobotSkills(scene, FakeMotionBackend(scene), world)
    assert sk.pick('green_cube') == Status.INVALID_OBJECT
    assert sk.place('red_cube', 'zone_z') == Status.INVALID_ZONE
    assert sk.place('red_cube', 'zone_a') == Status.INVALID_STATE     # not held
    assert sk.place('red_cube', 'buffer_1') == Status.INVALID_ZONE    # LLM cannot use buffers
    assert sk.pick('red_cube') == Status.SUCCESS
    assert sk.pick('blue_cube') == Status.INVALID_STATE               # hand full
    assert sk.move_above('red_cube') == Status.INVALID_STATE          # it is in the hand


def test_unreachable_target_is_planning_failed(scene, world):
    world.positions['red_cube'] = (0.9, 0.0)                          # out of reach
    sk = RobotSkills(scene, FakeMotionBackend(scene), world)
    assert sk.pick('red_cube') == Status.PLANNING_FAILED


# ============================================= real HTTP client (mock 9Router)
class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        self.server.last = (self.path, self.headers.get('Authorization'), body)
        if self.headers.get('Authorization') != 'Bearer good-key':
            self.send_response(401)
            self.end_headers()
            self.wfile.write(b'{"error":"unauthorized"}')
            return
        content = '```json\n{"plan":[{"skill":"home"}]}\n```'
        answer = {'choices': [{'message': {'role': 'assistant', 'content': content}}]}
        data = json.dumps(answer).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


@pytest.fixture
def mock_router():
    srv = http.server.HTTPServer(('127.0.0.1', 0), _Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()


def test_http_client_ok(mock_router):
    url = f'http://127.0.0.1:{mock_router.server_port}/v1'
    c = OpenAICompatibleClient(url, 'good-key', 'kr/claude-sonnet-4.5')
    text = c.chat([{'role': 'user', 'content': 'hi'}])
    assert extract_json(text) == {'plan': [{'skill': 'home'}]}
    path, auth, body = mock_router.last
    assert path == '/v1/chat/completions' and body['model'] == 'kr/claude-sonnet-4.5'
    assert body['temperature'] == 0.0


def test_http_client_bad_key(mock_router):
    c = OpenAICompatibleClient(f'http://127.0.0.1:{mock_router.server_port}/v1', 'bad', 'm')
    with pytest.raises(LLMError, match='401'):
        c.chat([{'role': 'user', 'content': 'hi'}])


def test_http_client_unreachable():
    c = OpenAICompatibleClient('http://127.0.0.1:9/v1', 'k', 'm', timeout_s=2)
    with pytest.raises(LLMError, match='cannot reach'):
        c.chat([{'role': 'user', 'content': 'hi'}])


def test_http_client_ignores_proxy_for_localhost(mock_router, monkeypatch):
    """A local 9Router must be reached directly even if http_proxy is set."""
    monkeypatch.setenv('http_proxy', 'http://127.0.0.1:9')      # dead proxy
    monkeypatch.setenv('HTTP_PROXY', 'http://127.0.0.1:9')
    monkeypatch.delenv('no_proxy', raising=False)
    monkeypatch.delenv('NO_PROXY', raising=False)
    c = OpenAICompatibleClient(f'http://localhost:{mock_router.server_port}/v1',
                               'good-key', 'gemini/gemini-2.5-flash')
    assert extract_json(c.chat([{'role': 'user', 'content': 'hi'}]))['plan']


def test_config_requires_model_and_key(monkeypatch):
    from ur3_llm_control.llm_planner import make_client_from_config
    for v in ('NINEROUTER_API_KEY', 'NINEROUTER_MODEL', 'NINEROUTER_BASE_URL'):
        monkeypatch.delenv(v, raising=False)
    with pytest.raises(LLMError, match='API key'):
        make_client_from_config({'model': 'gemini/gemini-2.5-flash', 'api_key': ''})
    monkeypatch.setenv('NINEROUTER_API_KEY', 'sk-env')
    c = make_client_from_config({'model': 'gemini/gemini-2.5-flash', 'api_key': ''})
    assert c.api_key == 'sk-env' and c.model == 'gemini/gemini-2.5-flash'


def test_shipped_config_uses_gemini_and_has_no_secret():
    import yaml
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config',
                                           'student_config.yaml'), encoding='utf-8'))
    assert '/' in cfg['llm']['model'] and cfg['llm']['model'], 'llm.model must be "<provider>/<name>"'
    assert not cfg['llm']['api_key'], 'do not ship a 9Router key in the package'
    compute_p(cfg['student_id'])
