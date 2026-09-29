"""LLM task planner: natural language -> structured JSON plan.

The LLM is reached through 9Router (OpenAI-compatible /v1/chat/completions).
The LLM only chooses and orders skills from the whitelist in skill_spec.py; it
never sees joint names, joint values or trajectories.
"""
import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request

from .skill_spec import SKILLS
from .student_task import compute_p, zone_assignment
from .task_validator import ValidationResult


class LLMError(RuntimeError):
    pass


# --------------------------------------------------------------------- client
class OpenAICompatibleClient:
    """Minimal client for 9Router (no extra pip dependency)."""

    def __init__(self, base_url, api_key='', model='', temperature=0.0, timeout_s=60):
        self.url = base_url.rstrip('/') + '/chat/completions'
        self.api_key = api_key
        self.model = model
        self.temperature = temperature
        self.timeout_s = timeout_s
        # A local 9Router must never be reached through http(s)_proxy (common
        # on university / company networks and in some WSL setups).
        host = (urllib.parse.urlparse(self.url).hostname or '').lower()
        if host in ('localhost', '127.0.0.1', '::1'):
            self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        else:
            self._opener = urllib.request.build_opener()

    def chat(self, messages):
        body = {'model': self.model, 'messages': messages,
                'temperature': self.temperature, 'stream': False}
        req = urllib.request.Request(
            self.url, data=json.dumps(body).encode('utf-8'), method='POST',
            headers={'Content-Type': 'application/json',
                     'Authorization': f'Bearer {self.api_key}'})
        try:
            with self._opener.open(req, timeout=self.timeout_s) as resp:
                data = json.loads(resp.read().decode('utf-8'))
        except urllib.error.HTTPError as e:
            detail = e.read().decode('utf-8', 'replace')[:300]
            hint = ''
            if e.code == 401:
                hint = (' -> check the 9Router API key (dashboard > Endpoint; '
                        'env NINEROUTER_API_KEY or llm.api_key)')
            elif e.code in (400, 404) and 'model' in detail.lower():
                hint = f' -> model "{self.model}" is not available in 9Router'
            elif e.code == 429:
                hint = ' -> provider rate limit / quota: wait a minute or use another model'
            raise LLMError(f'9Router HTTP {e.code}: {detail}{hint}') from e
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise LLMError(f'cannot reach 9Router at {self.url}: {e}') from e
        except json.JSONDecodeError as e:
            raise LLMError(f'9Router returned non-JSON response: {e}') from e
        try:
            return data['choices'][0]['message']['content'] or ''
        except (KeyError, IndexError, TypeError) as e:
            raise LLMError(f'unexpected 9Router response: {str(data)[:300]}') from e


def extract_json(text: str):
    """Parse the first JSON object in an LLM answer (tolerates ```json fences
    and <think> blocks some models emit)."""
    text = re.sub(r'<think>.*?</think>', '', text, flags=re.S)
    fence = re.search(r'```(?:json)?\s*(\{.*?\})\s*```', text, flags=re.S)
    if fence:
        text = fence.group(1)
    start = text.find('{')
    if start < 0:
        raise ValueError('no JSON object in LLM answer')
    obj, _ = json.JSONDecoder().raw_decode(text[start:])
    return obj


# --------------------------------------------------------------------- prompt
def build_system_prompt(scene, student_name, student_id):
    skills = '\n'.join(
        f"  - {name}({', '.join(s['params'])}): {s['doc']}" for name, s in SKILLS.items())
    objects = '\n'.join(f"  - {n} (colour: {o['color']})" for n, o in scene.objects.items())
    zones = '\n'.join(f"  - {n} (\"{z['label']}\")" for n, z in scene.zones.items())
    assign = zone_assignment(student_id)
    p = compute_p(student_id)
    personal = '\n'.join(f'  - {obj} -> {zone}' for zone, obj in assign.items())
    robot = 'UR3e' if scene.ur_type == 'ur3e' else 'UR3'
    example = ('{"plan": [{"skill": "pick", "object": "red_cube"}, '
               '{"skill": "place", "object": "red_cube", "zone": "zone_b"}, {"skill": "home"}]}')
    return f"""You are the task planner of a {robot} robot arm that manipulates cubes on a table.
You translate the user's command (English or Vietnamese) into an ordered list of robot skills.

Available skills (the ONLY ones you may use):
{skills}

Objects:
{objects}

Zones:
{zones}

Student: {student_name}, student ID {student_id}. P = last two digits mod 6 = {p}.
"Arrange according to my student ID" (or similar, e.g. "sắp xếp theo MSSV") means:
{personal}

Rules:
1. Output ONLY one JSON object, no prose, no markdown:
   {example}
2. Use exactly the skill, object and zone identifiers listed above.
3. Every place(object, zone) must be preceded by pick(object);
   the gripper holds one object at a time.
4. End every plan with {{"skill": "home"}}.
5. Never output joint values, poses, trajectories or any parameter not listed above.
6. Do not handle occupied zones yourself; the executor will clear a zone automatically if needed.
7. Objects already in their requested zone (see current state) should be skipped.
8. If the command is unclear, asks for an object/zone that does not exist, or is not a
   manipulation task, return {{"plan": [], "error": "<short reason>"}}.
"""


def build_user_prompt(command, world):
    state = ', '.join(f'{o}: {where}' for o, where in world.summary().items())
    return f'Current state: {state}.\nCommand: {command}'


# -------------------------------------------------------------------- planner
class LLMPlanner:
    def __init__(self, client, scene, validator, student_name, student_id,
                 max_attempts=2, log=print):
        self.client = client
        self.scene = scene
        self.validator = validator
        self.system_prompt = build_system_prompt(scene, student_name, student_id)
        self.max_attempts = max(1, int(max_attempts))
        self.log = log

    def plan(self, command, world):
        """Returns (ValidationResult, raw_text_of_last_answer).

        Raises LLMError only for transport problems."""
        messages = [{'role': 'system', 'content': self.system_prompt},
                    {'role': 'user', 'content': build_user_prompt(command, world)}]
        result, answer = None, ''
        for attempt in range(1, self.max_attempts + 1):
            answer = self.client.chat(messages)
            try:
                raw = extract_json(answer)
            except ValueError as e:
                raw = None
                result = ValidationResult(False, errors=[f'LLM output is not valid JSON: {e}'])
            if raw is not None:
                result = self.validator.validate(raw, world)
            if result.ok:
                return result, answer
            self.log(f'[validator] attempt {attempt}/{self.max_attempts} REJECTED: '
                     + '; '.join(result.errors))
            # Refusals of the LLM itself (empty plan) are final.
            if raw is not None and isinstance(raw, dict) and raw.get('plan') == []:
                break
            messages += [{'role': 'assistant', 'content': answer},
                         {'role': 'user', 'content':
                          'Your plan was rejected by the validator: '
                          + '; '.join(result.errors)
                          + '. Return a corrected JSON plan using only the allowed '
                            'skills, objects and zones. Do NOT substitute a different '
                            'object or zone for the one the user asked for: if the '
                            'command cannot be satisfied, return {"plan": [], '
                            '"error": "<reason>"}.'}]
        return result, answer


def make_client_from_config(llm_cfg: dict):
    api_key = os.environ.get('NINEROUTER_API_KEY') or llm_cfg.get('api_key', '')
    base_url = os.environ.get('NINEROUTER_BASE_URL') or llm_cfg.get(
        'base_url', 'http://localhost:20128/v1')
    model = os.environ.get('NINEROUTER_MODEL') or llm_cfg.get('model', '')
    if not model:
        raise LLMError('no LLM model configured (student_config.yaml llm.model '
                       'or env NINEROUTER_MODEL)')
    if not api_key:
        raise LLMError('no 9Router API key: export NINEROUTER_API_KEY="sk-..." '
                       '(copy it from the 9Router dashboard > Endpoint) '
                       'or set llm.api_key in student_config.yaml')
    return OpenAICompatibleClient(base_url, api_key, model,
                                  float(llm_cfg.get('temperature', 0.0)),
                                  float(llm_cfg.get('timeout_s', 60)))
