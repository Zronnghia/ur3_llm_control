"""The ONLY skills the LLM may use (whitelist) and their parameters.

Shared by the prompt builder, the validator and the executor so the three can
never disagree.
"""
from collections import OrderedDict

SKILLS = OrderedDict([
    ('home', {'params': [], 'doc': 'move the arm to the safe home configuration'}),
    ('pick', {'params': ['object'],
              'doc': 'approach, grasp and lift the object (gripper must be empty)'}),
    ('place', {'params': ['object', 'zone'],
               'doc': 'put the currently held object into the zone and release it'}),
    ('move_above', {'params': ['object'],
                    'doc': 'move the tool above an object (no grasp)'}),
    ('move_to_zone', {'params': ['zone'],
                      'doc': 'move the tool above a zone (no release)'}),
])

MAX_PLAN_STEPS = 20


class Status:
    SUCCESS = 'SUCCESS'
    SKIPPED = 'SKIPPED'
    FAILED = 'FAILED'
    INVALID_OBJECT = 'INVALID_OBJECT'
    INVALID_ZONE = 'INVALID_ZONE'
    INVALID_STATE = 'INVALID_STATE'
    PLANNING_FAILED = 'PLANNING_FAILED'
    EXECUTION_FAILED = 'EXECUTION_FAILED'

    OK = (SUCCESS, SKIPPED)


def step_to_str(step: dict) -> str:
    params = SKILLS.get(step.get('skill'), {}).get('params', [])
    args = ', '.join(str(step.get(p)) for p in params)
    return f"{step.get('skill')}({args})"
