"""Plan validator: the safety gate between the LLM and the robot.

Checks, in order:
 1. syntax      – top level is {"plan": [...]}, each step is an object
 2. whitelist   – skill in SKILLS, exactly the required parameters, no extra
                  keys (e.g. "joints", "trajectory" are rejected)
 3. symbols     – object / zone names exist in scene.yaml
 4. semantics   – simulated on a copy of the world state: cannot pick with a
                  full gripper, cannot place an object that is not held, plan
                  must not end while holding something
Nothing is executed unless every check passes.
"""
import re
from dataclasses import dataclass, field
from typing import List

from .scene_model import WorldState
from .skill_spec import MAX_PLAN_STEPS, SKILLS

_LOW_LEVEL_KEYS = re.compile(r'joint|trajector|position|velocit|torque|effort|angle', re.I)


@dataclass
class ValidationResult:
    ok: bool
    plan: List[dict] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def _norm_symbol(value):
    return re.sub(r'[\s\-]+', '_', value.strip().lower())


class TaskValidator:
    def __init__(self, scene, append_home=True):
        self.scene = scene
        self.append_home = append_home

    def validate(self, raw, world: WorldState) -> ValidationResult:
        errors, warnings = [], []

        # ---------------------------------------------------------- 1 syntax
        if not isinstance(raw, dict):
            return ValidationResult(False, errors=['plan must be a JSON object {"plan": [...]}'])
        if 'plan' not in raw or not isinstance(raw['plan'], list):
            return ValidationResult(False, errors=['missing "plan" list'])
        steps = raw['plan']
        if not steps:
            reason = raw.get('error') or 'empty plan'
            return ValidationResult(False, errors=[f'LLM returned no plan: {reason}'])
        if len(steps) > MAX_PLAN_STEPS:
            return ValidationResult(
                False, errors=[f'plan too long ({len(steps)} > {MAX_PLAN_STEPS})'])
        if raw.get('error'):
            errors.append(f'LLM reported an error together with a plan: {raw["error"]}')

        plan = []
        for i, step in enumerate(steps, 1):
            tag = f'step {i}'
            if not isinstance(step, dict):
                errors.append(f'{tag}: must be an object, got {type(step).__name__}')
                continue
            skill = step.get('skill')
            if not isinstance(skill, str):
                errors.append(f'{tag}: missing "skill"')
                continue
            skill = _norm_symbol(skill)
            # ------------------------------------------------ 2 whitelist
            if skill not in SKILLS:
                errors.append(f'{tag}: skill "{skill}" is not allowed '
                              f'(allowed: {", ".join(SKILLS)})')
                continue
            required = SKILLS[skill]['params']
            extra = [k for k in step if k not in ('skill', *required)]
            low_level = [k for k in extra if _LOW_LEVEL_KEYS.search(k)]
            if low_level:
                errors.append(f'{tag}: low-level robot control is forbidden '
                              f'(keys {low_level}); the LLM may only select skills')
                continue
            if extra:
                errors.append(f'{tag}: unexpected parameter(s) {extra} for {skill}')
                continue
            clean = {'skill': skill}
            for p in required:
                v = step.get(p)
                if not isinstance(v, str) or not v.strip():
                    errors.append(f'{tag}: {skill} needs string parameter "{p}"')
                    break
                clean[p] = _norm_symbol(v)
            else:
                # -------------------------------------------- 3 symbols
                if 'object' in clean and clean['object'] not in self.scene.objects:
                    errors.append(f'{tag}: INVALID_OBJECT "{clean["object"]}" '
                                  f'(valid: {", ".join(self.scene.object_names)})')
                    continue
                if 'zone' in clean and clean['zone'] not in self.scene.zones:
                    errors.append(f'{tag}: INVALID_ZONE "{clean["zone"]}" '
                                  f'(valid: {", ".join(self.scene.zone_names)})')
                    continue
                plan.append(clean)

        if errors:
            return ValidationResult(False, plan=plan, errors=errors)

        # ----------------------------------------------------- 4 semantics
        sim = world.copy()
        for i, step in enumerate(plan, 1):
            s = step['skill']
            if s == 'pick':
                if sim.held is not None:
                    errors.append(f'step {i}: pick({step["object"]}) while already '
                                  f'holding {sim.held}')
                else:
                    sim.held = step['object']
            elif s == 'place':
                if sim.held != step['object']:
                    errors.append(f'step {i}: place({step["object"]}, {step["zone"]}) but '
                                  f'gripper holds {sim.held or "nothing"}')
                else:
                    sim.positions[step['object']] = self.scene.slot_xy(step['zone'])
                    sim.held = None
            elif s == 'move_above' and step['object'] == sim.held:
                errors.append(f'step {i}: move_above({step["object"]}) but it is in the gripper')
        if sim.held is not None:
            errors.append(f'plan ends while still holding {sim.held}')
        if errors:
            return ValidationResult(False, plan=plan, errors=errors)

        if plan[-1]['skill'] != 'home' and self.append_home:
            plan.append({'skill': 'home'})
            warnings.append('plan did not end with home(); home() appended')
        return ValidationResult(True, plan=plan, warnings=warnings)
