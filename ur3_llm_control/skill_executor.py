"""Skill executor.

1. expand_plan(): deterministic conflict resolution on a validated plan
   – object already in its target zone  -> pick/place are SKIPPED
   – target zone occupied by another cube -> that cube is first moved to a
     free buffer slot (auto-inserted steps, shown with '[auto]')
2. execute(): runs the steps one by one through RobotSkills, prints the
   status table, stops at the first failure.
"""
from .skill_spec import Status, step_to_str

LINE_WIDTH = 30


class PlanExpansionError(RuntimeError):
    pass


def _find_place(plan, start, obj):
    for j in range(start, len(plan)):
        s = plan[j]
        if s['skill'] == 'place' and s['object'] == obj:
            return j
        if s['skill'] == 'pick':
            break
    return None


def reorder_pairs(plan, world):
    """If the plan is only pick/place pairs (+ trailing home), order the pairs
    so that a pair whose target zone is free runs first.  The final result is
    the same, but fewer cubes have to be parked in a buffer slot.
    Only a real cycle (e.g. swapping two cubes) still needs a buffer."""
    body = list(plan)
    tail = []
    while body and body[-1]['skill'] == 'home':
        tail.insert(0, body.pop())
    if len(body) % 2 or not body:
        return list(plan)
    pairs = []
    for a, b in zip(body[0::2], body[1::2]):
        if not (a['skill'] == 'pick' and b['skill'] == 'place' and a['object'] == b['object']):
            return list(plan)
        pairs.append((a, b))
    sim = world.copy()
    ordered = []
    while pairs:
        pick = next((p for p in pairs
                     if sim.occupant(p[1]['zone'], exclude=p[0]['object']) is None), pairs[0])
        pairs.remove(pick)
        ordered += list(pick)
        sim.positions[pick[0]['object']] = sim.scene.slot_xy(pick[1]['zone'])
    return ordered + tail


def expand_plan(plan, world):
    """Return a list of steps (dicts, may carry 'auto' or 'skip' flags)."""
    plan = reorder_pairs(plan, world)
    sim = world.copy()
    out = []
    skip_idx = set()
    for i, step in enumerate(plan):
        if i in skip_idx:
            out.append(dict(step, skip=True))
            continue
        s = step['skill']
        if s == 'pick':
            obj = step['object']
            j = _find_place(plan, i + 1, obj)
            if j is not None:
                zone = plan[j]['zone']
                if sim.slot_of(obj) == zone:
                    out.append(dict(step, skip=True))
                    skip_idx.add(j)
                    continue
                occ = sim.occupant(zone, exclude=obj)
                if occ is not None:
                    buf = sim.free_buffer()
                    if buf is None:
                        raise PlanExpansionError(
                            f'{zone} is occupied by {occ} and no buffer slot is free')
                    out.append({'skill': 'pick', 'object': occ, 'auto': True})
                    out.append({'skill': 'place', 'object': occ, 'zone': buf, 'auto': True})
                    sim.positions[occ] = sim.scene.slot_xy(buf)
            sim.held = obj
        elif s == 'place':
            sim.positions[step['object']] = sim.scene.slot_xy(step['zone'])
            sim.held = None
        out.append(dict(step))
    return out


def format_line(label, status):
    dots = '.' * max(2, LINE_WIDTH - len(label))
    return f'{label} {dots} {status}'


class SkillExecutor:
    def __init__(self, skills, log=print):
        self.skills = skills
        self.log = log

    def _run(self, step):
        s = step['skill']
        auto = bool(step.get('auto'))
        if s == 'home':
            return self.skills.home()
        if s == 'pick':
            return self.skills.pick(step['object'])
        if s == 'place':
            return self.skills.place(step['object'], step['zone'], allow_buffer=auto)
        if s == 'move_above':
            return self.skills.move_above(step['object'])
        if s == 'move_to_zone':
            return self.skills.move_to_zone(step['zone'])
        return Status.FAILED

    def execute(self, steps):
        """Returns (task_ok, [(label, status), ...])."""
        results = []
        failed = False
        for step in steps:
            label = step_to_str(step) + ('  [auto]' if step.get('auto') else '')
            if failed:
                status = 'NOT EXECUTED'
            elif step.get('skip'):
                status = Status.SKIPPED
            else:
                try:
                    status = self._run(step)
                except Exception as e:           # never leave the loop silently
                    self.log(f'[executor] exception in {label}: {e!r}')
                    status = Status.FAILED
                if status not in Status.OK:
                    failed = True
            results.append((label, status))
            self.log(format_line(label, status))
        return (not failed), results
