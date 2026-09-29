"""User command -> LLM -> validator -> executor, with the terminal report
required by the assignment.  Independent of ROS (the ROS node just feeds it
commands and a MoveIt backend)."""
import json

from .skill_executor import PlanExpansionError, SkillExecutor, expand_plan
from .skill_spec import step_to_str
from .llm_planner import LLMError

BAR = '=' * 60


class CommandPipeline:
    def __init__(self, planner, skills, world, log=print):
        self.planner = planner
        self.skills = skills
        self.world = world
        self.log = log
        self.executor = SkillExecutor(skills, log=log)

    def run(self, command, dry_run=False):
        """Returns a dict report (also published on /llm_robot/result)."""
        log = self.log
        report = {'command': command, 'plan': [], 'status': 'TASK FAILED', 'steps': []}
        log(BAR)
        log('USER COMMAND:')
        log(command)
        log('')
        try:
            result, answer = self.planner.plan(command, self.world)
        except LLMError as e:
            log(f'LLM ERROR: {e}')
            log('')
            log('TASK REJECTED')
            report['status'] = 'TASK REJECTED'
            report['error'] = str(e)
            log(BAR)
            return report

        if not result.ok:
            log('LLM PLAN: REJECTED BY VALIDATOR')
            for err in result.errors:
                log(f'  - {err}')
            log(f'  raw LLM answer: {answer.strip()[:400]}')
            log('')
            log('TASK REJECTED (nothing was executed)')
            report['status'] = 'TASK REJECTED'
            report['errors'] = result.errors
            log(BAR)
            return report

        report['plan'] = result.plan
        log('LLM PLAN:')
        for step in result.plan:
            log(step_to_str(step))
        for w in result.warnings:
            log(f'  (validator note: {w})')
        log('')
        log('JSON PLAN:')
        log(json.dumps({'plan': result.plan}, ensure_ascii=False))
        log('')

        try:
            steps = expand_plan(result.plan, self.world)
        except PlanExpansionError as e:
            log(f'EXECUTION ABORTED: {e}')
            log('')
            log('TASK FAILED')
            log(BAR)
            return report
        plain = [{k: v for k, v in s.items() if k not in ('auto', 'skip')} for s in steps]
        if plain != result.plan or any(s.get('skip') for s in steps):
            log('EXECUTION PLAN (reordered / zone conflicts resolved):')
            for s in steps:
                tag = '  [auto: clear target zone]' if s.get('auto') else (
                    '  [skip: already in place]' if s.get('skip') else '')
                log(step_to_str(s) + tag)
            log('')

        if dry_run:
            log('DRY RUN: plan validated, robot not moved')
            report['status'] = 'DRY RUN'
            log(BAR)
            return report

        log('EXECUTION:')
        ok, results = self.executor.execute(steps)
        report['steps'] = results
        log('')
        report['status'] = 'TASK SUCCESS' if ok else 'TASK FAILED'
        log(report['status'])
        log('World state: ' + ', '.join(f'{o}={w}' for o, w in self.world.summary().items()))
        log(BAR)
        return report
