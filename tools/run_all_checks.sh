#!/usr/bin/env bash
# Offline checks (no Gazebo / MoveIt needed). Run from anywhere:
#   ./src/ur3_llm_control/tools/run_all_checks.sh
# Exit code 0 = everything passed.
set -u
PKG="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PKG"
export PYTHONPATH="$PKG:${PYTHONPATH:-}"
fail=0
step() { echo; echo "=== $1"; }
ok()   { echo "    -> OK"; }
bad()  { echo "    -> FAILED"; fail=1; }

step "1/5 python syntax (package + launch files)"
python3 -m py_compile ur3_llm_control/*.py launch/*.py && ok || bad

step "2/5 unit + integration tests (pytest, no ROS)"
python3 -m pytest test -q && ok || bad

step "3/5 scene reachability (IK, tool down, joint limits) for UR3 and UR3e"
python3 -m ur3_llm_control.check_scene config/scene.yaml && ok || bad

step "4/5 Gazebo world generated from scene.yaml is valid XML"
python3 - <<'EOF' && ok || bad
import xml.dom.minidom
from ur3_llm_control.scene_model import SceneConfig
from ur3_llm_control.world_gen import generate
xml.dom.minidom.parseString(generate(SceneConfig.from_file('config/scene.yaml')))
EOF

step "5/5 student config + LLM settings"
python3 - <<'EOF' && ok || bad
import os, yaml
from ur3_llm_control.student_task import describe
c = yaml.safe_load(open('config/student_config.yaml', encoding='utf-8'))
print(describe(c['student_name'], c['student_id']))
llm = c.get('llm', {})
print('model   :', os.environ.get('NINEROUTER_MODEL') or llm.get('model'))
print('base_url:', os.environ.get('NINEROUTER_BASE_URL') or llm.get('base_url'))
key = os.environ.get('NINEROUTER_API_KEY') or llm.get('api_key')
print('api key :', 'set' if key else 'NOT SET (export NINEROUTER_API_KEY=...)')
EOF

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix ur3_llm_control >/dev/null 2>&1; then
  step "extra: ROS 2 environment"
  for p in ur_description ur_moveit_config ur_simulation_gazebo gazebo_ros gazebo_ros2_control; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "    $p: found"; else echo "    $p: MISSING"; fail=1; fi
  done
fi

echo
if [ $fail -eq 0 ]; then echo "ALL CHECKS PASSED"; else echo "SOME CHECKS FAILED"; fi
exit $fail
