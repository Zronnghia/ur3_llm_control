"""Check ur_kinematics.py against the REAL UR URDF.

`ur_kinematics.py` is only used for the offline reachability check
(`check_scene`) and for the unit tests, but if its numbers drifted from the
actual robot description, `check_scene` would happily approve a scene the
real robot cannot reach.  This script rebuilds the URDF with xacro from
ur_description and compares, for both ur3 and ur3e:

  * forward kinematics world -> tool0 over random joint configurations
  * the joint limits
  * that the links the package relies on exist (world, tool0, flange)
  * the Gazebo / ros2_control plugins the launch file assumes

Usage
  on a machine with ROS 2 Humble:   python3 verify_urdf_fk.py --from-ros
  anywhere (needs network, once):   ./fetch_deps.sh && python3 verify_urdf_fk.py
"""
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEPS = os.path.join(HERE, '_deps')
sys.path.insert(0, os.path.join(HERE, '..', '..'))

from ur3_llm_control.ur_kinematics import (JOINT_LOWER, JOINT_NAMES,  # noqa: E402
                                           JOINT_UPPER, URKinematics)

FROM_ROS = '--from-ros' in sys.argv


# --------------------------------------------------------------- URDF maths
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def mat(xyz, rot):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = xyz
    return m


def rotz(q):
    m = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    m[:2, :2] = [[c, -s], [s, c]]
    return m


class Urdf:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.joints = {}
        for j in self.root.findall('joint'):
            o = j.find('origin')
            lim = j.find('limit')
            self.joints[j.get('name')] = dict(
                type=j.get('type'),
                parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                xyz=[float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()],
                rpy=[float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()],
                axis=[float(v) for v in j.find('axis').get('xyz').split()]
                if j.find('axis') is not None else None,
                lower=float(lim.get('lower')) if lim is not None and lim.get('lower') else None,
                upper=float(lim.get('upper')) if lim is not None and lim.get('upper') else None)
        self.child_of = {v['child']: k for k, v in self.joints.items()}

    def chain(self, tip, base='world'):
        out, link = [], tip
        while link != base:
            jn = self.child_of[link]
            out.append(jn)
            link = self.joints[jn]['parent']
        return list(reversed(out))

    def fk(self, tip, q):
        m, qi = np.eye(4), dict(zip(JOINT_NAMES, q))
        for jn in self.chain(tip):
            j = self.joints[jn]
            m = m @ mat(j['xyz'], rpy(*j['rpy']))
            if j['type'] in ('revolute', 'continuous'):
                assert j['axis'] == [0, 0, 1], (jn, j['axis'])
                m = m @ rotz(qi[jn])
        return m

    @property
    def links(self):
        return {ln.get('name') for ln in self.root.findall('link')}


# ----------------------------------------------------------------- building
def build_urdf(ur_type):
    """Run the real xacro on ur_description/urdf/ur.urdf.xacro."""
    out = os.path.join(tempfile.gettempdir(), f'ur3_llm_check_{ur_type}.urdf')
    args = ['safety_limits:=true', 'safety_pos_margin:=0.15', 'safety_k_position:=20',
            'name:=ur', f'ur_type:={ur_type}', 'prefix:=', 'sim_gazebo:=true']
    if FROM_ROS:
        from ament_index_python.packages import get_package_share_directory
        src = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.urdf.xacro')
        with open(out, 'w') as f:
            subprocess.run(['xacro', src] + args, check=True, stdout=f)
        return out
    env = dict(os.environ)
    shim = os.path.join(tempfile.gettempdir(), 'ur3_llm_xacro_shim')
    os.makedirs(os.path.join(shim, 'ament_index_python'), exist_ok=True)
    with open(os.path.join(shim, 'ament_index_python', '__init__.py'), 'w') as f:
        f.write('from .packages import get_package_share_directory\n')
    with open(os.path.join(shim, 'ament_index_python', 'packages.py'), 'w') as f:
        f.write('MAP = %r\n' % {
            'ur_description': os.path.join(DEPS, 'ur_description'),
            'ur_simulation_gazebo': os.path.join(DEPS, 'ur_simulation_gazebo',
                                                 'ur_simulation_gazebo')})
        f.write('class PackageNotFoundError(KeyError):\n    pass\n'
                'def get_package_share_directory(n):\n'
                '    if n not in MAP:\n        raise PackageNotFoundError(n)\n'
                '    return MAP[n]\n'
                'def get_package_prefix(n):\n    return get_package_share_directory(n)\n')
    env['PYTHONPATH'] = os.pathsep.join([shim, os.path.join(DEPS, 'xacro'),
                                         env.get('PYTHONPATH', '')])
    src = os.path.join(DEPS, 'ur_description', 'urdf', 'ur.urdf.xacro')
    if not os.path.exists(src):
        sys.exit('run ./fetch_deps.sh first (or pass --from-ros on a ROS machine)')
    code = ('import sys; sys.argv=%r\nimport xacro; xacro.main()'
            % (['xacro', src] + args))
    with open(out, 'w') as f:
        subprocess.run([sys.executable, '-c', code], check=True, stdout=f, env=env)
    return out


# -------------------------------------------------------------------- check
def check(ur_type):
    path = build_urdf(ur_type)
    u, mine = Urdf(path), URKinematics(ur_type)
    problems = []
    print(f'[{ur_type}] chain: {" -> ".join(u.chain("tool0"))}')

    rng = np.random.default_rng(12345)
    worst = max(np.abs(u.fk('tool0', q) - mine.fk(q)).max()
                for q in rng.uniform(-3.0, 3.0, (500, 6)))
    print(f'[{ur_type}] max FK deviation over 500 configs: {worst:.2e}')
    if worst > 1e-9:
        problems.append(f'FK differs from the URDF by {worst:.2e}')

    # Only a model that is LOOSER than the URDF is a defect: check_scene would
    # then approve a pose the real robot cannot reach.  A tighter model just
    # makes the offline check conservative, which is safe.
    for i, n in enumerate(JOINT_NAMES):
        j = u.joints[n]
        if j['lower'] is None:                    # wrist_3 is `continuous` = unlimited
            print(f'[{ur_type}] note: {n} is continuous in the URDF, the model uses '
                  f'[{JOINT_LOWER[i]:+.4f},{JOINT_UPPER[i]:+.4f}] (conservative)')
            continue
        if JOINT_LOWER[i] < j['lower'] - 1e-6 or JOINT_UPPER[i] > j['upper'] + 1e-6:
            problems.append(f'{n}: model [{JOINT_LOWER[i]:.4f},{JOINT_UPPER[i]:.4f}] is '
                            f'LOOSER than the URDF [{j["lower"]:.4f},{j["upper"]:.4f}]')
        elif abs(j['lower'] - JOINT_LOWER[i]) > 1e-6 or abs(j['upper'] - JOINT_UPPER[i]) > 1e-6:
            print(f'[{ur_type}] note: {n} model is tighter than the URDF (conservative)')

    for link in ('world', 'tool0', 'wrist_3_link', 'flange'):
        if link not in u.links:
            problems.append(f'link {link!r} missing from the URDF')

    gz = [p.get('filename') for g in u.root.findall('gazebo') for p in g.iter('plugin')]
    rc = [p.text for r in u.root.findall('ros2_control') for p in r.iter('plugin')]
    print(f'[{ur_type}] gazebo plugin: {gz} | ros2_control: {rc}')
    if 'libgazebo_ros2_control.so' not in gz:
        problems.append('libgazebo_ros2_control.so not in the URDF')
    if 'gazebo_ros2_control/GazeboSystem' not in rc:
        problems.append('GazeboSystem hardware plugin not in the URDF')

    # ground_plane only exists in the GAZEBO description (sim_gazebo:=true).
    # MoveIt's description (ur_moveit.launch.py) has no such link, which is why
    # moveit_backend.setup_scene() adds its own floor box.
    gpj = u.joints.get('ground_plane_joint')
    if gpj:
        print(f'[{ur_type}] Gazebo-only ground_plane link at z={gpj["xyz"][2]} '
              '(not in MoveIt model -> planning scene adds a floor)')
    for p in problems:
        print(f'[{ur_type}] PROBLEM: {p}')
    print(f'[{ur_type}] ' + ('OK' if not problems else f'{len(problems)} problem(s)'))
    return problems


if __name__ == '__main__':
    bad = sum(len(check(t)) for t in ('ur3e', 'ur3'))
    print('URDF CHECK:', 'ALL OK' if not bad else f'{bad} PROBLEM(S)')
    sys.exit(1 if bad else 0)
