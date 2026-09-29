"""Generate the Gazebo Classic world (SDF 1.6) from scene.yaml.

    python3 -m ur3_llm_control.world_gen config/scene.yaml > /tmp/ur3_llm.world

Table and zone pads are static; cubes are kinematic links (no gravity drift),
moved only by our virtual gripper through /gazebo/set_entity_state.
"""
import sys

from .scene_model import SceneConfig


def _rgba(v):
    return ' '.join(f'{float(c):.3f}' for c in v)


def _box_model(name, xyz, size, rgba, static=True, collision=True, kinematic=False):
    sx, sy, sz = size
    geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
    coll = f'<collision name="collision">{geom}</collision>' if collision else ''
    inertial = ''
    if not static:
        m = 0.05
        i = m * (sx * sx + sz * sz) / 12.0
        inertial = (f'<inertial><mass>{m}</mass><inertia><ixx>{i:.8f}</ixx><iyy>{i:.8f}</iyy>'
                    f'<izz>{i:.8f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    return f"""
    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose>
      <link name="link">
        {'<kinematic>true</kinematic><gravity>false</gravity>' if kinematic else ''}
        {inertial}
        {coll}
        <visual name="visual">{geom}
          <material><ambient>{_rgba(rgba)}</ambient><diffuse>{_rgba(rgba)}</diffuse></material>
        </visual>
      </link>
    </model>"""


def generate(scene: SceneConfig) -> str:
    tx, ty = scene.table['center']
    sx, sy = scene.table['size']
    h = scene.table_top
    models = [_box_model('work_table', (tx, ty, h / 2), (sx, sy, h), (0.55, 0.40, 0.25, 1))]
    for name, z in scene.zones.items():
        x, y = z['xy']
        s = z['size']
        models.append(_box_model(name, (x, y, h + 0.001), (s, s, 0.002), z['rgba'],
                                 collision=False))
    for name, b in scene.buffers.items():
        x, y = b['xy']
        models.append(_box_model(name, (x, y, h + 0.001), (0.06, 0.06, 0.002),
                                 (0.7, 0.7, 0.7, 0.5), collision=False))
    c = scene.cube_size
    for name, o in scene.objects.items():
        x, y = o['xy']
        models.append(_box_model(name, (x, y, scene.cube_center_z()), (c, c, c), o['rgba'],
                                 static=False, kinematic=True))
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="ur3_llm_world">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <gravity>0 0 -9.81</gravity>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
    <gui><camera name="user_camera"><pose>1.2 -0.9 0.9 0 0.45 2.45</pose></camera></gui>
    {''.join(models)}
  </world>
</sdf>
"""


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    print(generate(SceneConfig.from_file(argv[0])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
