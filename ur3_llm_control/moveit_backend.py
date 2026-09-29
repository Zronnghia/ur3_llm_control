"""MoveIt 2 motion backend (ROS 2 Humble, rclpy).

Talks to move_group through its standard ROS interfaces (moveit_py is not
released for Humble):
  /move_action              moveit_msgs/action/MoveGroup      plan + execute (OMPL)
  /compute_cartesian_path   moveit_msgs/srv/GetCartesianPath  straight approach/retreat
  /execute_trajectory       moveit_msgs/action/ExecuteTrajectory
  /apply_planning_scene     moveit_msgs/srv/ApplyPlanningScene table, cubes, attach/detach
Gazebo (optional):
  /gazebo/set_entity_state  gazebo_msgs/srv/SetEntityState    virtual gripper: the held cube
                                                               follows tool0

MoveIt enforces joint limits (URDF/joint_limits.yaml) and checks self-collision
and collision with the planning scene (floor, table, cubes, held cube) for
every motion, including the Cartesian segments (avoid_collisions=True).
"""
import math
import threading
import time

from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import tf2_ros

from .robot_skills import MotionBackend
from .skill_spec import Status
from .ur_kinematics import JOINT_NAMES, URKinematics

try:                                   # Gazebo is optional (MoveIt-only demo works too)
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
except ImportError:                    # pragma: no cover
    SetEntityState = None

_EXEC_ERRORS = {MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,
                MoveItErrorCodes.CONTROL_FAILED, MoveItErrorCodes.TIMED_OUT,
                MoveItErrorCodes.PREEMPTED}

# tool0 z-axis pointing straight down = rotation of pi about world x
TOOL_DOWN = (1.0, 0.0, 0.0, 0.0)       # (x, y, z, w)

# floor collision box (see setup_scene)
FLOOR_SIZE = 2.0
FLOOR_THICKNESS = 0.01
FLOOR_GAP = 0.005


# ------------------------------------------------------------------ helpers
def _quat(x, y, z, w):
    q = Quaternion()
    q.x, q.y, q.z, q.w = float(x), float(y), float(z), float(w)
    return q


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _yaw_only(q):
    x, y, z, w = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _pose(xyz, q=(0.0, 0.0, 0.0, 1.0)):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation = _quat(*q)
    return p


def _box(size):
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(s) for s in size]
    return prim


def _set_if(msg, name, value):
    if hasattr(msg, name):
        setattr(msg, name, value)


class MoveItBackend(MotionBackend):
    def __init__(self, node, scene, callback_group=None, use_gazebo=True):
        self.node = node
        self.scene = scene
        self.m = scene.motion
        self.frame = scene.frame_id
        self.ee = scene.ee_link
        self.group = scene.planning_group
        self.log = node.get_logger()

        cg = callback_group
        self.move_client = ActionClient(node, MoveGroup, '/move_action', callback_group=cg)
        self.exec_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory',
                                        callback_group=cg)
        self.cart_client = node.create_client(GetCartesianPath, '/compute_cartesian_path',
                                              callback_group=cg)
        self.scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                               callback_group=cg)
        self.gz_client = None
        if use_gazebo and SetEntityState is not None:
            self.gz_client = node.create_client(SetEntityState, '/gazebo/set_entity_state',
                                                callback_group=cg)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self._attached = None
        self._lock = threading.Lock()
        # Two DIFFERENT offsets for two different consumers -- using one
        # value for both (as an earlier version of this file did) caused a
        # real regression: bumping it to give the Gazebo-visual cube
        # clearance from wrist_3_link/flange also pushed the MoveIt-side
        # ATTACHED COLLISION OBJECT that far below tool0, which put it
        # *below the table surface* right after grasping (tool0 is only
        # ~grasp_gap above the table there) -- MoveIt then refused to plan
        # the retreat lift because its own model of the held cube already
        # read as embedded in the table.
        #
        # _grip_offset: how far MoveIt's ATTACHED COLLISION OBJECT sits
        # below tool0 (attach(), below). Keep this accurate/small so the
        # planning model doesn't imagine the held cube somewhere it isn't.
        self._grip_offset = self.m['grasp_gap'] + scene.cube_size / 2.0
        # _visual_follow_offset: how far the Gazebo-visual cube (the one
        # SetEntityState teleports to trail tool0 -- see _follow_attached
        # and _cube_pose_from_tool) sits below tool0. This one CAN be
        # larger than _grip_offset: it only affects what Gazebo renders and
        # its own (independent) physics, not MoveIt's planning, so it's
        # free to add clearance from wrist_3_link/flange's collision
        # geometry without confusing the planner.
        self._visual_follow_offset = 0.05
        # cube orientation relative to tool0 (tool z down -> cube z up)
        self._q_tool_to_cube = (1.0, 0.0, 0.0, 0.0)
        # The held cube isn't a real physical attachment: this timer
        # teleports it (SetEntityState) to wherever tool0 currently is, at a
        # fixed rate. Each tick is a discontinuous jump, not a continuous
        # physics motion, and the cube is still a full-size rigid body with
        # real collision in Gazebo. When the descent brings it close to the
        # table (the final few mm before place/release), a large per-tick
        # jump can land it slightly overlapping the table surface; the ODE
        # solver then fights the next forced teleport with a contact-
        # correction impulse, which is what shows up as the cube visibly
        # stretching/glitching for a frame. A higher follow rate shrinks the
        # per-tick jump and keeps it well clear of that overlap.
        node.create_timer(0.02, self._follow_attached, callback_group=cg)

        # Numerical IK (ur_kinematics.URKinematics), seeded from the robot's
        # OWN current joint state, so move_above/move_to_zone command the
        # solution branch closest to wherever the arm already is -- see the
        # comment in move_pose_down for why this matters.
        self._kin = URKinematics(scene.ur_type)
        self._joint_state = {}
        self._js_lock = threading.Lock()
        node.create_subscription(JointState, '/joint_states', self._on_joint_state,
                                 10, callback_group=cg)

    # ---------------------------------------------------------------- infra
    @staticmethod
    def _wait(future, timeout):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                return None
            time.sleep(0.01)
        return future.result()

    def wait_for_servers(self, timeout=120.0):
        end = time.monotonic() + timeout
        pending = {'/move_action': self.move_client.server_is_ready,
                   '/execute_trajectory': self.exec_client.server_is_ready,
                   '/compute_cartesian_path': self.cart_client.service_is_ready,
                   '/apply_planning_scene': self.scene_client.service_is_ready}
        while time.monotonic() < end:
            missing = [n for n, ready in pending.items() if not ready()]
            if not missing:
                break
            time.sleep(0.5)
        else:
            return missing
        # TF world -> tool0 must be available as well
        while time.monotonic() < end:
            if self.tool_pose() is not None:
                return []
            time.sleep(0.5)
        return ['TF %s -> %s' % (self.frame, self.ee)]

    def _on_joint_state(self, msg):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_state[name] = pos

    def _current_joints(self, timeout=2.0):
        """Return [q0..q5] in JOINT_NAMES order, or None if /joint_states
        hasn't delivered all six yet (e.g. called too soon after launch)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._js_lock:
                if all(n in self._joint_state for n in JOINT_NAMES):
                    return [self._joint_state[n] for n in JOINT_NAMES]
            time.sleep(0.02)
        return None

    def tool_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.ee, Time(),
                                                timeout=Duration(seconds=0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr, r = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w)

    def _apply(self, scene_msg):
        req = ApplyPlanningScene.Request()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        req.scene = scene_msg
        res = self._wait(self.scene_client.call_async(req), 10.0)
        return res is not None and res.success

    # ------------------------------------------------------- planning scene
    def _collision_object(self, oid, size, xyz, q=(0, 0, 0, 1), op=CollisionObject.ADD,
                          frame=None):
        co = CollisionObject()
        co.header.frame_id = frame or self.frame
        co.id = oid
        co.pose = _pose(xyz, q)
        co.primitives = [_box(size)]
        co.primitive_poses = [_pose((0, 0, 0))]
        co.operation = op
        return co

    def setup_scene(self, world):
        """Floor, table and cubes as collision objects (zones are flat).

        The floor IS needed: move_group gets its robot_description from
        ur_moveit.launch.py, which runs xacro WITHOUT sim_gazebo:=true, so the
        `ground_plane` link that exists in the Gazebo model does not exist in
        MoveIt's model.  Without this box MoveIt would happily plan through the
        ground.  Its top face is `floor_gap` below z=0, so it never touches the
        robot base (lowest point of the UR3/UR3e base collision mesh: -0.0007 m).
        """
        ps = PlanningScene()
        tx, ty = self.scene.table['center']
        sx, sy = self.scene.table['size']
        h = self.scene.table_top
        gap = float(self.m.get('floor_gap', FLOOR_GAP))
        objs = [
            self._collision_object('floor', (FLOOR_SIZE, FLOOR_SIZE, FLOOR_THICKNESS),
                                   (0.0, 0.0, -gap - FLOOR_THICKNESS / 2.0)),
            self._collision_object('table', (sx, sy, h), (tx, ty, h / 2.0)),
        ]
        s = self.scene.cube_size
        for name, (x, y) in world.positions.items():
            objs.append(self._collision_object(name, (s, s, s),
                                               (x, y, self.scene.cube_center_z())))
        # 1) drop anything left attached to tool0 by a previous run (errors for
        #    objects that are not attached are harmless -> result ignored)
        for name in world.positions:
            aco = AttachedCollisionObject()
            aco.link_name = self.ee
            aco.object.id = name
            aco.object.operation = CollisionObject.REMOVE
            ps.robot_state.attached_collision_objects.append(aco)
        self._apply(ps)
        # 2) (re)add floor, table and cubes at their true poses (ADD on an
        #    existing id replaces it, so restarting the node is safe)
        ps2 = PlanningScene()
        ps2.world.collision_objects = objs
        return self._apply(ps2)

    def reset_gazebo(self, world):
        if not self.gz_client or not self.gz_client.wait_for_service(timeout_sec=2.0):
            self.log.warn('/gazebo/set_entity_state not available: Gazebo cubes not synced')
            return False
        for name, (x, y) in world.positions.items():
            self._gz_set(name, (x, y, self.scene.cube_center_z()), (0, 0, 0, 1), wait=True)
        return True

    def _gz_set(self, name, xyz, q, wait=False):
        if not self.gz_client or not self.gz_client.service_is_ready():
            return
        req = SetEntityState.Request()
        st = EntityState()
        st.name = name
        st.pose = _pose(xyz, q)
        st.reference_frame = 'world'
        req.state = st
        fut = self.gz_client.call_async(req)
        if wait:
            self._wait(fut, 2.0)

    def _cube_pose_from_tool(self, tool):
        (px, py, pz), q = tool
        # tool z axis in world = third column of R(q)
        x, y, z, w = q
        zx = 2 * (x * z + w * y)
        zy = 2 * (y * z - w * x)
        zz = 1 - 2 * (x * x + y * y)
        d = self._visual_follow_offset
        return (px + zx * d, py + zy * d, pz + zz * d), _qmul(q, self._q_tool_to_cube)

    def _follow_attached(self):
        with self._lock:
            obj = self._attached
        if obj is None:
            return
        tool = self.tool_pose()
        if tool is None:
            return
        xyz, q = self._cube_pose_from_tool(tool)
        self._gz_set(obj, xyz, q)

    # --------------------------------------------------------------- motion
    def _base_request(self):
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = int(self.m['planning_attempts'])
        req.allowed_planning_time = float(self.m['planning_time'])
        req.max_velocity_scaling_factor = float(self.m['velocity_scaling'])
        req.max_acceleration_scaling_factor = float(self.m['acceleration_scaling'])
        req.start_state.is_diff = True
        ws = req.workspace_parameters
        ws.header.frame_id = self.frame
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.0, -1.0, -0.05
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.0, 1.0, 1.2
        return req

    def _send_move_group(self, constraints):
        if not self.move_client.server_is_ready():
            return Status.PLANNING_FAILED
        goal = MoveGroup.Goal()
        goal.request = self._base_request()
        goal.request.goal_constraints = [constraints]
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self._wait(self.move_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return Status.PLANNING_FAILED
        res = self._wait(handle.get_result_async(), 120.0)
        if res is None:
            return Status.EXECUTION_FAILED
        return self._map(res.result.error_code.val)

    @staticmethod
    def _map(code):
        if code == MoveItErrorCodes.SUCCESS:
            return Status.SUCCESS
        if code in _EXEC_ERRORS:
            return Status.EXECUTION_FAILED
        return Status.PLANNING_FAILED

    def move_joints(self, joints):
        c = Constraints()
        for name, val in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        status = self._send_move_group(c)
        if status == Status.SUCCESS:
            # move_joints is always immediately followed by a Cartesian move
            # (move_vertical) whose start-state check requires the robot to
            # be within 0.01 rad of where MoveIt thinks it stopped
            # (trajectory_execution.allowed_start_tolerance). The controller
            # reporting "goal reached" doesn't guarantee every state channel
            # (TF, /joint_states, the planning-scene monitor's cached robot
            # state) has caught up yet -- small residual settling/jitter
            # (a couple of degrees) is enough to trip that tight check if the
            # next request goes out immediately. A short pause here lets
            # everything settle before we plan the next segment.
            time.sleep(0.3)
        return status

    def move_pose_down(self, xyz):
        # Prefer a DETERMINISTIC route: solve IK numerically, seeded from the
        # robot's own current joint state (ur_kinematics.ik_down does damped
        # least-squares from that seed, so it converges to the branch nearest
        # to where the arm already is), then send it as an exact joint-space
        # goal via move_joints() (tight 0.01 rad tolerance).
        #
        # The previous approach asked OMPL to plan into a whole REGION of
        # valid poses (a position sphere + an orientation cone). That region
        # is still a continuous, under-constrained target, so OMPL's sampling
        # was free to land on a different (but equally valid) IK branch than
        # the one the robot was already in -- most often a wrist joint
        # solution a full extra revolution away. Physically identical
        # orientation, but the Gazebo joint_trajectory_controller has no
        # notion of 2*pi-equivalence and aborts with a ~2*pi state-tolerance
        # error on whichever wrist joint absorbed the extra turn. Narrowing
        # the orientation region (see git history) only made this rarer, not
        # impossible, because it was still a region, not a single target.
        # Solving IK ourselves and commanding an exact joint goal removes the
        # region-search step (and its branch ambiguity) entirely.
        seed = self._current_joints()
        if seed is not None:
            sol = self._kin.ik_down(xyz, seeds=[seed])
            if sol is not None:
                return self.move_joints(sol)
            self.log.warn(f'IK seeded from the current joint state did not '
                          f'converge for target {xyz}; falling back to OMPL '
                          f'pose-region planning for this move')
        else:
            self.log.warn('no /joint_states yet; falling back to OMPL '
                          'pose-region planning for this move')
        return self._move_pose_down_ompl(xyz)

    def _move_pose_down_ompl(self, xyz):
        """Fallback used only when we cannot seed IK from a live joint
        state (see move_pose_down). Plans into a pose *region* via OMPL,
        which -- unlike the IK-seeded path above -- cannot guarantee it
        stays on the robot's current kinematic branch."""
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.frame
        pc.link_name = self.ee
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [float(self.m['position_tolerance'])]
        pc.constraint_region.primitives = [region]
        pc.constraint_region.primitive_poses = [_pose(xyz)]
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.ee
        # Centre the allowed yaw on the direction from the robot base to the
        # target (rather than a fixed global heading) so this fallback is at
        # least a reasonable, reachable posture for off-centre targets.
        natural_yaw = math.atan2(xyz[1], xyz[0])
        q_yaw = (0.0, 0.0, math.sin(natural_yaw / 2.0), math.cos(natural_yaw / 2.0))
        oc.orientation = _quat(*_qmul(q_yaw, TOOL_DOWN))
        tol = float(self.m['orientation_tolerance'])
        oc.absolute_x_axis_tolerance = tol
        oc.absolute_y_axis_tolerance = tol
        oc.absolute_z_axis_tolerance = math.radians(45)
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return self._send_move_group(c)

    def move_vertical(self, z):
        tool = self.tool_pose()
        if tool is None:
            return Status.FAILED
        (x, y, _), q = tool
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.start_state.is_diff = True
        req.group_name = self.group
        req.link_name = self.ee
        req.waypoints = [_pose((x, y, z), q)]
        req.max_step = float(self.m['cartesian_step'])
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        _set_if(req, 'max_velocity_scaling_factor', float(self.m['velocity_scaling']) * 0.5)
        _set_if(req, 'max_acceleration_scaling_factor', float(self.m['acceleration_scaling']))
        res = self._wait(self.cart_client.call_async(req), 15.0)
        low_fraction = (res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS
                       and res.fraction < float(self.m['min_cartesian_fraction']))
        if res is None or (res.error_code.val != MoveItErrorCodes.SUCCESS) or low_fraction:
            if low_fraction:
                self.log.warn(f'Cartesian path only {res.fraction * 100:.0f}% feasible; '
                              f'retrying this vertical move via IK + joint-space planning '
                              f'(the straight-line path is likely blocked by a nearby '
                              f'kinematic singularity from this specific starting posture)')
            return self._move_vertical_ik_fallback((x, y, z))
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        handle = self._wait(self.exec_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return Status.EXECUTION_FAILED
        out = self._wait(handle.get_result_async(), 60.0)
        if out is None:
            return Status.EXECUTION_FAILED
        return self._map(out.result.error_code.val)

    def _move_vertical_ik_fallback(self, xyz):
        """Used only when the straight-line Cartesian path for move_vertical
        can't even get started (0% feasible) from the current posture --
        seen when the IK-seeded move_pose_down lands close enough to a
        kinematic singularity that the Jacobian-based Cartesian planner has
        nowhere to go, even though the pose itself looks perfectly ordinary.
        Solves IK for the exact target directly (same current-state seed) and
        commands it as a joint-space goal via move_joints(), which uses
        OMPL's own path planner instead of a Jacobian step -- OMPL can route
        around a singularity that a straight Cartesian line cannot."""
        seed = self._current_joints()
        if seed is None:
            return Status.PLANNING_FAILED
        sol = self._kin.ik_down(xyz, seeds=[seed])
        if sol is None:
            return Status.PLANNING_FAILED
        return self.move_joints(sol)

    # ------------------------------------------------------ virtual gripper
    def attach(self, obj):
        # The grasped cube is attached with a slightly SMALLER collision box
        # (negative padding, `attached_shrink`).  A cube held at its resting
        # height touches the table with zero clearance; MoveIt would report
        # that resting contact as a collision and the retreat motion would
        # fail with START_STATE_IN_COLLISION.  This is exactly what MoveIt's
        # `default_attached_padding` parameter does, which ur_moveit_config
        # leaves at 0.0 and which we cannot set from here.
        s = self.scene.cube_size - float(self.m.get('attached_shrink', 0.008))
        ps = PlanningScene()
        rm = CollisionObject()
        rm.header.frame_id = self.frame
        rm.id = obj
        rm.operation = CollisionObject.REMOVE
        ps.world.collision_objects = [rm]
        if not self._apply(ps):
            return Status.FAILED
        aco = AttachedCollisionObject()
        aco.link_name = self.ee
        aco.object = self._collision_object(obj, (s, s, s), (0, 0, self._grip_offset),
                                            frame=self.ee)
        aco.touch_links = [self.ee, 'wrist_3_link', 'flange']
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [aco]
        if not self._apply(ps):
            return Status.FAILED
        with self._lock:
            self._attached = obj
        return Status.SUCCESS

    def detach(self, obj, center_xyz):
        tool = self.tool_pose()
        q = (0.0, 0.0, 0.0, 1.0)
        if tool is not None:
            q = _yaw_only(self._cube_pose_from_tool(tool)[1])
        with self._lock:
            self._attached = None
        aco = AttachedCollisionObject()
        aco.link_name = self.ee
        aco.object.id = obj
        aco.object.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [aco]
        if not self._apply(ps):
            return Status.FAILED
        s = self.scene.cube_size          # released cube: full size again
        ps = PlanningScene()
        ps.world.collision_objects = [self._collision_object(obj, (s, s, s), center_xyz, q)]
        if not self._apply(ps):
            return Status.FAILED
        self._gz_set(obj, center_xyz, q, wait=True)
        return Status.SUCCESS
