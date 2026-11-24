"""右手 G4、VE33 闭链与 SuperDex 的网页操控入口。"""

import argparse
from contextlib import ExitStack, contextmanager
import ctypes
import fcntl
import json
import math
import multiprocessing as mp
import os
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
import time
# 每帧闭链求解是小矩阵运算，单线程 BLAS 避免与实时仿真抢占。
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np
from scipy.spatial.transform import Rotation
from tempfile import NamedTemporaryFile, TemporaryDirectory
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from ve33_closed_chain_view import FINGER_SLEEVE_LINKS, ROOT, attach_compound_visuals
from motorhand.contact_torque import ContactTorqueProjector
from motorhand.feedback import (CONFIG, FORCE_FRAME_SIZE, FORCE_INPUT_END, FORCE_POSITION_END, FORCE_VELOCITY_END,
                                MAPPING, MOTOR_COUNT, PROXY_FRAME_SIZE, WRIST_STAMP_INDEX, ContactForceFeedbackLoop,
                                ContactProxy)
from motorhand.kinematics import GloveKinematics, KinematicsError
from motorhand.mapping import GloveMapping, MODEL_MOTOR_JOINTS
from motorhand.g4_types import G4Joint
from ve33_web_control import WebControl, measure_frame_gaps

MODEL_DIR = ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R"
MODEL_BOT = MODEL_DIR / "hand_motor_ve40_right_closed_chain.superdex_bot"
LIBRARY = ROOT / "hardware/encos_g4_gateway_sdk/build/libg4_shim.so"
PHYSICS_STEP_S = 1. / 240.
SCENE_VIEW_PERIOD_S = 1. / 60.


class GraspPresentationSchedule:
    """本地抓握画面与诊断降频，物理反馈仍按控制节拍运行。"""

    def __init__(self):
        self.next_at = 0.
        self.revision = None

    def due(self, now_s, revision):
        if revision == self.revision and now_s < self.next_at:
            return False
        self.revision = revision
        self.next_at = now_s + 1. / 30.
        return True


class GatewayBusy(RuntimeError):
    pass


def runtime_dir():
    path = Path(os.environ.get("XDG_RUNTIME_DIR") or Path.home() / ".cache/superdex")
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    return path


def zero_socket(port):
    return runtime_dir() / f"ve33-{port.resolve().name}.sock"


@contextmanager
def gateway_lock(port):
    lock_dir = runtime_dir()
    with (lock_dir / f"ve33-{port.resolve().name}.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise GatewayBusy("该网关已有采集进程") from exc
        yield


@contextmanager
def gateway(port, feedback_capable=False):
    with gateway_lock(port), open_reader(port, feedback_capable) as read:
        yield read


@contextmanager
def open_reader(port, feedback_capable=False):
    lib = ctypes.CDLL(str(LIBRARY))
    if lib.g4_joint_size() != ctypes.sizeof(G4Joint):
        raise RuntimeError("G4 快照结构与驱动版本不匹配")
    lib.g4_open_reader.argtypes = [ctypes.c_char_p]
    lib.g4_open_reader.restype = ctypes.c_void_p
    lib.g4_close.argtypes = [ctypes.c_void_p]
    lib.g4_close.restype = None
    lib.g4_snapshots.argtypes = [ctypes.c_void_p, ctypes.POINTER(G4Joint), ctypes.c_int]
    transport_lost = getattr(lib, "g4_transport_lost", None)
    if transport_lost is not None:
        transport_lost.argtypes = [ctypes.c_void_p]
        transport_lost.restype = ctypes.c_int
    open_fresh = getattr(lib, "g4_open_fresh", None)
    if open_fresh is not None:
        open_fresh.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
        open_fresh.restype = ctypes.c_void_p
    port_bytes = str(port).encode()
    if feedback_capable:
        lib.g4_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
        lib.g4_open.restype = ctypes.c_void_p
        batch_send = getattr(lib, "g4_renew_mit_batch", None)
        if batch_send is None:
            raise RuntimeError("缺少 G4 批量指令接口，请按 SDK README 重新编译 shim")
        batch_send.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int,
                               ctypes.c_int64, ctypes.c_int64]
        batch_send.restype = ctypes.c_int
        batch_buffer = (ctypes.c_float * (12 * 5))()
        lib.g4_release.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
        lib.g4_release.restype = None
        handle = lib.g4_open(port_bytes, 921600, 200)
    else:
        handle = lib.g4_open_reader(port_bytes)
    if not handle:
        raise OSError(f"无法打开 G4 网关 {port}")
    buf = (G4Joint * 12)()
    current_times = (ctypes.c_int64 * len(buf))()
    snapshots_with_time = getattr(lib, "g4_snapshots_with_current_time", None)
    if snapshots_with_time is not None:
        snapshots_with_time.argtypes = [ctypes.c_void_p, ctypes.POINTER(G4Joint),
                                         ctypes.POINTER(ctypes.c_int64), ctypes.c_int]
        snapshots_with_time.restype = ctypes.c_int

    class Session:
        _initial_delay_ns = 250_000_000
        _max_delay_ns = 2_000_000_000

        def __init__(self, initial_handle):
            self._handle = initial_handle
            self._delay_ns = self._initial_delay_ns
            self._retry_at_ns = 0
            self._closed = False
            self.reconnect_count = 0

        @property
        def online(self):
            return self._handle is not None

        def poll_transport(self, now_ns=None):
            if self._closed or transport_lost is None:
                return "none"
            now = time.monotonic_ns() if now_ns is None else now_ns
            if self._handle is not None:
                if not transport_lost(self._handle):
                    return "none"
                lib.g4_close(self._handle)
                self._handle = None
                self._delay_ns = self._initial_delay_ns
                self._retry_at_ns = now + self._delay_ns
                return "lost"
            if now < self._retry_at_ns:
                return "none"
            if feedback_capable:
                reopened = open_fresh(port_bytes, 921600, 200) if open_fresh is not None else lib.g4_open(port_bytes, 921600, 200)
            else:
                reopened = lib.g4_open_reader(port_bytes)
            if not reopened:
                self._delay_ns = min(self._delay_ns * 2, self._max_delay_ns)
                self._retry_at_ns = now + self._delay_ns
                return "none"
            self._handle = reopened
            self._delay_ns = self._initial_delay_ns
            self.reconnect_count += 1
            return "restored"

        def close(self):
            self._closed = True
            if self._handle is not None:
                lib.g4_close(self._handle)
                self._handle = None

        def __call__(self):
            if self._handle is None:
                return {}
            if snapshots_with_time is None:
                count = lib.g4_snapshots(self._handle, buf, len(buf))
            else:
                count = snapshots_with_time(self._handle, buf, current_times, len(buf))
            result = {}
            for i, joint in enumerate(buf[:count]):
                joint.current_received_ns = current_times[i]
                result[joint.id] = joint
            return result

        def send(self, batch, deadline):
            return self.send_before(batch, deadline, 0)

        def send_before(self, batch, deadline, previous):
            if not feedback_capable or len(batch) != 12:
                raise RuntimeError("G4 会话未配置 12 轴反馈")
            if self._handle is None:
                raise RuntimeError("G4 串口已断开")
            batch_buffer[:] = [value for command in batch for value in command]
            accepted = batch_send(self._handle, batch_buffer, len(batch), deadline, previous)
            if not accepted:
                raise TimeoutError("G4 指令已过期")
            if accepted < 0:
                raise ValueError("G4 批量指令含无效值")

        def release(self):
            if feedback_capable and self._handle is not None:
                for mid in range(1, 13):
                    lib.g4_release(self._handle, 0, mid)

    session = Session(handle)
    try:
        yield session
    finally:
        session.close()


class HandContactDrive:
    """有限刚度的腕部追踪允许手掌在碰撞时退让。"""

    def __init__(self, sdp, actor, info, motor_names, *, fixed_wrist=False):
        self.actor = actor
        self.sdp = sdp
        n = len(info.link_names)
        self.root = info.dof_info[0]
        self.fixed_wrist = fixed_wrist
        params = sdp.PoseControllerParams(n)
        self.motor_links = [i for i, name in enumerate(list(info.joint_names)[:n]) if name in motor_names]
        if fixed_wrist:
            if self.root.get_size():
                raise ValueError("掌心测试需要固定的手腕根关节")
        else:
            if self.root.trans_size != 3 or self.root.rot_size != 3:
                raise ValueError("接触模式需要浮动的手腕根关节")
            params.link_pos_tracking[0] = sdp.PoseTrackingParams(stiffness=400, damping=6, saturation=.025)
            params.link_rot_tracking[0] = sdp.PoseTrackingParams(stiffness=3, damping=.05, saturation=.25)
        for i, name in enumerate(list(info.joint_names)[:n]):
            if name in motor_names:
                params.joint_tracking[i] = sdp.PoseTrackingParams(stiffness=.6, damping=.005, saturation=.2)
            elif info.dof_info[i].rot_size == 1:
                # 弱回正使卸载后的被动连杆回到求解器选定的装配分支。
                params.joint_tracking[i] = sdp.PoseTrackingParams(stiffness=.09, damping=.002, saturation=.25)
        actor.add_articulated_pose_controller(params)
        self.params = params

    def set_free_grip(self, enabled):
        # 只切换虚拟电机轴的姿态驱动，腕部和闭链的被动轴使用原增益。
        gains = (1.5, .012, .13) if enabled else (.6, .005, .2)
        for i in self.motor_links:
            self.params.joint_tracking[i] = self.sdp.PoseTrackingParams(
                stiffness=gains[0], damping=gains[1], saturation=gains[2])
        self.actor.set_articulated_pose_controller_params(self.params)

    def set_target(self, pose, wrist):
        if not self.fixed_wrist:
            for i, x in enumerate(wrist[:3]):
                pose[self.root.get_trans_offset() + i] = x
            for i, x in enumerate(Rotation.from_quat(wrist[3:]).as_rotvec()):
                pose[self.root.get_rot_offset() + i] = x
        self.actor.reset_articulated_target_pose(pose)

class ShellContacts:
    """统计手与场景内可交互表面的接触。"""

    def __init__(self, sdp, scene, hand, surfaces):
        self.hand = [(name, scene.get_actor(handle)) for name, handle in
                     zip(hand.get_articulated_shape_info().link_names, hand.get_nested_link_actors())]
        self.shells = surfaces
        self.hand = [(name, actor) for name, actor in self.hand
                     if actor.is_query_supported(sdp.QueryType.TOTAL_CONTACT_FORCE)]
        for _, actor in self.hand:
            actor.register_query(sdp.QueryType.TOTAL_CONTACT_FORCE)

    def read(self, measured=None):
        contacts, forces = {}, {}
        for surface_id, (name, shell) in enumerate(self.shells):
            force_row = measured[surface_id] if measured is not None else (
                actor.get_contact_force_from_actor_world(shell) for _, actor in self.hand)
            for (link, _), force in zip(self.hand, force_row):
                if sum(x*x for x in force) > 1e-6 and all(map(math.isfinite, force)):
                    contacts.setdefault(name, []).append(link)
                    total = forces.setdefault(link, [0., 0., 0.])
                    for i, value in enumerate(force):
                        total[i] += value
        return contacts, forces

    def set_grip_surfaces(self, sdp, scene, enabled):
        params = sdp.ContactPairParamsOverride(coulomb_friction_coefficient=1.25,
                                               friction_falloff_vel=.001,
                                               normal_viscous_damping_coefficient=1.)
        for name, pad in self.hand:
            if name not in ("thumb_ip_link", "index_dip_link"):
                continue
            for _, shell in self.shells:
                pair = (pad.get_handle(), shell.get_handle())
                if enabled:
                    scene.set_contact_pair_params_override(*pair, params)
                else:
                    scene.clear_contact_pair_params_override(*pair)

    def clear_of_shells(self, fingers):
        if not fingers:
            return set()
        bounds = [shell.get_aabb_world() for _, shell in self.shells]
        low = np.min([box.min for box in bounds], axis=0)
        high = np.max([box.max for box in bounds], axis=0)
        clear = set(fingers)
        for name, actor in self.hand:
            finger = name.split("_")[0]
            if finger in clear:
                box = actor.get_aabb_world()
                gap = np.maximum(0, np.maximum(low - np.asarray(box.max), np.asarray(box.min) - high))
                if np.linalg.norm(gap) <= CONFIG["stale_release_clearance_m"]:
                    clear.remove(finger)
        return clear


class MeasuredFingers:
    """仅保留物理求解后的腕部自由度，指节必须位于实测或受阻代理的闭链姿态。"""

    def __init__(self, actor, tree_names, indices, sdp):
        self.actor, self.indices = actor, indices
        self.pose = sdp.DynamicArrayReal(actor.get_num_dofs())
        self.vel = sdp.DynamicArrayReal(actor.get_num_dofs())
        actor.get_articulated_pose(self.pose)
        self.initial_pose = tuple(self.pose)
        self.mapping = GloveMapping.load(MAPPING)
        self.kin = GloveKinematics(MODEL_DIR, self.mapping)
        self.rows = [self.kin.joint_names.index(n) for n in tree_names]
        self.motor_rows = [list(tree_names).index(n) for n in MODEL_MOTOR_JOINTS]
        self.proxy = ContactProxy()
        self.finger_names = np.array([name.split("_")[0] for name in tree_names])
        self.last = np.zeros(len(tree_names))
        self.solved_motor = None
        self.solved_tree = None
        self.valid = False

    def reset(self, target_tree):
        self.proxy = ContactProxy()
        self.last = target_tree.copy()
        self.solved_motor = None
        self.solved_tree = None
        for i, angle in enumerate(self.initial_pose):
            self.pose[i] = angle
            self.vel[i] = 0.
        for index, angle in zip(self.indices, self.last):
            self.pose[index] = angle
        self.actor.set_articulated_pose_from_joints(self.pose)
        self.actor.set_articulated_joint_velocities(self.vel)

    def angles(self, target_tree):
        q = target_tree[self.motor_rows]
        cmd = self.proxy.visible(q)
        try:
            if np.array_equal(q, cmd):
                self.last = target_tree.copy()
            else:
                if not np.array_equal(cmd, self.solved_motor):
                    solved = self.kin.full_pose(cmd, None, self.mapping, slot="visual")
                    self.solved_tree = solved[self.rows]
                    self.solved_motor = cmd.copy()
                self.last = self.solved_tree.copy()
            self.valid = True
        except (KinematicsError, ValueError):
            self.valid = False
        return self.last

    def guarded_angles(self, target, guard, *, publish_blocks):
        previous = self.last.copy()
        candidate = self.angles(target)
        if np.array_equal(previous, candidate):
            return {}
        start = guard.points()
        self.project(candidate)
        hits = guard.sweep(start, guard.points())[2]
        if hits:
            held = {finger for links in hits.values() for finger in guard.fingers[links]}
            for finger in held:
                mask = self.finger_names == finger
                self.last[mask] = previous[mask]
            if publish_blocks and self.valid:
                self.proxy.hold_swept(target[self.motor_rows], self.last[self.motor_rows], held)
            self.project(self.last)
        return hits

    def project(self, angles):
        self.actor.get_articulated_pose(self.pose)
        self.actor.get_articulated_joint_velocities(self.vel)
        for i, val in zip(self.indices, angles):
            self.pose[i] = val
            self.vel[i] = 0.
        self.actor.set_articulated_pose_from_joints(self.pose)
        self.actor.set_articulated_joint_velocities(self.vel)


def limit_wrist(guard, fingers, drive, before, start):
    """腕部可沿接触面平移，向着表面内侧的位移由扫描截断。"""
    fraction, normal, hits = guard.sweep(start, guard.points())
    if normal is None:
        return hits
    translation = drive.root.get_trans_offset()
    rotation = drive.root.get_rot_offset()
    ts, rs = slice(translation, translation + 3), slice(rotation, rotation + 3)
    after = np.asarray(fingers.pose).copy()
    travel = after[ts] - before[ts]
    initial = Rotation.from_rotvec(before[rs])
    change = Rotation.from_rotvec(after[rs]) * initial.inv()
    fraction = max(0., fraction - .001)
    wrist = before[ts] + fraction * travel
    quat = Rotation.from_rotvec(change.as_rotvec() * fraction) * initial

    def move(position):
        for i, value in enumerate(position, translation):
            fingers.pose[i] = value
            fingers.vel[i] = 0.
        for i, value in enumerate(quat.as_rotvec(), rotation):
            fingers.pose[i] = value
            fingers.vel[i] = 0.
        fingers.actor.set_articulated_pose_from_joints(fingers.pose)
        fingers.actor.set_articulated_joint_velocities(fingers.vel)

    move(wrist)
    remaining = (1. - fraction) * travel
    tangent = remaining - min(0., remaining @ normal) * normal
    if np.linalg.norm(tangent) > 1e-6:
        points = guard.points()
        slide, _, _ = guard.sweep(points, points + tangent)
        move(wrist + tangent * max(0., slide - .001))
    return hits


def publish_contact_frame(output, proxy, stamp, simulated_at, valid, palm_contact, wrist_stamp=0):
    surfaces = [proxy.blocks[i][1] if i in proxy.blocks else math.nan for i in range(12)]
    directions = [proxy.blocks[i][0] if i in proxy.blocks else 0 for i in range(12)]
    output[:] = [stamp, simulated_at, valid, *surfaces, *directions, palm_contact, wrist_stamp]


def read_feedback_display(source, requested=False, *, revision=None, frame_ns=None):
    if source is None or len(source) < 26:
        return None
    stamp, active, *values = source[:]
    now = time.monotonic_ns()
    if not 0 <= now - stamp < int(CONFIG["command_ttl_s"] * 1e9):
        return None
    if revision is not None:
        if len(values) < 26 or values[24] != revision or not now < values[25]:
            return None
        if frame_ns is not None and (len(values) != 27 or values[26] != frame_ns):
            return None
    nominal = [n if math.isfinite(n) else None for n in values[:12]]
    current = [n if math.isfinite(n) else None for n in values[12:24]]
    result = {"active": bool(active), "requested": requested, "nominal_nm": nominal, "current_a": current}
    if revision is not None:
        result["expires_ns"] = int(values[25])
    if frame_ns is not None:
        result["contact_frame_ns"] = int(frame_ns)
    return result


def aligned_feedback_display(read, frame, requested, *, wait_s):
    stamp = frame["physics_frame_ns"]
    if not stamp:
        return None
    grasp = frame.get("grasp_object")
    revision = grasp["contact_revision"] if grasp is not None else frame.get("scene_revision", 0)
    deadline = time.monotonic() + wait_s
    while True:
        data = read(stamp, revision)
        if data:
            return read_feedback_display(data, requested, revision=revision, frame_ns=stamp)
        left = deadline - time.monotonic()
        if left <= 0:
            return None
        time.sleep(min(.001, left))


def render(tree_names, motor_names, pending, displayed, stop, debugger, quest_port=None, cube_dir=None, align=None, wrist_state=None, headset=False, proxy_frame=None, tabletop=False, palm_test=False, official_puzzle=False, ball_size_mm=45., feedback_display=None, web_control=None, control_port=None, force_frame=None, feedback_mapping=None, use_cpp=False, shared_path=None, wrist_offset=(0., 0., 0.), wrist_pose=None):
    import superdex.physics as sdp
    import superdex.robotics as sdr

    grasp_settings = web_control.grasp_request() if web_control is not None else None
    local_grasp = grasp_settings is not None
    # 大球满握由物理工作线程求解，给 G4 与网页留出 CPU。
    workers = min(6, max(0, len(os.sched_getaffinity(0)) - 2)) if use_cpp else 0
    sdp.initialize(num_worker_threads=workers)
    bot = None
    view = None
    grasp_physics = None
    shared_link = None
    bridge = environment = left_support = None
    puzzle_table = None
    services = ExitStack()
    fixed_wrist = palm_test or official_puzzle or local_grasp
    puzzle_on_table = bool(quest_port) and not (fixed_wrist or tabletop)
    try:
        scene = sdp.create_scene("Puzzle Cube · MotorHand grip_lift" if official_puzzle else "VE33 G4 - right")
        scene.set_gravity([0., 0., -9.81 if (puzzle_on_table or tabletop) else 0.])
        context = sdr.create_context()
        prefab = sdr.load_bot_prefab_from_file(str(MODEL_BOT))
        if local_grasp:
            from ve33_local_grasp import UPRIGHT_WRIST_POSE
            prefab.world_from_root = sdp.TransformRT(rotation=UPRIGHT_WRIST_POSE[3:], translation=UPRIGHT_WRIST_POSE[:3])
        with TemporaryDirectory(prefix="ve33-") as folder:
            if fixed_wrist:
                prefab.joints[0].type = sdp.ArticulatedJointType.HARD
            if quest_port:
                # 从手表面按面积采样并查询外壳现成的 SDF。反向检测会让每个
                # 角块反复查询复合手部的高精度三角网格，按压时成本很高。
                for link in prefab.links:
                    link.layer = "RightHand"
                    if puzzle_on_table or tabletop:
                        link.has_gravity = False
                    link.boundary_element_type = sdp.ActorBoundaryElementType.P1Q1
                    link.boundary_subsampling = sdp.BoundarySubsamplingParams(
                        strategy=sdp.BoundarySubsamplingStrategy.AREA_PROPORTIONAL,
                        subsampling_density=.10,
                    )
                scene.enable_layer_contact_symmetric("RightHand", "RightHand", False)
            if debugger or quest_port:
                attach_compound_visuals(prefab, MODEL_DIR, folder, sdp, full_collision=bool(quest_port), scene=scene)
            bot = sdr.create_bot(scene, prefab, context)
        actor = bot.get_articulated_actor()
        info = actor.get_articulated_shape_info()
        offsets = {name: dof.offset for name, dof in zip(info.joint_names, info.dof_info)
                   if name in tree_names and dof.trans_size == 0 and dof.rot_size == 1}
        indices = [offsets[name] for name in tree_names]
        pose = sdp.DynamicArrayReal(actor.get_num_dofs())
        actor.get_articulated_pose(pose)
        velocities = sdp.DynamicArrayReal(actor.get_num_dofs())
        if debugger:
            sdp.get_debug_server().set_coordinate_space(sdp.CoordinateSpace(axes=sdp.CoordinateSpaceAxes.FLU))
            if web_control is not None and control_port is not None:
                from ve33_quest import WristBridge as DebugWristBridge, wrist_server as debug_wrist_server
                services.enter_context(debug_wrist_server(DebugWristBridge(), control_port, control=web_control))
            if not sdp.debugger.attach():
                raise RuntimeError("SuperDex 调试器连接失败")
        else:
            from ve33_material_view import MaterialView
            view = MaterialView(MODEL_DIR, info.link_names, ROOT / "superdex_studio/assets/ibl/studio_small_08_1k.hdr",
                                fit_workspace=bool(quest_port))
            transforms = sdp.DynamicArrayTransformRT(len(info.link_names))
        if quest_port:
            from ve33_cube import PuzzleCube, link_poses
            from ve33_quest import IDENTITY, WristBridge, wrist_server
            if local_grasp:
                from ve33_local_grasp import LocalGrasp, grasp_needs_rebuild
                from ve33_tabletop import Tabletop
                environment = LocalGrasp(sdp, scene, grasp_settings, hand=actor)
                puzzle_table = Tabletop(sdp, scene, view, grasp_table=True)
                scene.enable_layer_contact_asymmetric("GraspRigid", "RightHand", False)
                scene.enable_layer_contact_asymmetric("puzzle_cube_shell", "RightHand", False)
                # 软球的表面采样点查询手的碰撞面。
                scene.enable_layer_contact_asymmetric("RightHand", "GraspSoft", False)
                collection = "objects"
            elif palm_test:
                from ve33_palm_object import PalmBall
                environment = PalmBall(sdp, scene, view, ball_size_mm)
                collection = "objects"
                view.window.SetWindowName(f"MotorHand · 本地掌心球 Ø{ball_size_mm:g} mm")
            elif tabletop:
                from ve33_tabletop import Tabletop
                environment = Tabletop(sdp, scene, view)
                collection = "objects"
            else:
                switchable = web_control is not None and web_control.cube_fixed is not None
                environment = PuzzleCube(sdp, scene, cube_dir, view, official_grip=official_puzzle,
                                         switchable=switchable)
                if puzzle_on_table:
                    from ve33_tabletop import Tabletop
                    puzzle_table = Tabletop(sdp, scene, view, puzzle_shells=environment.contact_actors)
                if official_puzzle:
                    view.window.SetWindowName("MotorHand · 官方 Puzzle Cube")
                    view.window.SetSize(1440, 1000)
                scene.enable_layer_contact_asymmetric("puzzle_cube_shell", "RightHand", False)
                collection = "puzzle_cube"
            params = scene.get_solver_params()
            params.non_linear_solver.max_elapsed_time_seconds = .009
            scene.set_solver_params(params)
            fingers = MeasuredFingers(actor, tree_names, indices, sdp)
            drive = HandContactDrive(sdp, actor, info, motor_names, fixed_wrist=fixed_wrist)
            contact_targets = environment.dynamic_actors if tabletop else environment.contact_actors
            contacts = ShellContacts(sdp, scene, actor, contact_targets)
            if local_grasp:
                drive.set_free_grip(True)
            elif tabletop:
                drive.set_free_grip(True)
                contacts.set_grip_surfaces(sdp, scene, True)
            motor_indices = [offsets[name] for name in motor_names]
            if use_cpp:
                from motorhand.native_runtime import load_native
                link_names, links = zip(*contacts.hand)
                palm_index = link_names.index("base_link") if "base_link" in link_names else -1
                grasp_physics = load_native().GraspPhysics(scene, actor, links, indices, motor_indices, palm_index)
                if shared_path is not None:
                    from motorhand.native_kinematics import NativeGloveKinematics
                    shared_link = load_native().SharedLink(str(shared_path))
                    native_kin = NativeGloveKinematics(MODEL_DIR, feedback_mapping or fingers.mapping).native
                    if local_grasp:
                        def connect_motion():
                            obj = environment.prefab
                            if obj is not None:
                                rigid, fixed, soft = obj.roots, obj.static_roots, obj.soft
                                center, axis = obj.center, obj.yaw_axis
                            else:
                                soft = environment.actor if environment.soft else None
                                rigid = [] if environment.soft else [environment.actor]
                                fixed = []
                                center = environment.origin
                                axis = environment.rotation.apply([0., 0., 1.])
                            center_node = (obj or environment).node if soft is not None else -1
                            grasp_physics.set_motion(rigid, fixed, [body for _, body in environment.contact_actors],
                                                     [soft] if soft is not None else [], center_node,
                                                     center, axis, environment.settings["angle_deg"])
                        connect_motion()
                        grasp_physics.motion_command(grasp_settings["locked"], grasp_settings["angle_deg"],
                                                    grasp_settings["drag"] or [])
            if force_frame is not None and shared_link is None:
                projector = ContactTorqueProjector(contacts.hand, fingers.kin,
                                                   feedback_mapping or fingers.mapping, indices, motor_indices)
                if grasp_physics is None:
                    contact_pose = sdp.DynamicArrayReal(actor.get_num_dofs())
                    contact_vel = sdp.DynamicArrayReal(actor.get_num_dofs())
            bridge = WristBridge(controller_offset=wrist_offset, controller_to_hand=wrist_pose)
            if not fixed_wrist and (collection == "puzzle_cube" or tabletop):
                from ve33_left_hand import LeftHandSupport
                left_shells = environment.dynamic_actors if tabletop else environment.contact_actors
                left_support = LeftHandSupport(sdp, scene, left_shells)
            from ve33_contact_guard import CubeContactGuard
            guard = None
            if not (local_grasp or tabletop):
                solid_surfaces = {name: scene.get_actor(handle) for name, handle in
                                  zip(info.link_names, actor.get_nested_link_actors())
                                  if name in FINGER_SLEEVE_LINKS}
                guard = CubeContactGuard(sdp, actor, view, info.link_names, contacts.shells,
                                         solid_surfaces=solid_surfaces,
                                         sphere_radii=environment.sphere_radii if palm_test else None)
            last_registration = None
            from ve33_quest_mesh import scene_glb
            support_groups = puzzle_table.groups if puzzle_table is not None else None
            glb = scene_glb(view.groups, {} if local_grasp else environment.groups,
                            collection_name=collection, support_groups=support_groups)
            # 在 HTTP 请求线程匹配已完成的物理帧与控制周期，物理线程无需等待反馈。
            scene_feedback = None
            if shared_link is not None:
                def scene_feedback(frame):
                    return aligned_feedback_display(shared_link.read_display_for, frame,
                                                    web_control.requested.is_set(), wait_s=1./CONFIG["control_hz"])
            services.enter_context(wrist_server(
                bridge, quest_port, glb, control=web_control, scene_feedback=scene_feedback,
                offset_config_path=ROOT / "hardware/encos_g4_gateway_sdk/config/right_sim_feedback.json"))
        frame_id = 0
        scene_revision = 0
        last_view = 0.
        palm_contact = False
        was_cube_free = False
        joint_count = len(tree_names)
        no_encoder_sample = [math.nan] * MOTOR_COUNT
        presentation = GraspPresentationSchedule()
        pairs, contact_forces = {}, {}

        def read_target():
            stamp, *values = shared_link.read_target() if shared_link is not None else pending[:]
            encoder = values[joint_count:]
            return stamp, values[:joint_count], encoder if len(encoder) == MOTOR_COUNT else no_encoder_sample

        parent = mp.parent_process()
        while not stop.is_set() and (parent is None or parent.is_alive()):
            frame_start = time.monotonic()
            if (view and view.closed) or (debugger and not sdp.debugger.is_attached()):
                break
            if shared_link is None or not local_grasp:
                stamp, angles, encoder_sample = read_target()
            if bridge:
                if shared_link is None or not local_grasp:
                    target_tree = np.asarray(angles)
                reset_grasp = False
                if web_control is not None and web_control.scene_revision is not None:
                    if local_grasp:
                        revision, settings = web_control.grasp_scene_request()
                        cube_fixed = None
                    else:
                        cube_fixed, revision = web_control.scene_request()
                    if scene_revision != revision:
                        static_objects = puzzle_table
                        if local_grasp:
                            reset_grasp = True
                        elif hasattr(environment, "reset"):
                            environment.reset()
                        else:
                            static_objects = environment
                        if static_objects is not None:
                            for (_, prop), transform in zip(static_objects.contact_actors, static_objects.transforms):
                                prop.set_root_transform(transform)
                        bridge.reset_scene()
                        fingers.reset(np.asarray(read_target()[1]) if shared_link is not None else target_tree)
                        if left_support is not None:
                            left_support.update(None, 1 / 240)
                        if guard is not None:
                            guard.previous_target_local = None
                        if shared_link is not None and not local_grasp:
                            shared_link.invalidate_force()
                        palm_contact = False
                        last_registration = None
                        scene_revision = revision
                    if cube_fixed is not None:
                        environment.set_fixed(cube_fixed)
                if local_grasp:
                    reset_grasp |= settings.get("home", 0) != grasp_settings.get("home", 0)
                    if reset_grasp or settings != grasp_settings:
                        # 场景切换后只发布新物体算出的接触反馈。
                        replacing = reset_grasp or grasp_needs_rebuild(grasp_settings, settings)
                        if replacing and shared_link is not None:
                            shared_link.invalidate_force()
                            grasp_physics.clear_motion()
                        if replacing and force_frame is not None:
                            force_frame[2] = 0.
                        if environment.configure(settings, reset=reset_grasp):
                            contacts.shells = environment.contact_actors
                            if shared_link is not None:
                                connect_motion()
                        if shared_link is not None:
                            grasp_physics.motion_command(settings["locked"], settings["angle_deg"], settings["drag"] or [])
                        grasp_settings = settings
                if fixed_wrist:
                    root_pose = UPRIGHT_WRIST_POSE if local_grasp else IDENTITY
                    tracking, registration = "固定腕部", None
                    wrist_stamp = 0
                else:
                    calibrate = align.is_set()
                    if calibrate:
                        align.clear()
                    root_pose, tracking, registration, wrist_stamp = bridge.wrist(calibrate)
                if web_control is not None and web_control.wrist_stamp is not None:
                    web_control.wrist_stamp.value = int(wrist_stamp)
                if registration is not None and registration is not last_registration:
                    # 重新对齐会改变腕部参考位置，旧的接触面需要重建。
                    fingers.proxy = ContactProxy()
                    if guard is not None:
                        guard.previous_target_local = None
                    palm_contact = False
                    last_registration = registration
                cube_free = collection == "puzzle_cube" and environment.mode == "free"
                if cube_free != was_cube_free:
                    # 切换支撑方式后，旧参考位姿建立的受阻面需要重新探测。
                    fingers.proxy = ContactProxy()
                    if guard is not None:
                        guard.previous_target_local = None
                    palm_contact = False
                    drive.set_free_grip(cube_free)
                    contacts.set_grip_surfaces(sdp, scene, cube_free)
                    if not cube_free:
                        actor.get_articulated_pose(pose)
                        fingers.last = np.asarray(pose)[indices]
                    was_cube_free = cube_free
                native_grasp = cube_free or local_grasp or tabletop
                if native_grasp:
                    # 自由抓取用真实接触，保留求解后的姿态和速度以持续夹持。
                    target_contacts, target_fingers, finger_hits = {}, set(), {}
                    if shared_link is None or not local_grasp:
                        angles = target_tree
                else:
                    q = target_tree[fingers.motor_rows]
                    for index, angle in zip(indices, target_tree):
                        pose[index] = angle
                    drive.set_target(pose, root_pose)
                    previous = fingers.proxy.previous
                    compare_motion = previous is not None and np.any(q < previous - 1e-12)
                    target_contacts, deepening = guard.target_contacts(root_pose, compare_motion)
                    target_fingers = {str(guard.fingers[i]) for links in target_contacts.values() for i in links}
                    fingers.proxy.update(q, target_fingers if stamp else set(), set(),
                                         deepening=deepening if stamp else ())
                    finger_hits = fingers.guarded_angles(target_tree, guard, publish_blocks=bool(stamp))
                    angles = fingers.last
                if proxy_frame is not None and shared_link is None:
                    # 转层步进可能拖慢画面，几何受阻角已可独立交给电机线程。
                    publish_contact_frame(proxy_frame, fingers.proxy, stamp, time.monotonic_ns(),
                                          bool(stamp and (native_grasp or fingers.valid)),
                                          palm_contact or "base" in target_fingers, wrist_stamp)
            if not (bridge and native_grasp):
                for index, angle in zip(indices, angles):
                    pose[index] = angle
            if bridge:
                wrist_hits = {}
                left = bridge.left_hand() if left_support is not None else None
                left_probes = None
                guard_wrist = not (fixed_wrist or native_grasp)
                if grasp_physics is not None and shared_link is None:
                    grasp_target = list(pose)
                if shared_link is not None and local_grasp:
                    stamp = grasp_physics.run_until(shared_link, native_kin, PHYSICS_STEP_S,
                                                    round(1e9 / CONFIG["control_hz"]),
                                                    round((frame_start + 1/30) * 1e9),
                                                    grasp_settings["contact_revision"])
                elif shared_link is not None:
                    if not fixed_wrist:
                        root_target = [*root_pose[:3], *Rotation.from_quat(root_pose[3:]).as_rotvec()]
                        grasp_physics.set_root_target(drive.root.get_trans_offset(), drive.root.get_rot_offset(), root_target)
                    if native_grasp:
                        budget_end = time.monotonic_ns() + round(1e9 / CONFIG["control_hz"])
                        if left_support is None:
                            _, stamp = grasp_physics.step_contact(shared_link, native_kin, PHYSICS_STEP_S,
                                                                  budget_end, 4, True, scene_revision)
                        else:
                            for substep in range(4):
                                left_probes = left_support.update(left, PHYSICS_STEP_S)
                                last = substep == 3 or time.monotonic_ns() >= budget_end
                                published, stamp = grasp_physics.step_contact(shared_link, native_kin, PHYSICS_STEP_S,
                                                                              budget_end, 1, last, scene_revision)
                                if published or last:
                                    break
                    else:
                        for substep in range(4):
                            if left_support is not None:
                                left_probes = left_support.update(left, PHYSICS_STEP_S)
                            if guard_wrist:
                                points = guard.points()
                                before = np.asarray(fingers.pose).copy()
                            drive.set_target(pose, root_pose)
                            scene.step(PHYSICS_STEP_S)
                            last = substep == 3 or time.monotonic() - frame_start >= .010
                            if last:
                                grasp_physics.publish_contact(
                                    shared_link, native_kin, float(stamp),
                                    [float(x) for x in target_tree[fingers.motor_rows]],
                                    [float(x) for x in encoder_sample], scene_revision)
                            fingers.project(angles)
                            if guard_wrist:
                                wrist_hits.update(limit_wrist(guard, fingers, drive, before, points))
                            if last:
                                break
                else:
                    for substep in range(4):
                        if local_grasp:
                            environment.step(PHYSICS_STEP_S)
                        if left_support is not None:
                            left_probes = left_support.update(left, PHYSICS_STEP_S)
                        if guard_wrist:
                            points = guard.points()
                            before = np.asarray(fingers.pose).copy()
                        if native_grasp:
                            # 同一轮多个子步各自获取最新输入，发布实际驱动末子步的那组采样。
                            stamp, angles, encoder_sample = read_target()
                            target_tree = np.asarray(angles)
                            target = grasp_target if grasp_physics is not None else pose
                            for index, angle in zip(indices, angles):
                                target[index] = angle
                        if grasp_physics is not None:
                            grasp_physics.step(grasp_target, PHYSICS_STEP_S)
                        else:
                            drive.set_target(pose, root_pose)
                            scene.step(PHYSICS_STEP_S)
                        last = substep == 3 or time.monotonic() - frame_start >= .010
                        if last and force_frame is not None:
                            contact_stamp = time.monotonic_ns()
                            valid = False
                            torque = speed = joint_positions = np.zeros(12)
                            if stamp:
                                try:
                                    if grasp_physics is not None:
                                        usable, joint_positions, speed, tree_torque = grasp_physics.effort()
                                        if usable and all(np.isfinite(values).all() for values in (tree_torque, speed, joint_positions)):
                                            torque = projector.project_tree(tree_torque, joint_positions)
                                        else:
                                            usable = False
                                    else:
                                        usable = scene.get_solver_stats().convergence_status != sdp.ConvergenceStatus.DIVERGED
                                        if usable:
                                            actor.get_articulated_pose(contact_pose)
                                            actor.get_articulated_joint_velocities(contact_vel)
                                            torque = projector.project(contact_pose)
                                            speed = np.asarray(contact_vel)[motor_indices]
                                            joint_positions = np.asarray(contact_pose)[motor_indices]
                                    valid = usable and all(np.isfinite(values).all() for values in (torque, speed, joint_positions))
                                except (KinematicsError, ValueError, sdp.Error):
                                    pass
                            effort_values = [stamp, contact_stamp, bool(valid), *torque, *speed, *joint_positions,
                                             *target_tree[fingers.motor_rows], *encoder_sample,
                                             grasp_settings["contact_revision"] if local_grasp else 0]
                            force_frame[:] = effort_values[:len(force_frame)]
                        if not native_grasp:
                            fingers.project(angles)
                        if guard_wrist:
                            wrist_hits.update(limit_wrist(guard, fingers, drive, before, points))
                        # 碰撞复杂时先发布已完成的步，下一帧立即使用最新手柄输入。
                        if last or (force_frame is None and time.monotonic() - frame_start >= .010):
                            break
            elif not bridge:
                actor.set_articulated_pose_from_joints(pose)
                actor.set_articulated_joint_velocities(velocities)
                scene.step(1e-6)
            show_scene = (not local_grasp or presentation.due(
                time.monotonic(), (scene_revision, grasp_settings["contact_revision"])))
            if environment and show_scene:
                measured = (grasp_physics.contacts([body for _, body in contacts.shells])
                            if grasp_physics is not None else None)
                pairs, contact_forces = contacts.read(measured)
                if (local_grasp and environment.awaiting_hand_clearance
                        and 0 <= time.monotonic_ns() - stamp < int(CONFIG["command_ttl_s"] * 1e9)
                        and (grasp_physics.last_timing_ms()[0] >= 0 if shared_link is not None
                             else force_frame is not None and force_frame[2])
                        and environment.ready_to_return(bool(pairs.get("grasp_object")), time.monotonic())):
                    web_control.return_grasp_when_clear(grasp_settings)
                for shell, links in [*finger_hits.items(), *wrist_hits.items()]:
                    pairs[shell] = sorted(set(pairs.get(shell, ())) | {str(info.link_names[i]) for i in links})
                touched = {link.split("_")[0] for group in pairs.values() for link in group}
                blocked_fingers = {MODEL_MOTOR_JOINTS[i].split("_")[0] for i in fingers.proxy.blocks}
                # 与参考仿真一致，目标穿过后即使暂时无碰撞，仍保留
                # 已受阻角，直到实测角回撤或显示手确实远离所有角块。
                clear = contacts.clear_of_shells(blocked_fingers - touched - target_fingers)
                for i in list(fingers.proxy.blocks):
                    if MODEL_MOTOR_JOINTS[i].split("_")[0] in clear:
                        del fingers.proxy.blocks[i]
                palm_contact = "base" in target_fingers or "base" in touched
                if proxy_frame is not None and shared_link is None:
                    publish_contact_frame(proxy_frame, fingers.proxy, stamp, time.monotonic_ns(),
                                          bool(stamp and (native_grasp or fingers.valid)), palm_contact, wrist_stamp)
            if shared_link is None:
                actor.get_articulated_pose(pose)
                displayed[:] = [stamp, *(pose[offsets[name]] for name in motor_names)]
            if view and show_scene:
                actor.get_articulated_link_transforms(transforms)
                if environment:
                    if hasattr(environment, "update"):
                        environment.update()
                    frame_id += 1
                    if pairs or palm_contact:
                        contacted = "魔方"
                        if local_grasp:
                            contacted = "抓握物体"
                        elif palm_test:
                            contacted = "掌心球"
                        elif tabletop:
                            from ve33_tabletop import OBJECT_LABELS
                            hit_names = [OBJECT_LABELS.get(k, k) for k in pairs if k != "table"]
                            contacted = "、".join(hit_names[:2]) if hit_names else "桌面"
                        contact_status = (" · 掌部接触" if palm_contact else " · 接触") + contacted
                    else:
                        contact_status = " · 自由空间"
                    wrist_detail = tracking if fixed_wrist else bridge.tracking_detail()
                    if not fixed_wrist and tracking != "右腕跟踪中":
                        wrist_detail += " · 手腕保持"
                    wrist_state.value = (wrist_detail + contact_status).encode()
                    frame = {"version": 1, "frame": frame_id, "time_monotonic_s": time.monotonic(),
                             "basis": "FLU", "length_unit": "m", "pose_order": "x y z qx qy qz qw",
                             "wrist_status": tracking, "wrist_detail": wrist_detail, "registration": registration,
                             "wrist_received_ns": wrist_stamp,
                             "wrist_target": list(map(float, root_pose)),
                             "right_hand": link_poses(info.link_names, transforms),
                             "motor_received_ns": int(stamp),
                             "scene_kind": ("local_grasp" if local_grasp else "official_puzzle" if official_puzzle else
                                            "palm_test" if palm_test else "tabletop" if tabletop else "puzzle_cube"),
                             "finger_control": "physics" if native_grasp else "contact_guard",
                             collection: {} if local_grasp else link_poses(environment.names, environment.transforms),
                             "hand_cube_contacts": pairs,
                             "right_contact_force_n": contact_forces,
                             "motor_feedback": read_feedback_display(feedback_display,
                                 web_control is not None and web_control.requested.is_set()),
                             "left_hand_tracking": left,
                             "left_contact_probes": left_probes,
                             "target_cube_contacts": {shell: [str(info.link_names[i]) for i in links]
                                                      for shell, links in target_contacts.items()},
                             "blocked_motor_ids": [i + 1 for i in sorted(fingers.proxy.blocks)],
                             "blocked_extension_motor_ids": [i + 1 for i, (sign, _, _) in fingers.proxy.blocks.items()
                                                             if sign < 0],
                             "palm_contact": palm_contact}
                    if collection == "puzzle_cube" and environment.switchable:
                        frame["cube_mode"] = environment.mode
                    if web_control is not None and web_control.scene_revision is not None:
                        frame["scene_revision"] = scene_revision
                    if puzzle_table is not None:
                        frame["objects"] = link_poses(puzzle_table.names, puzzle_table.transforms)
                    if local_grasp:
                        if shared_link is not None and environment.prefab is not None:
                            environment.prefab.center = np.asarray(grasp_physics.motion_center())
                        frame["grasp_object"] = environment.packet()
                    if shared_link is not None:
                        frame["physics_frame_ns"] = int(grasp_physics.last_contact_frame_ns()) if stamp else 0
                        input_ms, step_ms, publish_ms = grasp_physics.last_timing_ms()
                        frame["physics_timing_ms"] = {"input_age": input_ms, "last_substep": step_ms,
                                                      "sample_to_publish": publish_ms}
                    bridge.publish_scene(frame)
                if not headset:
                    now = time.monotonic()
                    if not official_puzzle or now - last_view >= .045:
                        view.update(transforms)
                        last_view = now
            period = 1. / CONFIG["control_hz"] if local_grasp else SCENE_VIEW_PERIOD_S if environment else .015
            stop.wait(max(0., period - (time.monotonic() - frame_start)))
    finally:
        if shared_link is not None and grasp_physics is not None:
            grasp_physics.clear_motion()
        services.close()
        if view:
            view.close()
        if bot is not None:
            sdr.destroy_bot(scene, bot)
        sdp.shutdown()


def position_sample(j, mid, now):
    age = (now - j.received_ns) / 1e6 if j and j.received_ns else None
    raw = j.angle_deg if j and j.valid & 1 else None
    if raw is None or not math.isfinite(raw) or age is None or not 0 <= age <= 100:
        return raw, age, "缺失或过期"
    if j.error:
        return raw, age, f"报警 {j.error}"
    if not (1 <= j.bus <= 3):
        return raw, age, "路由错误"
    return raw, age, "在线"


def mapping_identity(axes):
    return {str(mid): [name, config.sign, config.scale] for mid, (name, config) in axes.items()}


def load_zero(path, axes):
    if not path.exists():
        return None
    record = json.loads(path.read_text(encoding="utf-8"))
    if record.get("hand") != "right" or record.get("axes") != mapping_identity(axes):
        raise ValueError("标零记录与当前右手映射不匹配")
    values = {int(mid): float(x) for mid, x in record["zero_deg"].items()}
    if values.keys() != axes.keys() or not all(map(math.isfinite, values.values())):
        raise ValueError("标零记录缺少有效的 12 轴零偏")
    return values


def capture_zero(read, axes, path, *, accept=None):
    # 每轴至少收到多次独立反馈，静止窗口内取均值。
    samples = {mid: [] for mid in axes}
    last_stamps = dict.fromkeys(axes, 0)
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        feedback = read()
        now = time.monotonic_ns()
        for mid in axes:
            j = feedback.get(mid)
            raw, _, state = position_sample(j, mid, now)
            if state != "在线":
                raise ValueError(f"轴 {mid} {state}，标零取消")
            if j.received_ns > last_stamps[mid]:
                samples[mid].append(raw)
                last_stamps[mid] = j.received_ns
        time.sleep(0.015)
    zero = {}
    for mid, values in samples.items():
        if len(values) < 10 or max(values) - min(values) > 0.5:
            raise ValueError(f"轴 {mid} 未保持静止或反馈不足，请保持参考姿态重试")
        zero[mid] = sum(values) / len(values)
    record = {"hand": "right", "created": datetime.now(timezone.utc).isoformat(),
              "axes": mapping_identity(axes), "zero_deg": zero}
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as out:
        json.dump(record, out, indent=2, allow_nan=False)
        out.write("\n")
    try:
        if accept is not None:
            accept(zero)
        os.replace(out.name, path)
    finally:
        Path(out.name).unlink(missing_ok=True)
    return zero


def request_panel_zero(port, path, axes):
    with TemporaryDirectory(dir=runtime_dir()) as directory, socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
        client.bind(str(Path(directory) / "reply"))
        client.settimeout(4)
        request = {"path": str(path.resolve()), "axes": mapping_identity(axes), "time": time.monotonic()}
        client.sendto(json.dumps(request).encode(), str(zero_socket(port)))
        error = json.loads(client.recv(8192))
        if error:
            raise ValueError(error)


def request_web_zero(http_port, device, path, axes):
    endpoint = f"http://127.0.0.1:{http_port}/api/v1/control"
    with urlopen(endpoint, timeout=1.) as response:
        before = json.load(response)
    request = Request(endpoint, json.dumps({
        "action": "zero", "device": str(device.resolve()),
        "zero_path": str(path.resolve()), "axes": mapping_identity(axes),
    }).encode(), {"Content-Type": "application/json"})
    with urlopen(request, timeout=1.):
        pass
    end = time.monotonic() + 5.
    while time.monotonic() < end:
        with urlopen(endpoint, timeout=1.) as response:
            status = json.load(response)
        if status.get("zero_revision", 0) != before.get("zero_revision", 0):
            if status.get("zero_error"):
                raise ValueError(status["zero_error"])
            return
        time.sleep(.055)
    raise TimeoutError("等待网页标零完成超时")


def run_control(read, axes, solver, mapping, pending, displayed, renderer, zero_path,
                wrist_state, feedback, proxy_frame, force_frame, fixed_wrist, feedback_display, control, device,
                *, demo=False, recorder=None, use_cpp=False):
    """单一 G4 所有者，维护采集、控制流与网页快照。"""
    zero = dict.fromkeys(axes, 0.) if demo else load_zero(zero_path, axes)
    cached_q, cached_pose = None, None
    zero_path = zero_path.resolve()
    status = {"device": "演示" if demo else str(device.resolve()), "zero_path": None if demo else str(zero_path),
              "axes": mapping_identity(axes), "zero_revision": 0,
              "zero_error": None, "can_align": control.align is not None, "fixed_wrist": fixed_wrist,
              "demo": demo, "can_zero": not demo,
              "can_feedback": feedback is not None and not demo,
              "feedback_profile": feedback.profile if feedback is not None else None,
              "feedback_source": "joint_position_and_contact_force" if feedback else None}
    if control.grasp is not None:
        status["grasp_backend"] = "cpp" if use_cpp else "python"
    ttl = int(CONFIG["command_ttl_s"] * 1e9)
    period = 1. / CONFIG["control_hz"]
    next_publish = 0.
    was_requested = False
    requires_new_zero = False
    model_limits = {mid: motor.model_rad_limits() for mid, (_, motor) in axes.items()}

    def poll_transport():
        nonlocal zero, cached_q, requires_new_zero, was_requested, next_publish
        if demo or not hasattr(read, "poll_transport"):
            return "none"
        event = read.poll_transport()
        if event != "none":
            requires_new_zero = True
            zero = None
            cached_q = None
            pending[0] = 0.
            control.requested.clear()
            was_requested = False
            if feedback is not None:
                feedback.stop("串口连接变化 · 等待重新标零")
            next_publish = 0.
        return event

    while renderer.is_alive():
        began = time.monotonic()
        poll_transport()
        if control.zero.is_set():
            control.requested.clear()
            pending[0] = 0.
            if feedback is not None:
                feedback.stop("标零中 · 真机出力关闭")
            status.update(calibrating=True, requested=False, active=False, feedback_message="标零中 · 真机出力关闭")
            control.publish(status)
            try:
                if poll_transport() != "none" or not getattr(read, "online", True):
                    raise ValueError("串口离线或已重新连接，请重新标零")
                def accept_zero(_):
                    if poll_transport() != "none" or not getattr(read, "online", True):
                        raise ValueError("串口离线或已重新连接，请重新标零")
                zero = capture_zero(read, axes, zero_path, accept=accept_zero)
                requires_new_zero = False
                cached_q = None
                status["zero_error"] = None
            except (ValueError, OSError) as exc:
                status["zero_error"] = str(exc)
            status.update(calibrating=False, zero_revision=status["zero_revision"] + 1)
            control.zero.clear()
            next_publish = 0.

        samples = read()
        now = time.monotonic_ns()
        targets, encoder_positions, raw_angles = [], [], {}
        sample_stamps = []
        rows = []
        applied, *measured = displayed[:]
        alarm = False
        for i, (mid, (name, motor)) in enumerate(axes.items()):
            joint = samples.get(mid)
            raw, age, state = position_sample(joint, mid, now)
            alarm |= state.startswith("报警") or state == "路由错误"
            target = encoder_position = None
            if state == "在线" and zero is not None and not requires_new_zero:
                low, high = model_limits[mid]
                encoder_position = math.radians((raw - zero[mid]) * motor.sign * motor.scale)
                target = min(high, max(low, encoder_position))
                targets.append(target)
                encoder_positions.append(encoder_position)
                raw_angles[mid] = raw
                sample_stamps.append(joint.received_ns)
            rows.append({
                "id": mid, "joint": name, "raw_deg": raw if raw is not None and math.isfinite(raw) else None,
                "input_rad": target, "sim_rad": measured[i] if applied and math.isfinite(measured[i]) else None, "age_ms": age,
                "state": state, "encoder_rad": encoder_position,
            })
        requested = control.requested.is_set() and not requires_new_zero
        if feedback is not None:
            if not requested and was_requested:
                feedback.stop()
            elif requested and not was_requested:
                feedback.reset()
        was_requested = requested
        pose_error = None
        stamp = 0
        if len(targets) == 12:
            q = np.array(targets)
            try:
                if not np.array_equal(q, cached_q):
                    cached_pose = solver.full_pose(q, None, mapping)
                    cached_q = q.copy()
                stamp = min(sample_stamps)
                pending[:] = [stamp, *cached_pose, *encoder_positions]
            except (ValueError, KinematicsError) as exc:
                pose_error = str(exc)
                stamp = 0
        grasp_settings = control.grasp_request()
        effort_frame = force_frame[:] if force_frame is not None else None
        frame_matches_grasp = grasp_settings is None or feedback.match_grasp_frame(
            effort_frame, grasp_settings["contact_revision"])
        if effort_frame is not None and not frame_matches_grasp:
            effort_frame[2] = 0.
        if stamp and feedback is not None:
            # 停止命令可能在求解闭链时到达，提交前重新读取。
            requested = control.requested.is_set() and not control.zero.is_set() and not requires_new_zero
            try:
                feedback.follow_request(q, raw_angles, stamp, effort_frame, requested,
                                        sample_stamps=sample_stamps, encoder_position=encoder_positions)
            except (ValueError, KinematicsError) as exc:
                pose_error = str(exc)
                stamp = 0
        if not stamp:
            pending[0] = 0.
            if feedback is not None:
                if feedback.active:
                    missing = "、".join(f"J{row['id']} {row['state']}" for row in rows if row["state"] != "在线")
                    feedback.interrupt(f"编码器采集 · {missing}" if missing else "闭链姿态不可用")
                feedback.ready_since = None
                feedback.nominal = None
                if not feedback.output_error and not feedback.rearm_required:
                    feedback.message = ("暂停提交力反馈 · 电机报警或路由不符" if alarm else
                                        "已请求启用 · 等待有效手套数据" if requested else feedback.message)

        now = time.monotonic_ns()
        currents = []
        for mid in axes:
            joint = samples.get(mid)
            valid = joint is not None and joint.valid & 2 and 0 <= now - joint.current_received_ns <= ttl
            currents.append(joint.current_a if valid and math.isfinite(joint.current_a) else math.nan)
        nominal = feedback.nominal if feedback is not None else None
        active = bool(feedback and feedback.active)
        if feedback_display is not None:
            feedback_display[:] = [now, active, *(nominal if nominal is not None else [math.nan] * 12), *currents]
        publish = began >= next_publish
        recording = recorder is not None and (recorder.active or control.record_requested.is_set())
        if publish or recording:
            sent = feedback.sent_nm if active else None
            fresh_contact = (effort_frame is not None and effort_frame[2]
                             and 0 <= now - min(effort_frame[:2]) < ttl)
            sim_positions = effort_frame[FORCE_VELOCITY_END:FORCE_POSITION_END] if fresh_contact else measured
            if grasp_settings is not None:
                applied = effort_frame[0] if fresh_contact else 0
            for i, row in enumerate(rows):
                if fresh_contact:
                    row["sim_rad"] = sim_positions[i]
                elif grasp_settings is not None:
                    row["sim_rad"] = None
                row["contact_motor_nm"] = effort_frame[3 + i] if fresh_contact else None
                row["contact_input_rad"] = effort_frame[FORCE_POSITION_END + i] if fresh_contact else None
                contact_encoder = effort_frame[FORCE_INPUT_END + i] if fresh_contact else math.nan
                row["contact_encoder_rad"] = contact_encoder if math.isfinite(contact_encoder) else None
                row["motor_rad_s"] = float(feedback.velocity.dq[i]) if stamp and nominal is not None else None
                row["position_motor_nm"] = float(feedback.position_torque[i]) if fresh_contact and nominal is not None else None
                # 映射接触项为滤波及回撤加权前的值，便于追踪建力过程。
                row["mapped_contact_nm"] = float(feedback.mapped_contact[i]) if fresh_contact and nominal is not None else None
                row["contact_gate"] = bool(feedback.contact_gate[i]) if fresh_contact and nominal is not None else None
                row["retreat_weight"] = float(feedback.frame_weight[i]) if fresh_contact and nominal is not None else None
                row["nominal_nm"] = float(nominal[i]) if nominal is not None and math.isfinite(nominal[i]) else None
                row["sent_motor_nm"] = sent[i] if sent is not None else None
                row["current_a"] = currents[i] if math.isfinite(currents[i]) else None
            contact_input = min(effort_frame[:2]) if effort_frame is not None and frame_matches_grasp else 0
            has_timing = bool(contact_input and 0 < effort_frame[0] <= effort_frame[1] <= now)
            wrist = proxy_frame[WRIST_STAMP_INDEX] if proxy_frame is not None else 0
            online = sum(r["state"] == "在线" for r in rows)
            transport_online = True if demo else getattr(read, "online", True)
            reconnect_count = 0 if demo else getattr(read, "reconnect_count", 0)
            message = feedback.message if feedback is not None else "仅位置采集"
            if not transport_online:
                message = "串口已断开 · 等待重新连接"
            elif not online and not alarm:
                message = "串口已打开 · 等待电机位置回包"
            elif requires_new_zero:
                message = "网关已重连 · 请重新标零"
            status.update(rows=rows, online=online,
                          calibrated=zero is not None and not requires_new_zero, requested=requested, active=active,
                          can_feedback=feedback is not None and not demo and transport_online and zero is not None and not requires_new_zero,
                          can_zero=not demo and transport_online,
                          transport_online=transport_online, reconnect_count=reconnect_count,
                          requires_new_zero=requires_new_zero,
                          output_error=bool(feedback and feedback.output_error),
                          feedback_rearm_required=bool(feedback and feedback.rearm_required),
                          feedback_interruptions=feedback.interruption_count if feedback is not None else 0,
                          last_feedback_interruption=feedback.last_interruption if feedback is not None else None,
                          feedback_handoffs=feedback.handoff_count if feedback is not None else 0,
                          last_feedback_handoff_ms=feedback.last_handoff_gap_ms if feedback is not None else None,
                          demo_grip=control.demo_grip.value if demo else None,
                          feedback_gain=1. if feedback is not None else None,
                          pose_error=pose_error,
                          feedback_message=message,
                          wrist=wrist_state.value.decode() if wrist_state is not None else "本地调试",
                          wrist_ms=(now-wrist)/1e6 if wrist else None,
                          contact_ms=(now-contact_input)/1e6 if contact_input else None,
                          contact_frame_ns=int(effort_frame[1]) if fresh_contact else None,
                          physics_input_ns=int(effort_frame[0]) if has_timing else None,
                          physics_solved_ns=int(effort_frame[1]) if has_timing else None,
                          command_deadline_ns=feedback.deadline if active else None,
                          motor_ms=(now-stamp)/1e6 if stamp else None,
                          applied_ms=(now-applied)/1e6 if applied else None,
                          max_angle_error_rad=measure_frame_gaps(rows))
            if recording:
                recorder.tick(now, rows, grasp_settings, active, demo, status)
            if publish:
                if recorder is not None:
                    ready = control.record_ready.value
                    status.update(recording=control.record_requested.is_set(),
                                  recording_file=Path(os.fsdecode(ready)).name if ready else None,
                                  recording_dropped=control.record_dropped.value,
                                  recording_error=control.record_error.value.decode())
                control.publish(status)
                next_publish = began + .05
        time.sleep(max(0., period - (time.monotonic() - began)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, help="已连接右手 G4 的串口")
    startup = parser.add_mutually_exclusive_group()
    startup.add_argument("--feedback", action="store_true", help="默认请求启用真机力反馈")
    startup.add_argument("--preview", action="store_true", help="启动时仅预览，通过网页可启用反馈")
    parser.add_argument("--mapping", type=Path, help="右手 12 轴标定文件")
    parser.add_argument("--zero", action="store_true", help="将当前静止参考姿态存为零位并退出")
    parser.add_argument("--zero-file", type=Path, default=Path.home() / ".config/superdex/ve33-g4-right-zero.json")
    parser.add_argument("--debugger", action="store_true", help="使用 Mochi 调试器查看物理网格")
    parser.add_argument("--quest", action="store_true", help="启用桌面魔方环境、Quest 右腕输入与场景输出")
    parser.add_argument("--quest-port", type=int, default=8765, help="Quest USB 反向转发的本机端口")
    parser.add_argument("--cube-dir", type=Path, default=ROOT / "assets/prefabs/puzzle_cube_quest")
    parser.add_argument("--headset", action="store_true", help="启用 Web 仿真和 Quest 腕部定位（默认）")
    parser.add_argument("--tabletop", action="store_true", help="使用桌面和单个刚性方块进行接触测试")
    parser.add_argument("--palm-test", action="store_true", help="本地固定腕部与 45 mm 掌心硬球测试")
    parser.add_argument("--local-grasp", action="store_true", help="本地固定腕部掌心与桌面抓握")
    parser.add_argument("--grasp-backend", choices=("cpp", "python"), help="遥操力反馈实时后端（默认 cpp）")
    parser.add_argument("--feedback-profile", choices=("identity", *CONFIG.get("feedback_profiles", {})), default="identity",
                        help="接触力矩映射配置（默认 identity）")
    parser.add_argument("--demo", action="store_true", help="本地抓握的无串口演示")
    parser.add_argument("--official-puzzle", action="store_true", help="用官方 grip_lift 测试固定腕部与掌心魔方")
    parser.add_argument("--inspect-physics", action="store_true", help="同时打开官方 Physics Debugger")
    parser.add_argument("--ball-size-mm", type=float, default=45.,
                        help="--palm-test 的球直径，范围 25–200 mm")
    parser.add_argument("--wrist-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="手柄相对手套 base_link 原点的刚体偏移 (米，FLU：+X指尖 +Y拇指 +Z手背)")
    args = parser.parse_args()
    from ve33_quest import parse_controller_offset, parse_controller_to_hand
    try:
        wrist_offset = tuple(parse_controller_offset(
            args.wrist_offset if args.wrist_offset is not None else CONFIG.get("wrist_tracker_offset_m", (0., 0., 0.))
        ).tolist())
        wrist_pose = None if args.wrist_offset is not None else CONFIG.get("wrist_tracker_pose")
        if wrist_pose is not None:
            parse_controller_to_hand(wrist_pose)
    except ValueError as exc:
        parser.error(f"--wrist-offset 无效：{exc}")
    if args.demo and (not (args.local_grasp or args.tabletop) or args.port or args.zero or args.feedback):
        parser.error("--demo 只用于 --local-grasp 或 --tabletop 的无串口预览")
    if not args.demo and args.port is None:
        parser.error("真机运行需要 --port")
    if not 25 <= args.ball_size_mm <= 200 or (args.ball_size_mm != 45. and not args.palm_test):
        parser.error("--ball-size-mm 只用于 --palm-test，直径应在 25–200 mm 内")
    fixed_wrist = args.palm_test or args.official_puzzle or args.local_grasp
    if fixed_wrist and (args.headset or args.tabletop or args.debugger or sum((args.palm_test, args.official_puzzle, args.local_grasp)) > 1):
        parser.error("本地固定腕部场景只能选择一项")
    if args.inspect_physics and not args.official_puzzle:
        parser.error("--inspect-physics 用于 --official-puzzle")
    if args.headset or args.tabletop or fixed_wrist or not (args.debugger or args.zero):
        args.quest = True
    if args.grasp_backend and not args.quest:
        parser.error("--grasp-backend 仅用于 Quest 或本地抓握模式")
    if not args.zero and not (ROOT / "tools/quest_view/build/scene.js").is_file():
        parser.error("请先在 tools/quest_view 下执行 npm ci && npm run build")
    if (args.quest and args.debugger) or not 1 <= args.quest_port <= 65535:
        parser.error("Quest 模式需要材质视图和有效 TCP 端口")
    if args.quest and not (args.tabletop or args.palm_test or args.local_grasp) and not (args.cube_dir / "render/puzzle_cube_corner1_shell.glb").is_file():
        parser.error("请用 --cube-dir 指定官方 main 分支的二阶魔方目录")
    if args.official_puzzle and not (args.cube_dir / "tuning/grip_lift.mochi_scene").is_file():
        parser.error("官方场景缺少 tuning/grip_lift.mochi_scene")
    if args.feedback and (not args.quest or args.zero):
        parser.error("--feedback 需要 --quest 或 --headset 场景")
    use_cpp = args.quest and args.grasp_backend != "python" and not args.zero
    if use_cpp:
        from motorhand.native_runtime import load_native
        try:
            load_native()
        except (ImportError, RuntimeError) as exc:
            parser.error(f"C++ 抓握后端启动失败：{exc}")
    mapping = GloveMapping.load(args.mapping or MAPPING)
    if mapping.raw.get("hand", "right") != "right" or not mapping.raw.get("calibrated"):
        parser.error("需要已标定的右手轴序、方向和比例")
    semantics = {x["motor"]: x["name"].removesuffix("_joint") for x in
                 json.loads((MODEL_DIR / "config/joint_space.json").read_text())["independent"]}
    for m in mapping.motors:
        if (semantics[m.model_joint] != m.joint or m.sign not in (-1., 1.) or m.scale <= 0 or
                not np.isfinite([m.scale, m.zero_deg, m.min_deg, m.max_deg]).all() or m.min_deg >= m.max_deg):
            parser.error(f"轴 {m.motor_id} 的标定与模型不一致")
    axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
    if use_cpp:
        from motorhand.native_kinematics import NativeGloveKinematics
        solver = NativeGloveKinematics(MODEL_DIR, mapping)
    else:
        solver = GloveKinematics(MODEL_DIR, mapping)
    if args.zero:
        try:
            with gateway(args.port) as read:
                capture_zero(read, axes, args.zero_file)
        except GatewayBusy:
            try:
                try:
                    request_web_zero(args.quest_port, args.port, args.zero_file, axes)
                except HTTPError as exc:
                    if exc.code != 404:
                        raise
                    request_panel_zero(args.port, args.zero_file, axes)
            except (OSError, ValueError) as exc:
                parser.exit(1, f"网页标零失败：{exc}\n")
        except (OSError, ValueError) as exc:
            parser.exit(1, f"标零失败：{exc}\n")
        print(f"已保存 12 轴零偏：{args.zero_file}")
        return
    ctx = mp.get_context("spawn")
    pending = None if use_cpp else ctx.Array("d", 1 + len(solver.joint_names) + MOTOR_COUNT)
    displayed = None if use_cpp else ctx.Array("d", 13)
    stop = ctx.Event()
    switchable_tasks = ("local_grasp", "tabletop") if (use_cpp and (args.local_grasp or args.tabletop)) else ()
    initial_task = "local_grasp" if args.local_grasp else "tabletop" if args.tabletop else ""
    align = ctx.Event() if args.quest and (not fixed_wrist or bool(switchable_tasks)) else None
    wrist_state = ctx.Array("c", 128) if args.quest else None
    legacy_feedback = args.quest and not use_cpp
    proxy_frame = ctx.Array("d", PROXY_FRAME_SIZE) if legacy_feedback else None
    force_frame = ctx.Array("d", FORCE_FRAME_SIZE) if legacy_feedback else None
    feedback_display = ctx.Array("d", 26) if legacy_feedback else None
    default_enabled = args.quest and not args.preview and (not args.local_grasp or args.feedback)
    web_control = WebControl(ctx, align, default_enabled=default_enabled, allow_reset=args.quest,
                             cube_switch=args.quest and not (fixed_wrist or args.tabletop),
                             local_grasp=args.local_grasp, demo=args.demo,
                             switchable_tasks=switchable_tasks, initial_task=initial_task)
    with ExitStack() as resources:
        shared_path = None
        loop = feedback = read = None
        if use_cpp:
            from motorhand.native_control import make_feedback_control
            from motorhand.native_observer import run_native_control
            from ve33_local_grasp import DemoGlove
            folder = resources.enter_context(TemporaryDirectory(prefix="ve33-link-"))
            shared_path = str(Path(folder) / "control.map")
            native = load_native()
            link = native.SharedLink(shared_path, len(solver.joint_names))
            web_control.native_link_path = shared_path
            link.command(bool(default_enabled and not args.demo), 0, -1.)
            if not args.demo:
                resources.enter_context(gateway_lock(args.port))
            motors = [motor for _, motor in axes.values()]
            lo, hi = zip(*(motor.model_rad_limits() for motor in motors))
            disabled_mask = 0
            for mid in CONFIG.get("disabled_motor_ids", ()):
                if isinstance(mid, int) and 1 <= mid <= MOTOR_COUNT:
                    disabled_mask |= 1 << (mid - 1)
            loop = native.NativeLoop(solver.native, make_feedback_control(axes, native, args.feedback_profile),
                                     link, "" if args.demo else str(args.port),
                                     [m.sign * m.scale for m in motors], lo, hi, DemoGlove.GRIP,
                                     round(1e9 / CONFIG["control_hz"]), disabled_mask)
            resources.callback(loop.close)
        else:
            if args.demo:
                from ve33_local_grasp import DemoGlove
                read = DemoGlove(axes, web_control.demo_grip)
            else:
                read = resources.enter_context(gateway(args.port, args.quest))
            feedback = ContactForceFeedbackLoop(read, axes, profile=args.feedback_profile) if args.quest else None

        def load_live_wrist_calibration():
            if args.wrist_offset is not None:
                return wrist_offset, None
            try:
                cfg_data = json.loads((ROOT / "hardware/encos_g4_gateway_sdk/config/right_sim_feedback.json").read_text())
                off = tuple(parse_controller_offset(cfg_data.get("wrist_tracker_offset_m", (0., 0., 0.))).tolist())
                pose_val = cfg_data.get("wrist_tracker_pose")
                if pose_val is not None:
                    parse_controller_to_hand(pose_val)
                return off, pose_val
            except (OSError, ValueError):
                return wrist_offset, wrist_pose

        def spawn_renderer(task_name):
            is_tabletop = task_name == "tabletop" if switchable_tasks else args.tabletop
            live_offset, live_pose = load_live_wrist_calibration()
            proc = ctx.Process(target=render, args=(solver.joint_names, [n for n, _ in axes.values()],
                                                    pending, displayed, stop, args.debugger, args.quest_port if args.quest else None,
                                                    args.cube_dir.resolve(), align, wrist_state, args.quest, proxy_frame,
                                                    is_tabletop, args.palm_test,
                                                    args.official_puzzle, args.ball_size_mm, feedback_display, web_control,
                                                    args.quest_port, force_frame, mapping, use_cpp, shared_path,
                                                    live_offset, live_pose))
            proc.start()
            return proc

        renderer = spawn_renderer(initial_task)
        active_renderer = [renderer]

        def switch_task(old_renderer, new_task):
            stop.set()
            old_renderer.join(timeout=6.)
            if old_renderer.is_alive():
                old_renderer.terminate()
                old_renderer.join(timeout=2.)
            stop.clear()
            if wrist_state is not None:
                wrist_state.value = b""
            web_control.commit_task(new_task)
            new_renderer = spawn_renderer(new_task)
            active_renderer[0] = new_renderer
            return new_renderer, new_task == "local_grasp"

        inspector = None
        recorder = None
        try:
            if args.local_grasp or "local_grasp" in switchable_tasks:
                from ve33_grasp_recording import GraspRecorder
                recorder = GraspRecorder(web_control)
            if args.inspect_physics:
                inspector = subprocess.Popen([
                    sys.executable, "-Bu", str(ROOT / "tools/ve33_puzzle_inspector.py"),
                    "--port", str(args.quest_port), "--cube-dir", str(args.cube_dir),
                ])
            print(f"MotorHand 网页控制台 http://127.0.0.1:{args.quest_port}/", flush=True)
            if use_cpp:
                run_native_control(loop, web_control, axes, args.zero_file, wrist_state, renderer, recorder,
                                   args.port, args.demo, args.feedback_profile, load_zero, capture_zero,
                                   mapping_identity, fixed_wrist=fixed_wrist,
                                   on_switch_task=switch_task if switchable_tasks else None)
            else:
                run_control(read, axes, solver, mapping, pending, displayed, renderer, args.zero_file,
                            wrist_state, feedback, proxy_frame, force_frame, fixed_wrist, feedback_display, web_control,
                            args.port, demo=args.demo, recorder=recorder, use_cpp=use_cpp)
        finally:
            if feedback is not None:
                feedback.stop()
            stop.set()
            if recorder is not None:
                recorder.close()
            if inspector is not None and inspector.poll() is None:
                inspector.terminate()
            cur = active_renderer[0]
            cur.join(timeout=2)
            if cur.is_alive():
                cur.terminate()
                cur.join()


if __name__ == "__main__":
    main()
