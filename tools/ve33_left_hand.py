"""用左手指尖、掌心或左手柄的定位驱动纯仿真支撑点。"""

import numpy as np


class LeftHandSupport:
    # 指尖比实际手指更粗会使小角块难以松开。
    TIP_RADIUS = .008
    PALM_RADIUS = .020
    MAX_SPEED = .65

    def __init__(self, sdp, scene, surfaces):
        self.sdp, self.surfaces = sdp, [actor for _, actor in surfaces]
        self.real = np.float64 if sdp.USE_DOUBLE_PRECISION else np.float32
        self.actors, self.positions = [], [None] * 6
        self.parked = np.array([0., 0., -3.])
        self.radii = [self.TIP_RADIUS] * 5 + [self.PALM_RADIUS]
        for i, radius in enumerate(self.radii):
            self.actors.append(scene.create_rigid_actor(
                name=f"xr_left_{i}", layer="LeftHand",
                shape=sdp.create_sphere_shape([0., 0., 0.], radius),
                world_from_local=self.transform(self.parked),
                is_static=True, has_gravity=False,
            ))
        scene.enable_layer_contact_symmetric("LeftHand", "LeftHand", False)
        scene.enable_layer_contact_symmetric("LeftHand", "RightHand", False)
        scene.enable_layer_contact_symmetric("LeftHand", "puzzle_cube_core", False)
        self.kind = None

    def transform(self, point):
        return self.sdp.TransformRT(self.sdp.Quaternion(0, 0, 0, 1), self.sdp.Real3(*point))

    def can_place(self, point, radius):
        for surface in self.surfaces:
            bounds = surface.get_aabb_world()
            if (np.any(point < np.asarray(bounds.min) - radius) or
                    np.any(point > np.asarray(bounds.max) + radius)):
                continue
            output = np.empty(1, dtype=self.real)
            surface.get_points_distance_to_surface(np.asarray([point], dtype=self.real), output)
            if output[0] < radius - .001:
                return False
        return True

    def update(self, tracking, dt):
        kind = tracking["kind"] if tracking else None
        goals = [None] * 6
        if kind is not None:
            points = np.asarray(tracking["points"])
            if kind == "hand":
                goals[:5] = list(points[3::3])
                goals[5] = (points[0] + points[1::3].mean(axis=0)) / 2
            else:
                goals[5] = points[0]
        for i, (actor, goal, previous) in enumerate(zip(self.actors, goals, self.positions)):
            reacquiring = previous is None or kind != self.kind
            if goal is None or (reacquiring and not self.can_place(goal, self.radii[i])):
                if previous is not None:
                    actor.set_root_transform(self.transform(self.parked))
                self.positions[i] = None
                continue
            if not reacquiring:
                # 按实际物理步长限制速度，避免手柄瞬移把角块打走。
                delta = goal - previous
                travel = max(np.linalg.norm(delta), self.MAX_SPEED * dt)
                goal = previous + delta * (self.MAX_SPEED * dt / travel)
            actor.set_root_transform(self.transform(goal))
            self.positions[i] = goal
        self.kind = kind
        return [p.tolist() if p is not None else None for p in self.positions]
