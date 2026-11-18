"""给闭链姿态和物理手腕的位移加上角块表面约束。"""

import numpy as np
from scipy.spatial.transform import Rotation
from trimesh.remesh import subdivide_to_size
from vtk.util.numpy_support import vtk_to_numpy


class CubeContactGuard:
    """在跟随角块转动的包围盒中扫描手部外表面。

    SuperDex 的软接触继续提供角块受力。这里用保守的包围盒补足稀疏
    采样与直接写入闭链姿态之间的间隙，限制一次运动能越过多少表面。
    """

    def __init__(self, sdp, actor, view, link_names, shells, *, solid_surfaces, sphere_radii=None):
        self.actor = actor
        self.previous_target_local = None
        self.real = np.float64 if sdp.USE_DOUBLE_PRECISION else np.float32
        self.transforms = sdp.DynamicArrayTransformRT(len(link_names))
        samples, indices = [], []
        for i, name in enumerate(link_names):
            if name in solid_surfaces:
                mesh = solid_surfaces[name].get_surface_mesh()
                local_points = np.asarray(mesh.coordinates).reshape(-1, 3)
                triangles = np.asarray(mesh.connectivity).reshape(-1, 3)
                # 凸包新增的封口面内部也要参与扫描，单靠原 CAD 顶点会漏过环口。
                points, faces = subdivide_to_size(local_points, triangles, max_edge=.008)
                vertices = np.concatenate((points, points[faces].mean(axis=1)))
            else:
                pieces = []
                for part in view.groups[name].GetParts():
                    mapper = part.GetMapper()
                    mapper.Update()
                    vertices = vtk_to_numpy(mapper.GetInput().GetPoints().GetData())
                    matrix = part.GetMatrix()
                    local = np.array([[matrix.GetElement(r, c) for c in range(4)] for r in range(3)])
                    pieces.append(vertices @ local[:, :3].T + local[:, 3])
                if not pieces:
                    continue
                vertices = np.concatenate(pieces)
            # 保留各方向最外侧顶点以覆盖指腹边缘。
            voxels = np.floor(vertices / .0015).astype(np.int32)
            _, selected = np.unique(voxels, axis=0, return_index=True)
            selected = np.unique(np.r_[selected, vertices.argmax(axis=0), vertices.argmin(axis=0)])
            samples.append(vertices[selected])
            indices.extend([i] * len(selected))
        self.local = np.concatenate(samples)
        self.indices = np.array(indices)
        self.fingers = np.array([name.split("_")[0] for name in link_names])
        self.shells = []
        self.sphere_radii = sphere_radii or {}
        for name, shell in shells:
            if name in self.sphere_radii:
                self.shells.append((name, shell, None, None))
                continue
            vertices = np.asarray(shell.get_surface_mesh().coordinates).reshape(-1, 3)
            # 留给求解器少量接触行程。其余由连续扫描限制。
            self.shells.append((name, shell, vertices.min(axis=0) + .0015, vertices.max(axis=0) - .0015))

    def points(self, *, target=False):
        if target:
            self.actor.get_articulated_target_link_transforms(self.transforms)
        else:
            self.actor.get_articulated_link_transforms(self.transforms)
        rotation = Rotation.from_quat([list(t.rotation) for t in self.transforms]).as_matrix()
        translation = np.array([list(t.translation) for t in self.transforms])
        return np.einsum("nij,nj->ni", rotation[self.indices], self.local) + translation[self.indices]

    def target_contacts(self, wrist, compare_motion=False):
        """比较同一腕部与角块位姿下，当前与上一组电机角的接触。"""
        points = self.points(target=True)
        pose = Rotation.from_quat(wrist[3:])
        previous = None
        if compare_motion and self.previous_target_local is not None:
            previous = pose.apply(self.previous_target_local) + wrist[:3]
        self.previous_target_local = pose.inv().apply(points - wrist[:3])

        hits, deepening = {}, set()
        for name, shell, _, _ in self.shells:
            aabb = shell.get_aabb_world()
            candidates = np.flatnonzero(((points >= np.asarray(aabb.min)) &
                                         (points <= np.asarray(aabb.max))).all(axis=1))
            if not candidates.size:
                continue
            query = np.ascontiguousarray(points[candidates], dtype=self.real)
            distances = np.empty(len(query), dtype=self.real)
            shell.get_points_distance_to_surface(query, distances)
            inside = distances < 0.
            indices = candidates[inside]
            if not indices.size:
                continue
            hits[name] = np.unique(self.indices[indices])
            if previous is not None:
                query = np.ascontiguousarray(previous[indices], dtype=self.real)
                earlier = np.empty(len(query), dtype=self.real)
                shell.get_points_distance_to_surface(query, earlier)
                deeper_indices = indices[earlier - distances[inside] > .000003]
                deepening.update(self.fingers[self.indices[deeper_indices]])
        return hits, deepening

    @staticmethod
    def sphere_sweep(start, end, center, radius):
        a = start - center
        delta = end - start
        speed2 = (delta * delta).sum(axis=1)
        dot = (a * delta).sum(axis=1)
        separation = (a * a).sum(axis=1) - radius**2
        discriminant = dot**2 - speed2 * separation
        root = np.sqrt(np.maximum(0., discriminant))
        denom = np.maximum(speed2, 1e-18)
        first, last = (-dot - root) / denom, (-dot + root) / denom
        enters = ((speed2 > 1e-18) & (discriminant > 0.)
                  & (first < 1.) & (last > np.maximum(0., first) + 1e-8))
        # 已经接触的表面点可径向撤离。
        valid = enters & ~((separation < 0.) & (dot >= 0.))
        return valid, np.where(valid, np.maximum(first, 0.), np.inf)

    def sweep(self, start, end):
        """返回最先撞上的时间比例、外法线，以及各角块挡到的手部链接索引。"""
        fraction, normal, hits = 1., None, {}
        segment_min, segment_max = np.minimum(start, end), np.maximum(start, end)
        for name, actor, low, high in self.shells:
            aabb = actor.get_aabb_world()
            possible = np.flatnonzero(((segment_max >= np.asarray(aabb.min)) &
                                       (segment_min <= np.asarray(aabb.max))).all(axis=1))
            if not possible.size:
                continue
            transform = actor.get_root_transform()
            if name in self.sphere_radii:
                center = np.asarray(transform.translation)
                valid, candidates = self.sphere_sweep(start[possible], end[possible], center,
                                                       self.sphere_radii[name] - .0007)
                if valid.any():
                    hits[name] = np.unique(self.indices[possible[valid]])
                    best = int(candidates.argmin())
                    if candidates[best] < fraction:
                        fraction = float(candidates[best])
                        index = possible[best]
                        radial = start[index] + fraction * (end[index] - start[index]) - center
                        normal = radial / np.linalg.norm(radial)
                continue
            axes = Rotation.from_quat(list(transform.rotation)).as_matrix()
            position = np.asarray(transform.translation)
            a = (start[possible] - position) @ axes
            b = (end[possible] - position) @ axes
            delta = b - a
            moving = np.abs(delta) > 1e-9
            d = np.where(moving, delta, 1.)
            t0, t1 = (low-a)/d, (high-a)/d
            enter = np.where(moving, np.minimum(t0, t1), -np.inf)
            leave = np.where(moving, np.maximum(t0, t1), np.inf)
            first = enter.max(axis=1)
            inside = ((a > low + 1e-8) & (a < high - 1e-8)).all(axis=1)
            # 如果角块在这一步转入上一帧的位置，仍允许沿最近面撤离。
            margins = np.column_stack((a-low, high-a))
            face = margins.argmin(axis=1)
            outward = np.where(face < 3, -1., 1.)
            retreating = np.take_along_axis(delta, (face % 3)[:, None], axis=1)[:, 0] * outward >= -1e-8
            valid = ((moving | ((a >= low) & (a <= high))).all(axis=1)
                     & (leave.min(axis=1) > np.maximum(0., first) + 1e-8)
                     & (first < 1.) & ~(inside & retreating))
            if not valid.any():
                continue
            hits[name] = np.unique(self.indices[possible[valid]])
            candidates = np.where(valid, np.maximum(0., first), np.inf)
            best = int(candidates.argmin())
            if candidates[best] < fraction:
                fraction = float(candidates[best])
                axis = face[best] % 3 if inside[best] else enter[best].argmax()
                sign = outward[best] if inside[best] else -np.sign(delta[best, axis])
                normal = axes[:, axis] * sign
        return fraction, normal, hits
