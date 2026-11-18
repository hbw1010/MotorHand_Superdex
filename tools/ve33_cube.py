"""加载官方二阶魔方，渲染与场景接口共享同一组物理位姿。"""

import json

import numpy as np
import trimesh
import vtk
from scipy.spatial.transform import Rotation
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray


class PuzzleCube:
    def __init__(self, sdp, scene, folder, view, *, official_grip=False, switchable=False):
        name = "cube"
        path = folder / "puzzle_cube.mochi_prefab"
        if official_grip:
            fixture_file = folder / "tuning" / "grip_lift.mochi_scene"
            fixture = sdp.prefab.load_from_file(str(fixture_file), str(folder))
            reference, = fixture.prefabs
            path = (fixture_file.parent / reference.path).resolve()
            name = reference.name
        data = json.loads(path.read_text())
        definition, = data["actors"]["articulated"]
        prefab = sdp.prefab.load_from_file(str(path), str(folder))
        cube, = prefab.actors.articulated
        if cube.joints[0].name != "center_joint":
            raise ValueError("魔方资产的根关节应为 center_joint")
        if not switchable:
            cube.joints[0].type = sdp.ArticulatedJointType.HARD
        translation = [0.23, -.015, 0]
        if official_grip:
            # 使用 grip_lift 的场景层，将官方 padL/padR 替换为外层加入的 MotorHand。
            # 展开预制体以便固定中心，仍保留官方角块、约束和接触过滤。
            fixture.prefabs = []
            fixture.actors = prefab.actors
            fixture.constraints = prefab.constraints
            fixture.controllers = prefab.controllers
            fixture.contact_filter = prefab.contact_filter
            fixture.contact_pair_params_overrides = prefab.contact_pair_params_overrides
            prefab = fixture
            translation = np.array([.072, -.005, -.055]) - np.asarray(cube.translation)
            view.camera_position = (.18, -.25, -.35)
        params = sdp.prefab.PrefabParams(name=name, translation=translation, apply_scene_settings=False)
        instance = sdp.prefab.add_to_scene(prefab, scene, params)
        self.actor, = instance.filter(sdp.ActorType.ARTICULATED)
        info = self.actor.get_articulated_shape_info()
        self.names = list(info.link_names)
        self.transforms = sdp.DynamicArrayTransformRT(len(self.names))
        self.velocities = sdp.DynamicArrayReal(self.actor.get_num_dofs())
        self.initial_pose = sdp.DynamicArrayReal(self.actor.get_num_dofs())
        self.actor.get_articulated_pose(self.initial_pose)
        self.mode = "fixed"
        self.switchable = switchable
        if switchable:
            self.sdp, self.scene = sdp, scene
            self.center = scene.get_actor(self.actor.get_nested_link_actors()[0])
            root_dof = info.dof_info[0]
            self.root_indices = (*range(root_dof.get_trans_offset(), root_dof.get_trans_offset() + 3),
                                 *range(root_dof.get_rot_offset(), root_dof.get_rot_offset() + 3))
            self.pins = ()
            self.mode = "free"
            self.set_fixed(True)
        self.contact_actors = [(name, scene.get_actor(handle)) for name, handle in
                               zip(self.names, self.actor.get_nested_link_actors()) if name.endswith("_shell_link")]
        links = {link["name"]: link for link in definition["links"]}
        self.groups = {}
        self.matrices = []
        for name in self.names:
            link = links[name]
            assembly = vtk.vtkAssembly()
            self.groups[name] = assembly
            matrix = vtk.vtkMatrix4x4()
            assembly.SetUserMatrix(matrix)
            self.matrices.append(matrix)
            model = trimesh.load_scene(folder / link["renderModel"])
            for node in model.graph.nodes_geometry:
                transform, key = model.graph[node]
                mesh = model.geometry[key]
                # glTF 为 Y 向上，SuperDex 资产为 Z 向上。
                points = trimesh.transform_points(mesh.vertices, transform)
                points = points[:, [0, 2, 1]] * [1, -1, 1] * link["renderModelScale"]
                vertices = vtk.vtkPoints()
                vertices.SetData(numpy_to_vtk(np.ascontiguousarray(points), deep=True))
                cells = vtk.vtkCellArray()
                cells.SetData(numpy_to_vtkIdTypeArray(np.arange(0, mesh.faces.size + 1, 3, dtype=np.int64), deep=True),
                              numpy_to_vtkIdTypeArray(np.ascontiguousarray(mesh.faces.ravel(), dtype=np.int64), deep=True))
                poly = vtk.vtkPolyData()
                poly.SetPoints(vertices)
                poly.SetPolys(cells)
                normals = vtk.vtkPolyDataNormals()
                normals.SetInputData(poly)
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(normals.GetOutputPort())
                part = vtk.vtkActor()
                part.SetMapper(mapper)
                part.GetProperty().SetColor(*(mesh.visual.material.baseColorFactor[:3] / 255.)**(1/2.2))
                part.GetProperty().SetInterpolationToPBR()
                part.GetProperty().SetRoughness(.45)
                assembly.AddPart(part)
            view.scene.AddActor(assembly)
        self.update()

    def set_fixed(self, fixed):
        mode = "fixed" if fixed else "free"
        if mode == self.mode:
            return
        if not self.switchable:
            raise ValueError("此魔方场景固定中心")
        if fixed:
            center = self.center.get_root_transform()
            handle = self.center.get_handle()
            rotation = Rotation.from_quat(list(center.rotation)).as_rotvec()
            position_pin = self.scene.create_rigid_pivot_position_constraint(
                actor=handle, local_position=[0., 0., 0.],
                target_position=center.translation, damping=200.,
            )
            try:
                rotation_pin = self.scene.create_rigid_pivot_rotation_constraint(
                    actor=handle, local_rotation=[0., 0., 0.],
                    target_rotation=rotation, damping=2.,
                )
            except self.sdp.Error:
                self.scene.destroy_constraint(position_pin)
                raise
            self.pins = (position_pin, rotation_pin)
            self.actor.get_articulated_joint_velocities(self.velocities)
            for index in self.root_indices:
                self.velocities[index] = 0.
            self.actor.set_articulated_joint_velocities(self.velocities)
        else:
            for pin in self.pins:
                self.scene.destroy_constraint(pin)
            self.pins = ()
        self.mode = mode

    def reset(self):
        if self.switchable:
            self.set_fixed(False)
        self.actor.set_articulated_pose_from_joints(self.initial_pose)
        for index in range(len(self.velocities)):
            self.velocities[index] = 0.
        self.actor.set_articulated_joint_velocities(self.velocities)
        if self.switchable:
            self.set_fixed(True)

    def update(self):
        self.actor.get_articulated_link_transforms(self.transforms)
        matrix = np.eye(4)
        for target, tf in zip(self.matrices, self.transforms):
            matrix[:3, :3] = Rotation.from_quat(np.asarray(tf.rotation)).as_matrix()
            matrix[:3, 3] = np.asarray(tf.translation)
            target.DeepCopy(matrix.ravel())


def link_poses(names, transforms):
    return {name: [*map(float, tf.translation), *map(float, tf.rotation)]
            for name, tf in zip(names, transforms)}
