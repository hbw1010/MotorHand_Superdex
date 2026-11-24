"""为 MotorHand 创建魔方支撑桌或 Quest 多物体动态桌面交互场景。"""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray


PREFAB_ROOT = Path(__file__).resolve().parents[1] / "assets/prefabs"
TABLE_TOP_Z = -0.080

# Quest 动态桌面场景中的 8 个可交互官方刚体资产（名称保留 test_box 兼容既有前端判定）。
# solid_hull=True 时用外包实心凸包烘焙深内部 SDF，避免薄壁中空纸杯被手指轻易穿透导致无力反馈。
INTERACTIVE_OBJECTS = (
    ("test_box", "红色方块", "box_and_blocks/collision/block.mochi.h5", "box_and_blocks/render/block_red.glb",
     3.0, (0.24, 0.00), 0.095, (0.84, 0.24, 0.18), 0.0, 0.65, False),
    ("block_blue", "蓝色方块", "box_and_blocks/collision/block.mochi.h5", "box_and_blocks/render/block_blue.glb",
     2.6, (0.22, 0.125), 0.080, (0.20, 0.45, 0.85), 0.0, 0.65, False),
    ("block_yellow", "黄色方块", "box_and_blocks/collision/block.mochi.h5", "box_and_blocks/render/block_yellow.glb",
     2.6, (0.22, -0.125), 0.080, (0.92, 0.76, 0.18), 0.0, 0.65, False),
    ("block_green", "绿色方块", "box_and_blocks/collision/block.mochi.h5", "box_and_blocks/render/block_green.glb",
     2.3, (0.33, -0.185), 0.065, (0.22, 0.68, 0.36), 0.0, 0.65, False),
    ("paper_cup", "纸杯", "paper_cups/collision/paper_cup.mochi.h5", "paper_cups/render/paper_cup.glb",
     0.80, (0.34, 0.135), 0.085, (0.93, 0.91, 0.87), 0.0, 0.82, True),
    ("sphere_ball", "刚性球", "sphere/collision/sphere.mochi.h5", "sphere/render/sphere.glb",
     2.7, (0.34, -0.075), 0.090, (0.80, 0.82, 0.87), 0.35, 0.35, False),
    ("peg_cylinder", "插销圆柱", "functional_dexterity_test/collision/peg.mochi.h5",
     "functional_dexterity_test/render/peg.glb",
     2.6, (0.38, 0.035), 0.075, (0.86, 0.52, 0.22), 0.0, 0.62, False),
    ("duck_lamp", "小黄鸭", "duck_lamp/duck_lamp.mochi.h5", None,
     1.25, (0.35, 0.265), 0.130, (0.98, 0.80, 0.14), 0.0, 0.55, False),
)

OBJECT_LABELS = {item[0]: item[1] for item in INTERACTIVE_OBJECTS}
OBJECT_LABELS["table"] = "桌面"


def _trimesh_to_vtk_actor(mesh: trimesh.Trimesh, color, metallic=0.0, roughness=0.72):
    poly = vtk.vtkPolyData()
    pts = vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.ascontiguousarray(mesh.vertices, dtype=np.float32), deep=True))
    poly.SetPoints(pts)

    faces = np.ascontiguousarray(mesh.faces, dtype=np.int64)
    cells_arr = np.empty((len(faces), 4), dtype=np.int64)
    cells_arr[:, 0] = 3
    cells_arr[:, 1:] = faces
    cells = vtk.vtkCellArray()
    cells.SetCells(len(faces), numpy_to_vtkIdTypeArray(cells_arr.ravel(), deep=True))
    poly.SetPolys(cells)

    normals = vtk.vtkPolyDataNormals()
    normals.SetInputData(poly)
    normals.SetFeatureAngle(50.0)
    normals.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(normals.GetOutputPort())
    part = vtk.vtkActor()
    part.SetMapper(mapper)
    prop = part.GetProperty()
    prop.SetColor(*color)
    prop.SetInterpolationToPBR()
    prop.SetMetallic(metallic)
    prop.SetRoughness(roughness)
    return part


# 官方 .glb 为 glTF Y-up，而 SuperDex/Mochi 碰撞体为 Z-up；绕 X 轴转 +90° 对齐 (x, -z, y)。
_GLTF_TO_MOCHI = np.array([
    [1.0, 0.0,  0.0, 0.0],
    [0.0, 0.0, -1.0, 0.0],
    [0.0, 1.0,  0.0, 0.0],
    [0.0, 0.0,  0.0, 1.0],
])


def _load_scaled_glb(path: Path, scale: float) -> trimesh.Trimesh:
    loaded = trimesh.load(path)
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.to_mesh()
    else:
        mesh = loaded.copy()
    mesh.apply_transform(_GLTF_TO_MOCHI)
    mesh.apply_scale(scale)
    return mesh


def _build_solid_sdf_shape(sdp, template_h5: Path, mesh: trimesh.Trimesh):
    model = sdp.model.load_from_file(str(template_h5))
    model.mesh = sdp.MeshData(
        nodes_per_element=3,
        coordinates=np.ascontiguousarray(mesh.vertices, dtype=np.float64).ravel(),
        connectivity=np.ascontiguousarray(mesh.faces, dtype=np.int32).ravel(),
    )
    model.sdf = None
    sdp.model.bake_sdf(model)
    return sdp.create_model_shape(model)


def _load_duck_lamp_mesh_and_shape(sdp, h5_path: Path, scale: float):
    tet_shape = sdp.load_shape_from_file(str(h5_path))
    surf = sdp.get_shape_surface_mesh(tet_shape)
    verts = np.asarray(surf.coordinates, dtype=np.float64).reshape(-1, 3) * scale
    faces = np.asarray(surf.connectivity, dtype=np.int32).reshape(-1, 3)
    # 绕 Z 轴转向用户方向（+150°），保持肚皮平放、头朝上（+Z）
    rot = Rotation.from_euler("z", 150.0, degrees=True)
    verts = rot.apply(verts)
    verts[:, 0] -= (verts[:, 0].min() + verts[:, 0].max()) * 0.5
    verts[:, 1] -= (verts[:, 1].min() + verts[:, 1].max()) * 0.5
    verts[:, 2] -= verts[:, 2].min()
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    template_h5 = PREFAB_ROOT / "paper_cups/collision/paper_cup.mochi.h5"
    shape = _build_solid_sdf_shape(sdp, template_h5, mesh)
    return mesh, shape


class Tabletop:
    def __init__(self, sdp, scene, view, *, puzzle_shells=None, grasp_table=False):
        self.groups = {}
        self.contact_actors = []
        self.dynamic_actors = []
        self.transforms = []
        self._initial_transforms = []
        self._vtk_matrices = {}
        self._dynamic_indices = []
        layer = "tabletop"
        interactive = puzzle_shells is None and not grasp_table
        if grasp_table:
            from ve33_local_grasp import DESK_CENTER, DESK_SIZE
            static_objects = (("table", DESK_SIZE, DESK_CENTER, (.25, .32, .37)),)
        elif interactive:
            # 宽敞交互桌面（72 x 72 cm，上表面 z = -80 mm），支撑 8 个动态交互物体。
            static_objects = (("table", (.72, .72, .03), (.36, 0., TABLE_TOP_Z - .015), (.25, .32, .37)),)
        else:
            layer = "puzzle_table"
            # 桌面上表面 z=-2 mm，魔方从原来位置解锁时只落下一点。
            static_objects = (("table", (.76, .70, .03), (.50, -.015, -.017), (.29, .34, .39)),)

        names = []
        for name, extent, position, color in static_objects:
            mesh = trimesh.creation.box(extents=extent)
            shape = sdp.create_model_shape(mesh=sdp.MeshData(
                nodes_per_element=3, coordinates=mesh.vertices.ravel(),
                connectivity=mesh.faces.ravel(),
            ))
            transform = sdp.TransformRT(sdp.Quaternion(0, 0, 0, 1), sdp.Real3(*position))
            actor = scene.create_rigid_actor(
                name=name, layer=layer, shape=shape, world_from_local=transform,
                collider_type=sdp.ColliderType.BOX, is_static=True, has_gravity=False,
            )
            names.append(name)
            self.contact_actors.append((name, actor))
            self.transforms.append(transform)
            self._initial_transforms.append(transform)

            source = vtk.vtkCubeSource()
            source.SetXLength(extent[0])
            source.SetYLength(extent[1])
            source.SetZLength(extent[2])
            triangles = vtk.vtkTriangleFilter()
            triangles.SetInputConnection(source.GetOutputPort())
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(triangles.GetOutputPort())
            part = vtk.vtkActor()
            part.SetMapper(mapper)
            part.GetProperty().SetColor(*color)
            part.GetProperty().SetInterpolationToPBR()
            part.GetProperty().SetRoughness(.78)

            group = vtk.vtkAssembly()
            group.AddPart(part)
            group.SetPosition(*position)
            self.groups[name] = group
            view.scene.AddActor(group)

        if interactive:
            obj_layer = "TabletopObj"
            table_handle = self.contact_actors[0][1].get_handle()
            table_params = sdp.ContactPairParamsOverride(
                coulomb_friction_coefficient=0.90,
                normal_viscous_damping_coefficient=2.5,
            )
            for name, _, shape_rel, glb_rel, scale, (x, y), mass, color, metallic, roughness, solid_hull in INTERACTIVE_OBJECTS:
                if glb_rel is None:
                    mesh, shape = _load_duck_lamp_mesh_and_shape(sdp, PREFAB_ROOT / shape_rel, scale)
                else:
                    mesh = _load_scaled_glb(PREFAB_ROOT / glb_rel, scale)
                    if solid_hull:
                        shape = _build_solid_sdf_shape(sdp, PREFAB_ROOT / shape_rel, mesh.convex_hull)
                    else:
                        shape = sdp.load_shape_from_file(str(PREFAB_ROOT / shape_rel), [scale, scale, scale])
                z_min = float(mesh.bounds[0, 2])
                position = (float(x), float(y), TABLE_TOP_Z - z_min + 0.0005)
                transform = sdp.TransformRT(sdp.Quaternion(0, 0, 0, 1), sdp.Real3(*position))
                actor = scene.create_rigid_actor(
                    name=name, layer=obj_layer, shape=shape, world_from_local=transform,
                    collider_type=sdp.ColliderType.SDF, is_static=False, has_gravity=True, mass=mass,
                )
                scene.set_contact_pair_params_override(table_handle, actor.get_handle(), table_params)

                idx = len(names)
                names.append(name)
                self.contact_actors.append((name, actor))
                self.dynamic_actors.append((name, actor))
                self.transforms.append(transform)
                self._initial_transforms.append(transform)
                self._dynamic_indices.append(idx)

                part = _trimesh_to_vtk_actor(mesh, color, metallic=metallic, roughness=roughness)
                group = vtk.vtkAssembly()
                group.AddPart(part)
                mat = vtk.vtkMatrix4x4()
                self._vtk_matrices[name] = mat
                group.SetUserMatrix(mat)
                self.groups[name] = group
                view.scene.AddActor(group)

            # 禁用手与桌面的双向碰撞（避免误碰桌面触发电机持续堵转过热），保留物体与桌面支撑碰撞。
            scene.enable_layer_contact_symmetric(layer, "RightHand", False)
            scene.enable_layer_contact_asymmetric(obj_layer, "RightHand", False)
            scene.enable_layer_contact_symmetric(layer, "LeftHand", False)
            self.update()
        elif puzzle_shells is None:
            scene.enable_layer_contact_symmetric(layer, "RightHand", False)
        else:
            for other in ("puzzle_cube_core", "RightHand", "LeftHand"):
                scene.enable_layer_contact_symmetric(layer, other, False)
            # 只给桌面与外壳接触增加摩擦和冲击阻尼，保留角块及手指现有参数。
            params = sdp.ContactPairParamsOverride(coulomb_friction_coefficient=.8,
                                                   normal_viscous_damping_coefficient=2.)
            table = self.contact_actors[0][1].get_handle()
            for _, shell in puzzle_shells:
                scene.set_contact_pair_params_override(table, shell.get_handle(), params)
        self.names = tuple(names)

    def update(self):
        if not self._dynamic_indices:
            return
        matrix = np.eye(4)
        for idx in self._dynamic_indices:
            name, actor = self.contact_actors[idx]
            tf = actor.get_root_transform()
            self.transforms[idx] = tf
            mat = self._vtk_matrices.get(name)
            if mat is not None:
                matrix[:3, :3] = Rotation.from_quat(np.asarray(tf.rotation)).as_matrix()
                matrix[:3, 3] = np.asarray(tf.translation)
                mat.DeepCopy(matrix.ravel())

    def reset(self):
        for idx in self._dynamic_indices:
            _, actor = self.contact_actors[idx]
            actor.set_root_transform(self._initial_transforms[idx])
            actor.set_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.update()
