"""为 VE33 创建魔方支撑桌或桌面方块测试场景。"""

import trimesh
import vtk


class Tabletop:
    def __init__(self, sdp, scene, view, *, puzzle_shells=None):
        self.groups = {}
        self.contact_actors = []
        self.transforms = []
        layer = "tabletop"
        if puzzle_shells is None:
            # 桌面上表面 z=-80 mm，70 mm 方块就放在桌上。
            objects = (
                ("table", (.58, .58, .03), (.47, 0., -.095), (.25, .32, .37)),
                ("test_box", (.07, .07, .07), (.23, -.015, -.045), (.84, .34, .08)),
            )
        else:
            layer = "puzzle_table"
            # 桌面上表面 z=-2 mm，魔方从原来位置解锁时只落下一点。
            objects = (("table", (.76, .70, .03), (.50, -.015, -.017), (.29, .34, .39)),)
        self.names = tuple(item[0] for item in objects)
        for name, extent, position, color in objects:
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
            self.contact_actors.append((name, actor))
            self.transforms.append(transform)

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
        if puzzle_shells is None:
            scene.enable_layer_contact_asymmetric(layer, "RightHand", False)
        else:
            for other in ("puzzle_cube_core", "RightHand", "LeftHand"):
                scene.enable_layer_contact_symmetric(layer, other, False)
            # 只给桌面与外壳接触增加摩擦和冲击阻尼，保留角块及手指现有参数。
            params = sdp.ContactPairParamsOverride(coulomb_friction_coefficient=.8,
                                                   normal_viscous_damping_coefficient=2.)
            table = self.contact_actors[0][1].get_handle()
            for _, shell in puzzle_shells:
                scene.set_contact_pair_params_override(table, shell.get_handle(), params)
