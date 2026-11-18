"""独立于头显的刚性掌心球，复用参考仿真的尺寸、硬度和摆放。"""

import vtk


class PalmBall:
    def __init__(self, sdp, scene, view, diameter_mm=45.):
        # transfer/sim 的 ball_hard，参考掌坐标，单位 m。
        if not 25 <= diameter_mm <= 200:
            raise ValueError("球直径须在参考仿真的 25–200 mm 范围内")
        radius = diameter_mm / 2000.
        # 靠掌一侧固定在 z=-7 mm，改变直径只向掌外增加体积。
        center = (.072, -.005, -.007 - radius)
        transform = sdp.TransformRT(sdp.Quaternion(0, 0, 0, 1), sdp.Real3(*center))
        actor = scene.create_rigid_actor(
            name="palm_ball", layer="palmball",
            shape=sdp.create_sphere_shape(sdp.Real3(0, 0, 0), radius),
            world_from_local=transform, is_static=True, has_gravity=False,
        )
        scene.enable_layer_contact_asymmetric("palmball", "RightHand", False)

        source = vtk.vtkSphereSource()
        source.SetRadius(radius)
        source.SetThetaResolution(48)
        source.SetPhiResolution(32)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        part = vtk.vtkActor()
        part.SetMapper(mapper)
        part.GetProperty().SetColor(.92, .38, .09)
        part.GetProperty().SetInterpolationToPBR()
        part.GetProperty().SetRoughness(.60)
        assembly = vtk.vtkAssembly()
        assembly.AddPart(part)
        assembly.SetPosition(*center)
        view.scene.AddActor(assembly)
        view.camera_position = (.18, -.25, -.35)

        self.names = ("palm_ball",)
        self.transforms = (transform,)
        self.groups = {"palm_ball": assembly}
        self.contact_actors = [("palm_ball", actor)]
        self.sphere_radii = {"palm_ball": radius}
