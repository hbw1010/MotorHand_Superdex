"""用 SuperDex 刚体位姿显示 VE33 的独立 CAD 零件及材质。"""

import json
from pathlib import PureWindowsPath
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation
import vtk


def is_metal(component):
    part = PureWindowsPath(component["source_part"]).stem
    # CAD 中的“指节3-模型”为安装在指节上的圆柱电机组件。
    return ("connecting_rod" in component["link"] or "drive_link" in component["link"]
            or "连杆" in part or "动子" in part or "定子" in part or part == "指节3-模型")


class MaterialView:
    def __init__(self, directory, link_names, environment, *, fit_workspace=False):
        self.fit_workspace = fit_workspace
        self.camera_position = (0.35, -0.55, 1.0)
        self.scene = vtk.vtkRenderer()
        self.scene.SetBackground(0.32, 0.35, 0.38)
        tone = vtk.vtkToneMappingPass()
        tone.SetDelegatePass(vtk.vtkRenderStepsPass())
        tone.SetToneMappingType(vtk.vtkToneMappingPass.GenericFilmic)
        tone.SetGenericFilmicDefaultPresets()
        tone.SetExposure(0.85)
        self.scene.SetPass(tone)
        self.window = vtk.vtkRenderWindow()
        self.window.SetWindowName("VE33 · 右手实时仿真")
        screen_width, screen_height = self.window.GetScreenSize()
        height = min(round(screen_height * 0.80), round(screen_width * 0.65), 1500)
        self.window.SetSize(round(height * 4 / 3), height)
        self.window.AddRenderer(self.scene)
        self.controls = vtk.vtkRenderWindowInteractor()
        self.controls.SetRenderWindow(self.window)
        self.controls.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.closed = False
        self.controls.AddObserver("ExitEvent", self._close)

        hdr = vtk.vtkHDRReader()
        hdr.SetFileName(str(environment))
        lighting = vtk.vtkTexture()
        lighting.SetInputConnection(hdr.GetOutputPort())
        lighting.SetColorModeToDirectScalars()
        lighting.InterpolateOn()
        lighting.MipmapOn()
        self.scene.UseImageBasedLightingOn()
        self.scene.SetEnvironmentTexture(lighting)

        parts = json.loads((directory / "config/cad_components.json").read_text())
        by_mesh = {part["mesh"]: part for part in parts}
        groups = {}
        for link in ElementTree.parse(directory / "hand_tree_only.urdf").findall("link"):
            group = vtk.vtkAssembly()
            groups[link.attrib["name"]] = group
            self.scene.AddActor(group)
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                path = directory / mesh.attrib["filename"]
                reader = vtk.vtkSTLReader()
                reader.SetFileName(str(path))
                normals = vtk.vtkPolyDataNormals()
                normals.SetInputConnection(reader.GetOutputPort())
                normals.SetFeatureAngle(50)
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(normals.GetOutputPort())
                part = vtk.vtkActor()
                part.SetMapper(mapper)
                material = part.GetProperty()
                material.SetInterpolationToPBR()
                if is_metal(by_mesh[path.name]):
                    material.SetColor(0.83, 0.84, 0.86)
                    material.SetMetallic(0.97)
                    material.SetRoughness(0.34)
                else:
                    material.SetColor(0.018, 0.021, 0.025)
                    material.SetMetallic(0)
                    material.SetRoughness(0.80)
                transform = np.eye(4)
                origin = visual.find("origin")
                if origin is not None:
                    rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
                    transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                transform[:3, :3] *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                matrix = vtk.vtkMatrix4x4()
                matrix.DeepCopy(transform.ravel())
                part.SetUserMatrix(matrix)
                group.AddPart(part)
        self.groups = groups
        self.matrices = []
        for name in link_names:
            matrix = vtk.vtkMatrix4x4()
            groups[name].SetUserMatrix(matrix)
            self.matrices.append(matrix)
        self.initial = True

    def _close(self, *_):
        self.closed = True

    def update(self, transforms):
        xyz = np.array([np.asarray(p.translation) for p in transforms])
        rotations = Rotation.from_quat([np.asarray(p.rotation) for p in transforms]).as_matrix()
        matrix = np.eye(4)
        for node, position, rotation in zip(self.matrices, xyz, rotations):
            matrix[:3, :3] = rotation
            matrix[:3, 3] = position
            node.DeepCopy(matrix.ravel())
        if self.initial:
            camera = self.scene.GetActiveCamera()
            camera.SetPosition(*self.camera_position)
            camera.SetViewUp(1, 0, 0)
            camera.ParallelProjectionOn()
            self.scene.ResetCamera()
            if self.fit_workspace:
                self.scene.ResetCameraScreenSpace(0.82)
            else:
                camera.Zoom(1.28)
            self.controls.Initialize()
            self.initial = False
        self.scene.ResetCameraClippingRange()
        self.window.Render()
        self.controls.ProcessEvents()

    def close(self):
        self.window.Finalize()
