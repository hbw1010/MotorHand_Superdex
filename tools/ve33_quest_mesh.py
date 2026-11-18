"""从桌面显示所用的同一组零件导出 Quest 启动时下载的二进制 glTF。"""

import numpy as np
import trimesh
from vtk.util.numpy_support import vtk_to_numpy


def scene_glb(hand_groups, object_groups, *, collection_name="puzzle_cube", support_groups=None):
    scene = trimesh.Scene()
    collections = [("right_hand", hand_groups), (collection_name, object_groups)]
    if support_groups is not None:
        collections.append(("objects", support_groups))
    for collection, groups in collections:
        for link, assembly in groups.items():
            node = f"{collection}__{link}"
            scene.graph.update(frame_to=node, matrix=np.eye(4))
            for i, part in enumerate(assembly.GetParts()):
                mapper = part.GetMapper()
                mapper.Update()
                poly = mapper.GetInput()
                polys = poly.GetPolys()
                faces = vtk_to_numpy(polys.GetConnectivityArray()).reshape(-1, 3)
                mesh = trimesh.Trimesh(vertices=vtk_to_numpy(poly.GetPoints().GetData()),
                                       faces=faces, vertex_normals=vtk_to_numpy(poly.GetPointData().GetNormals()),
                                       process=False)
                prop = part.GetProperty()
                color = np.clip(np.round(np.array([*prop.GetColor(), 1.]) * 255), 0, 255).astype(np.uint8)
                material = trimesh.visual.material.PBRMaterial(baseColorFactor=color,
                    metallicFactor=prop.GetMetallic(), roughnessFactor=prop.GetRoughness())
                mesh.visual = trimesh.visual.TextureVisuals(material=material)
                matrix = part.GetMatrix()
                local = [[matrix.GetElement(r, c) for c in range(4)] for r in range(4)]
                scene.add_geometry(mesh, parent_node_name=node, node_name=f"{node}_{i}", transform=local)
    return scene.export(file_type="glb")
