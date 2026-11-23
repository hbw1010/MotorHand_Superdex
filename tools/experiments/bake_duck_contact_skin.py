"""用鸭灯原体网格的表面节点烘焙接触蒙皮。生成资产只供固定腕部软鸭灯使用。"""
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import superdex.physics as sdp
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy


def bake(source, target):
    model = sdp.model.load_from_file(str(source))
    nodes = np.asarray(model.mesh.coordinates).reshape(-1, 3)
    tetrahedra = np.asarray(model.mesh.connectivity).reshape(-1, 4)
    faces = tetrahedra[:, [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]].reshape(-1, 3)
    _, first, counts = np.unique(np.sort(faces, axis=1), axis=0, return_index=True, return_counts=True)
    boundary = faces[first[counts == 1]]
    vertices = nodes[boundary]
    if np.einsum('ij,ij->', vertices[:, 0], np.cross(vertices[:, 1], vertices[:, 2])) < 0:
        boundary = boundary[:, ::-1]
    ids, triangles = np.unique(boundary, return_inverse=True)
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(nodes[ids], deep=True))
    cells = vtk.vtkCellArray()
    cells.SetData(numpy_to_vtkIdTypeArray(np.arange(0, triangles.size + 1, 3, dtype=np.int64), deep=True),
                  numpy_to_vtkIdTypeArray(triangles.ravel().astype(np.int64), deep=True))
    surface = vtk.vtkPolyData()
    surface.SetPoints(points)
    surface.SetPolys(cells)
    simplify = vtk.vtkDecimatePro()
    simplify.SetInputData(surface)
    simplify.SetTargetReduction(.5)
    simplify.PreserveTopologyOn()
    simplify.SplittingOff()
    simplify.BoundaryVertexDeletionOff()
    simplify.Update()
    skin = simplify.GetOutput()
    coords = vtk_to_numpy(skin.GetPoints().GetData())
    conn = vtk_to_numpy(skin.GetPolys().GetConnectivityArray()).reshape(-1, 3)
    distances, source_ids = cKDTree(nodes).query(coords)
    if np.max(distances) > 1e-8:
        raise ValueError("接触顶点必须与体网格节点重合")
    # 将简化时丢失的尺寸极值放回所在三角面，每个点仍由原体网格的单个节点驱动。
    extremes = np.unique(np.concatenate((nodes.argmin(axis=0), nodes.argmax(axis=0))))
    for node in np.setdiff1d(extremes, source_ids):
        shell = trimesh.Trimesh(vertices=nodes[source_ids], faces=conn, process=False)
        point, _, idx = trimesh.proximity.closest_point_naive(shell, nodes[node:node+1])
        bary = trimesh.triangles.points_to_barycentric(shell.triangles[idx], point)[0]
        if bary.min() <= 1e-6:
            raise ValueError("尺寸极值投影落在边上，必须同时细分相邻的两个面")
        face = int(idx[0])
        a, b, c = conn[face]
        extra = len(source_ids)
        source_ids = np.append(source_ids, node)
        conn[face] = [a, b, extra]
        conn = np.vstack((conn, [b, c, extra], [c, a, extra]))
    model.contact_skin_mesh = sdp.MeshData(
        nodes_per_element=3, coordinates=nodes[source_ids].ravel(), connectivity=conn.ravel(),
        skinning=sdp.SkinningData(weights_per_node=1, indices=source_ids, weights=np.ones(len(source_ids))))
    sdp.model.save_to_file(model, str(target), sdp.FileFormat.H5)


if __name__ == '__main__':
    folder = Path(__file__).resolve().parents[2] / 'assets/prefabs/duck_lamp'
    bake(folder / 'duck_lamp.mochi.h5', folder / 'duck_lamp_hand_contact.mochi.h5')
