# Soft Duck Lamp Asset

## Overview

This folder contains a compliant duck lamp asset that was created in-house by modeling a shape inspired by a [commercially available Benson Duck Light](https://www.amazon.com/dp/B08CVP316Y). No third-party digital source files were used; the geometry in this directory is Meta's own work.

The physical product is sold under various brands including UNEEDE, the example linked above. Its manufacturer was not identified in the available provenance materials.

## Reference Object

| Field | Value |
| --- | --- |
| Product name | LED Benson Night Light |
| Source / purchase link | [Amazon product B08CVP316Y](https://www.amazon.com/dp/B08CVP316Y) |
| Vendor | [Amazon.com](https://www.amazon.com/) |
| Brand | UNEEDE |
| Manufacturer | Not identified in the source materials |

## Derivations

The following work was done to produce this simulation-ready asset:

- Hand-modeled an approximation of the duck lamp geometry in SolidWorks; the geometry was not 3D-scanned.
- Exported STEP CAD interchange files from which surface triangle meshes were generated.
- Generated a tetrahedral mesh from the surface mesh geometry.
- Constructed a soft-body physics simulation and collision model from surface and tetrahedral geometry.

本项目的 `duck_lamp_hand_contact.mochi.h5` 派生自 Meta 的 `duck_lamp.mochi.h5`，沿用下述 CC BY 4.0 许可。为固定腕部抓握添加了由 338 个原始体网格节点驱动的 672 面接触蒙皮（保留三个坐标轴上的尺寸极值节点）；材料仍由原 4059 个四面体求解，画面采用原 1328 面表面。在仓库根目录运行 `.venv/bin/python tools/experiments/bake_duck_contact_skin.py` 可以重新生成此派生文件。在 108 mm 模型的静止姿态，以两套三角面的顶点和面心采样，原表面到接触蒙皮的距离 P95 为 0.79 mm、最大 1.68 mm；反向分别为 0.65 mm、1.20 mm。这里统计的是采样点距离。

## License & IP Notes

The Meta-authored digital assets in this directory are distributed under the [Creative Commons Attribution 4.0 International License (CC-BY-4.0)](./LICENSE). You must give appropriate credit to Meta Platforms, Inc. when you use or redistribute them.

The license applies only to Meta-authored CAD, render meshes, collision data, processing metadata, and prefab manifests in this directory. It does not grant rights in the underlying physical product design, trade dress, product names, logos, or trademarks.

These assets were modeled after a commercially available Benson Duck Light. While the 3D geometry is Meta's own work, the underlying product design may be protected by copyright, design-patent, or trade-dress rights. Accordingly:

- These assets were created for simulation, research, visualization, and software development. CC-BY-4.0 applies only to copyrightable material Meta owns and may license; it does not grant patent, design-patent, trademark, trade-dress, or other rights in the referenced physical product. Manufacturing physical hardware may require separate permission from the applicable rightsholders.
- Any manufacturer, vendor, brand, logo, or product name associated with the reference object may not be used to imply endorsement, certification, sponsorship, or an official partnership without prior written permission from the applicable rightsholder.

**You are responsible for ensuring your use is compatible with all applicable third-party rights.**
