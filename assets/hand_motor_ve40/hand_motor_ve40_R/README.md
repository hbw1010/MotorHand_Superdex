# vE.4.0 右手电机外骨骼 · 12 主动自由度

导出日期：2026-11-23。来源：`motorhand/vE.4.0/手部电机外骨骼.SLDASM`，使用保存的 **12自由度** 配置。

**本次零位为 CAD 保存于 2026-11-23T18:39:44.641465+08:00 的装配姿态。全部主动输入设为 0，即恢复这次保存的位置。** 同时从动关节也以该装配姿态为 0；不是上一版的零位，也不是硬件编码器零点。来源 SHA256 和时间见 `config/source.json`。

## 打开模型

- 本机交互预览：<http://localhost:8098/viewer/index.html>，只显示 12 个主动输入，支持拖动、转轴正向显示和自定义命名。
- 标准 URDF：`hand_tree_only.urdf`。开发时同时使用 `config/loop_closures.yaml` 和 `scripts/solve_closed_chain.py`，才能计算从动联动。
- URDF Studio：导入 `Studio导入包.zip`，选择其中的 `hand_studio_preview.xml`。请使用包含网格的整个包。
- MuJoCo：`hand_closed_chain.xml` 保存完整闭链约束，已验证加载和运动学一致性。

**普通 URDF 只表示关节树，不能单独表达本模型的非线性闭链。** 仅导入 `.urdf` 不会自动让四连杆随动。请使用配套求解器或带闭链约束的 MJCF。被动关节不能删除或改成固定关节。

## 12 个主动输入

| 部位 | 主动输入 | 传动关系 |
| --- | --- | --- |
| 食指、中指、无名指、小指 | 每指 MCP 电机、PIP/DIP 联动电机 | 每指 2 个输入；指节和连杆由闭链联动 |
| 拇指 | CMC 屈伸、CMC 侧摆、MCP、IP 电机 | 4 个输入；MCP/IP 包含连杆传动 |

四指侧摆及小指 CMC 保持当前装配位置，未引入 16 自由度配置的额外驱动。

- `*_motor_joint` 是主动输入。`*_connecting_rod_数字` 是连杆，`*_connecting_rod_joint_*` 与 `*_output_joint` 是被动关节。
- 12 个主动输入、56 个被动转轴、4 个固定安装关节，共 73 个刚体和 72 个树关节。
- 28 处闭合转轴以两点约束表示，共 56 条 `equality/connect`。零位约束雅可比秩为 56，68 个可动转轴对应 **12 个独立自由度**。
- 本版包含 97 个可见零件实例、38 种零件/配置网格。隐藏的 `PICO 4U L1-1`、`左手柄支架-1` 和配置中压缩的零件未导出。
- 同名零件按各自配置分别提取，拇指的电机转子与安装连杆归入对应刚体。电机在 CAD 中用于定位零位的锁定销/角度配合不会作为永久运动锁；真实转轴取电机轴承同轴配合。

## 坐标与正方向

沿用用户确认的 V1.4 J1–J21 坐标与动作正向约定，与上一版 vE.3.3 的约定一致。

- 长度 m，角度 rad；所有可动关节局部 `axis="0 0 1"`，正转遵循局部 +Z 右手定则。
- 基坐标仍以手背底板为基准。保留 vE.4.0 的实际尺寸、轴心、零位姿态，不将新版强行摆回旧版姿态。
- 局部 X 取上一版对应关节 X 在实际转轴法平面上的投影，Y 构成右手坐标系。
- 正的电机输入使对应输出指节沿上一版正向运动。由于传动比和连杆关系，电机角度不等于指节角度。
- `12电机定义.csv` 给出轴心/正轴；`与上一版关节对应.csv` 给出 J 编号；`config/coordinate_alignment.json` 保存坐标变换和相对本次未对齐中间模型的符号转换。
- 不可直接把上一版或实物编码器的数值当成本次零位下的输入角度。

## 文件结构

```text
hand_motor_ve40_R/
├── hand_tree_only.urdf          标准关节树
├── hand.srdf                   12 主动分组、56 被动标记
├── hand_closed_chain.xml       完整 MJCF 闭链模型
├── hand_studio_preview.xml     Studio 预览范围保护版
├── Studio导入包.zip            Studio 入口与全部网格
├── meshes/                    97 个 STL 实例，单位 m
├── config/
│   ├── source.json             CAD 来源、保存时间、零位与哈希
│   ├── joint_roles.json        主动 / 被动 / 固定分类
│   ├── loop_closures.yaml      闭合转轴、12 输入和56从动轴
│   ├── assembly_branches.json  防翻面的装配分支约束
│   ├── motor_definitions.json  主动转轴与方向
│   ├── coordinate_alignment.json 坐标与正向转换
│   └── cad_components.json     CAD 零件、配置和网格映射
├── scripts/                   闭链求解器、本地预览服务
├── viewer/                    网页及离线依赖
├── examples/control_active.py  仅操作主动输入的离线示例
├── verification/              本次模型的检查报告
├── CAD零件对照.csv
├── 12电机定义.csv
├── 关节分类.csv
├── 与上一版关节对应.csv
├── 启动预览.ps1
└── README.md
```

## 启动与接入

本机运行 `启动预览.ps1`，然后打开上面的 HTTP 地址。不要直接双击 `viewer/index.html`，`file://` 页面无法正常连接闭链接口。换电脑后安装 Python 和依赖，在包目录执行：

```powershell
python -m pip install -r requirements.txt
python scripts/viewer_server.py --port 8098
```

```python
from scripts.solve_closed_chain import ClosureSolver
hand = ClosureSolver('hand_tree_only.urdf', 'config/loop_closures.yaml')
all_positions = hand.solve({'index_mcp_motor_joint': 0.2})
active_positions = {name: all_positions[name] for name in hand.independent}
# 12个主动角度用于上层接口；显示完整机构时使用 all_positions。
```

未指定的主动关节保持原值；返回值包含 68 个转轴位置。被动、固定、未知名称及不合法角度会被拒绝。无法到达的闭链请求保留上一有效姿态；求解器用连续初值、细分步长和分支约束避免翻面。

`hand.srdf` 是语义文件，不替代闭链求解器，也不代表已经接好 MoveIt 闭链规划。普通查看器若不读取 SRDF/分类 JSON，仍可能列出被动关节；自定义控制面板应只读取 `joint_roles.json` 的 `active` 列表。

## 本次验证与范围

- 零位：97 个可见零件的 URDF 世界变换与本次 CAD 提取结果一致，矩阵最大误差 5.55e-15；所有源 CAD 文件哈希保持不变。
- 正方向：12 个主动输入及四指 DIP 相对 PIP 的动作正向与 V1.4 一致；96 个运动姿态在坐标变换前后保持同一几何位姿。
- 配套求解器：1,468 步单输入往返和 12 输入组合测试，无分支翻面；最大闭合残差 1.12e-10 m。测试主范围为 ±0.6 rad，小指 PIP/DIP 为 0～0.6 rad。
- 控制分类：12 个主动输入接受、56 个被动与4个固定及未知输入拒绝；失败命令不改变上一有效状态。
- MuJoCo 3.17.0：97 网格、68 转轴、56 闭链约束成功编译；零位、所有输入0.2/0.4 rad及回零时，与 URDF FK 一致。
- Studio 开源求解器（提交 `23b96e2f3d5b1b68ad9629925d4479e9847d35a5`）完成 624 步往返，无翻面、无未收敛、无输入修正；最大闭合残差 0.0096 mm。这是本地固定版本测试，未声称当前线上版本完整验收。
- **Studio 预览保护范围：小指 MCP 为 −0.45～+0.6 rad，小指 PIP/DIP 为 −0.15～+0.6 rad。** 其余输入测试为 ±0.6 rad。小指 PIP/DIP 继续负转时通用求解器会修正输入，因此单独限制预览范围。完整 URDF/MJCF 未增加这两项主动角度限制。
- 本地滑块 ±90° 仅是操作范围。上述数值是数值验证/预览范围，均不是已测得的实物机械限位；超出可达范围会停留在上一有效姿态。

本包用于几何与运动学核对。尚未配置实测惯量、碰撞、力矩、效率、硬件控制器或编码器标定；MJCF 使用统一密度产生预览惯量，关闭接触与重力。未完成动力学验收。

源码提取工具：jsk-ros-pkg/solidworks_urdf_exporter2（sw2robot）。第三方许可见 `THIRD_PARTY_LICENSE` 与 `viewer/vendor/`。
