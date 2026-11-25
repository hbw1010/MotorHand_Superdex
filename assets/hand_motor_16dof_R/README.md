# 右手电机外骨骼 · 16 主动自由度

导出日期：2026-11-25。源总装：`Desktop/新建文件夹 (2)/手部电机外骨骼.SLDASM`，配置 **16自由度**。
使用当前 SolidWorks 总装及其实际引用的已保存子装配姿态作为零位，16 个输入均为 0。本次仅更新拇指零位，其他四指的零位、全部输入顺序及物理正方向保持不变。总装保存时间为 2026-11-25T11:40:56.647361+08:00；拇指子装配最新保存时间为 2026-11-25T11:40:56.162670+08:00。实际引用路径、各子文件保存时间和哈希见 `config/source.json`、`verification/source_audit.json`。这不是硬件编码器的标定零点。

## 打开方式

- 本机预览：<http://localhost:8099/viewer/index.html>。先运行 `启动预览.ps1`；只显示 16 个主动输入，连杆自动随动。
- **URDF Studio：导入 `Studio导入包.zip`，选择 `hand_studio_preview.xml`。**
- 标准 URDF：`hand_tree_only.urdf`，需配合 `config/loop_closures.yaml` 和 `scripts/solve_closed_chain.py` 计算从动关节。
- MuJoCo：`hand_closed_chain.xml`，含完整闭链约束，已验证加载和运动学一致性。

普通 URDF 只能表达关节树，单独打开 `.urdf` 不会自动求解四连杆。请连同求解器使用，或使用带闭链约束的 MJCF/Studio 导入包。被动关节保留在模型中，不可删除或改成固定。

## 输入顺序

前 12 项恢复上一版预览中用户看到的顺序：**1–4 拇指、5–6 食指、7–8 中指、9–10 无名指、11–12 小指**。新侧摆固定放在 13–16：**食指、中指、无名指、小指**。预览、CSV、JSON、SRDF 和控制接口使用同一顺序。

本次修订重新读取你保存的新零位。上一个 16 自由度包误把内部遍历顺序当作显示顺序，导致中指排在最前；此次已统一。关节字符串 ID 及对应动作保持不变。若曾按旧包数组下标控制，请按 `config/input_order_migration.json` 更新下标映射。

| 控制序号 | 名称 | 关节 ID |
| --- | --- | --- |
| 1 | 拇指 CMC 屈伸电机 | `thumb_cmc_flex_motor_joint` |
| 2 | 拇指 CMC 侧摆电机 | `thumb_cmc_abd_motor_joint` |
| 3 | 拇指 MCP 屈伸电机 | `thumb_mcp_motor_joint` |
| 4 | 拇指 IP 屈伸电机 | `thumb_ip_motor_joint` |
| 5 | 食指 MCP 屈伸电机 | `index_mcp_motor_joint` |
| 6 | 食指 PIP/DIP 联动电机 | `index_pip_dip_motor_joint` |
| 7 | 中指 MCP 屈伸电机 | `middle_mcp_motor_joint` |
| 8 | 中指 PIP/DIP 联动电机 | `middle_pip_dip_motor_joint` |
| 9 | 无名指 MCP 屈伸电机 | `ring_mcp_motor_joint` |
| 10 | 无名指 PIP/DIP 联动电机 | `ring_pip_dip_motor_joint` |
| 11 | 小指 MCP 屈伸电机 | `pinky_mcp_motor_joint` |
| 12 | 小指 PIP/DIP 联动电机 | `pinky_pip_dip_motor_joint` |
| 13 | 食指 侧摆电机 | `index_mcp_abd_motor_joint` |
| 14 | 中指 侧摆电机 | `middle_mcp_abd_motor_joint` |
| 15 | 无名指 侧摆电机 | `ring_mcp_abd_motor_joint` |
| 16 | 小指 侧摆电机 | `pinky_mcp_abd_motor_joint` |

这里的控制序号不是旧版数据手套的 J 编号；旧 J 编号对应关系在 `与上一版关节对应.csv`。侧摆与屈伸都是电机轴输入角度，不等于指节输出角度。

## 机构与坐标

- 115 个可见 CAD 零件实例、41 种零件/配置网格，81 个刚体、80 个转轴：16 主动、64 被动。
- 32 处闭合转轴，其中新增四指侧摆各 1 处；MJCF 中用每轴两点表示，共 64 条 connect 约束。
- 各侧摆电机带动主动连杆、耦合连杆和固定在指根上的侧摆从动支座。指根侧摆后，整根手指的屈伸机构随之转动。
- 沿用 V1.4 坐标及解剖动作正向：长度 m、角度 rad，全部转轴局部 `axis="0 0 1"`，正转服从右手定则。四指侧摆正向对应 V1.4 的 MCP 外展轴定义。
- 主动关节为 `*_motor_joint`；被动连杆为 `*_connecting_rod_数字`；指根侧摆输出为 `*_mcp_abd_output_joint`。小指 CMC 沿用总装位置，不额外添加独立输入。
- 保持隐藏/压缩状态：`手背顶板-1` 与 `装配体1-1` 未导出。

## 新模型引用处理

当前总装的子装配实际引用工作副本中的文件，拇指调整已保存于这些子装配。此次直接附接当前 SolidWorks 总装读取实际姿态和引用，避免把子装配替换回原目录中的旧状态。已归档实际引用文件用于追溯；提取前后 50 个实际来源文件的哈希保持不变。路径和哈希见 `verification/source_audit.json`，当前总装的捕获方式见 `config/source.json`。
导出器对无名指、小指两处跨子装配配合的零件归属存在误判；已用 SolidWorks 配合实体的真实零件名核实并纠正为“侧摆从动连杆”。记录见 `config/mate_owner_corrections.json` 及 `verification/cad_mate_owners.json`。

## 文件结构

```text
hand_motor_16dof_R/
  hand_tree_only.urdf          标准关节树
  hand.srdf                   16 个主动分组、64 个被动标记
  hand_closed_chain.xml       完整 MJCF 闭链
  hand_studio_preview.xml     Studio 预览范围保护版
  Studio导入包.zip            Studio 入口与全部网格
  meshes/                     115 个 STL 实例，单位 m
  config/                     闭链、分支、顺序、坐标、CAD 对照
  scripts/                    求解器与本地预览服务
  viewer/                     网页与离线依赖
  examples/control_active.py  仅控制主动输入的运动学示例
  verification/               本次导出的验证记录
  16电机定义.csv              1–16 顺序、轴心、正轴
  关节分类.csv
  CAD零件对照.csv
  与上一版关节对应.csv
  启动预览.ps1
  README.md
```

## 使用示例

```python
from scripts.solve_closed_chain import ClosureSolver
hand = ClosureSolver('hand_tree_only.urdf', 'config/loop_closures.yaml')
assert hand.independent[12] == 'index_mcp_abd_motor_joint'
q = hand.solve({'index_mcp_abd_motor_joint': 0.2,
                'index_mcp_motor_joint': 0.15})
# q 含 80 个转轴的位置，用于显示全部连杆；控制接口只取 hand.independent。
```

未指定的主动输入保持原值。被动/未知关节命令会拒绝；无法达到的目标保留上一有效姿态。换电脑后安装 Python 与 `requirements.txt` 中依赖，再运行 `python scripts/viewer_server.py --port 8099`。不要直接以 file:// 打开网页。

## 本次验证及范围

- 115 个零件在零位的世界变换与新 CAD 提取结果一致，最大矩阵误差 6.55e-15。
- 闭链雅可比秩 64；80 个转轴减去 64 个独立约束，得到 **16 个独立自由度**。
- 16 个主动输入与上一版对应动作正向一致，128 个坐标变换前后姿态一致。
- 1,964 步单输入往返/组合检查，加 161 步侧摆与屈伸混合运动；无分支翻面。单输入最大闭合残差 1.12e-10 m。
- MuJoCo 编译 115 个网格、80 个转轴、64 条闭链 connect；运动学与 URDF FK 一致。
- 固定版本 Studio 开源求解器完成 832 步测试，无未收敛、无翻面、无输入修正。最大残差 0.0096 mm；这不是对当前线上网站版本的完整验收。
- Studio 保护范围沿用小指 MCP −0.45～+0.6 rad、小指 PIP/DIP −0.15～+0.6 rad。新增侧摆四轴分别测试 ±0.6 rad；混合运动侧摆 ±0.3 rad、屈伸 ±0.25 rad、PIP/DIP 0～0.3 rad。
- 这些是运动学测试范围，不是实测机械限位；未验证零件碰撞或硬件可用范围。预览滑块 ±90° 仅表示操作范围。

本包用于几何和运动学核对；尚未设置实测惯量、碰撞、力矩、效率或硬件控制器。MJCF 使用统一密度生成预览惯量，关闭接触和重力，未完成动力学验收。SRDF 是语义标记，不能替代闭链求解器或 MoveIt 闭链规划集成。

第三方许可见 `THIRD_PARTY_LICENSE` 与 `viewer/vendor/`。
