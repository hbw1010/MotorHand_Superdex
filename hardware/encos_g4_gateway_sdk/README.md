# VE33 右手带动 SuperDex

从仓库根目录运行。当前 G4/Quest 入口使用项目内的右手映射、CAD 闭链求解器、物理接触投影和 G4 前馈控制。可通过 `--mapping` 指定另一份右手映射。[用户需求、反馈链路和标定状态](../../docs/motorhand-force-feedback.md) 随分支维护。

首次从源码运行或更新 shim 后编译一次。

```bash
mkdir -p hardware/encos_g4_gateway_sdk/build
g++ -std=c++17 -O2 -shared -fPIC -pthread \
  -Ihardware/encos_g4_gateway_sdk/vendor/include \
  hardware/encos_g4_gateway_sdk/shim/g4_shim.cpp \
  -o hardware/encos_g4_gateway_sdk/build/libg4_shim.so
```

首次运行时安装模型处理依赖 `uv pip install --python .venv/bin/python vtk`，并构建网页。

```bash
npm --prefix tools/quest_view ci
npm --prefix tools/quest_view run build
```

启动 SuperDex 仿真和 MotorHand 网页控制台。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0
```

在 Ubuntu 浏览器打开 `http://127.0.0.1:8765/`。网页可看场景、跟踪状态和 12 轴数据，操作力反馈、右腕对齐与标零。鼠标左键拖动旋转，滚轮缩放，右键拖动平移。标零时将右手摆到 VE33 的参考零姿态，保持静止，点击「标零并卸力」。成功后轻动食指，在网页查看 ID 5、6 的编码器角度（°）、换算后的模型电机角（rad）和 SuperDex 实际回读角（rad）。需要查看原有物理调试器时加 `--debugger`，该模式仅采集位置，可点调试器顶部 ▶ 继续步进。

也可执行以下命令标零。仿真运行时，命令通过网页服务请求采样并应用零偏；仿真关闭时，命令直接读取网关。两种方式使用相同的串口、轴映射和标零文件。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --zero
```

采集约 0.6 秒，每轴至少需要 10 个独立位置样本，窗口内角度极差须在 0.5° 以内；任一轴缺失、反馈过期或报警会取消本次标零。12 轴全部通过后原子替换当前用户的 `~/.config/superdex/ve33-g4-right-zero.json`。更换手套、重置编码器或更换佩戴参考后应重新标零。不同设备可用 `--zero-file` 分开保存。软件标零记录当前编码器值，要求使用者确认参考姿态。不会移动电机或更改编码器内部零位。

模型角度按 `radians((编码器角度 - 软件零偏) × 标定方向 × 比例)` 换算。全部 12 轴有效时一起求解 68 个树关节。闭链拒绝或位置帧超过 100 ms 时保留上次有效姿态。结束采集时，在启动终端按 Ctrl+C。

## Ubuntu 魔方环境与 Quest 腕部定位

Ubuntu 运行 SuperDex 物理，桌面浏览器和头显共用场景。G4 提供手指输入，默认使用 Quest 右手柄的 WebXR gripSpace 提供六自由度定位，进入场景前也可选择光学手部追踪。官方二阶魔方受 −Z 方向的 9.81 m/s² 重力，桌面提供支撑、摩擦和落地阻尼。载入时将魔方中心固定在世界坐标 (0.230, −0.015, 0.034) m，保留角块转层和对齐弹簧；手腕保留物理退让；指节按实测电机角解闭链，接触时只限制继续闭合的轴，撤回或脱离物体后释放。掌板等复合刚体按独立零件构造碰撞面，开边零件及 14 段空心指环、指托用封闭凸包表示。各活动关节之间保留间隙，填实前的惯量继续参与仿真，几何扫描也使用封口后的碰撞面。

魔方运行资产已随项目提供在 `assets/prefabs/puzzle_cube_quest`，来自官方提交 `da6d65b1131ba7680d6b2bb5293c947147432609`。魔方的署名与许可见资产目录的 LICENSE。

网页预览下方的「魔方模式」默认固定中心。切到「自由抓取」会开放整颗魔方的平移和转动，仍可拧层。松手后魔方会下落，桌面可接住桌板范围内的魔方。切回「固定中心」会把魔方锁在当前位置和朝向。点击「初始化场景」会让虚拟手腕和场景内所有物体回到各自起点，清除旧接触限位，并复原魔方的层状态和默认固定模式。右手柄定位有效时会以点击时的手柄位姿重新建立腕部原点。指节继续读取真机，实机标零和力反馈开关继续使用当前设置。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --quest
```

1. Quest 开启开发者模式，通过 USB 数据线连接 Ubuntu，在头显内允许 USB 调试。戴好头显，唤醒右手柄并使它与右手保持固定相对位置。
2. 仿真启动后执行 `tools/connect_quest_wrist.sh`。如果有多台 Android 设备，传入 Quest 的 ADB 序列号。
3. 在 Quest 浏览器点「进入头显场景」，允许透视。页面通过 WebXR 读取选定的定位源，网页控制台会从「等待头显位姿数据」变为「右腕已连接，请对齐参考姿态」。
4. 面朝正前方，右手掌背朝上、指尖向前，把手放在想作为起点的位置，按右手柄食指扳机或点击网页的「对齐右腕」。从此实际腕部相对该参考的平移与旋转驱动仿真手根部。若需重设起点，再点一次。手指编码器标零是另一个独立操作。
5. 移动、旋转右腕并单独弯曲食指，确认整手移动与手指运动可以独立发生。腕部采样超时后保持上一次目标腕位姿，手指和力反馈继续与当前场景交互。Quest 浏览器重启会话或参考空间重置后需要重新对齐，才能重新移动右腕。

当前采用单姿态配准，假设 VE33 基座原点与所选跟踪点重合（手柄模式为握持点）；尚未测量佩戴时腕点到手套底板的固定偏移。默认 USB 端口为 8765，浏览器访问 `http://localhost:8765/`。更换端口时同时使用 `--quest-port` 和连接脚本的 `QUEST_PORT` 环境变量。

### 为后续头显显示保留的接口

Ubuntu 是物理场景的唯一计算端。以下 HTTP 接口与 VTK 无关，绑定本机回环地址，可经 ADB reverse 从头显访问。未来显示端可预装相同的 CAD/GLB，按刚体名称更新网格。

| 接口 | 数据 |
|---|---|
| `POST /api/v1/wrist` | JSON 对象 `{space, seq, pose}`。space 为本次参考空间标识，seq 单调递增。pose 为 WebXR local 空间下的 `[x,y,z,qx,qy,qz,qw]`，单位米；追踪丢失传 null。 |
| `GET /api/v1/scene` | 返回版本、帧号、主机单调时钟秒数、追踪状态，以及 `right_hand` 和 `puzzle_cube` 两个名称到世界位姿的映射。位姿顺序同上，场景坐标为 X 前、Y 左、Z 上，长度单位米。时间戳属于 Ubuntu 主机时钟。 |
| `GET /api/v1/control` | 返回 12 轴采集和仿真数据、标零与追踪状态、力反馈请求和实际提交状态。 |
| `POST /api/v1/control` | JSON 对象 `{action}`，可用 `enable`、`disable`、`align` 或 `zero`。Web 交互场景支持 `reset_scene`。普通魔方场景还支持 `cube_mode`（需要 `mode: fixed` 或 `mode: free`）。 |

网页显示手部是否接触魔方。`hand_cube_contacts` 按角块列出几何扫描拦到或物理接触力超过 0.001 N 的手部链接。`target_cube_contacts` 单独列出未限位目标姿态与角块外壳的碰撞点所属链接。
`blocked_motor_ids` 列出受阻电机 ID。`blocked_extension_motor_ids` 列出其中的伸展受阻电机。`palm_contact` 表示掌部与角块接触。

头显输入上传腕部位姿和对齐请求，场景输出包含 73 个手部和 16 个魔方链接。完成对齐后，场景帧中的 `registration.space` 标识 Quest 参考空间，`registration.xr_from_world` 给出桌面世界到该参考空间的刚体变换。显示端可将该变换左乘到手部和魔方的各个世界位姿，在同一参考空间渲染。更换参考空间后该字段恢复为 null，需重新对齐。


### Quest 显示、Ubuntu 镜像

物理与 G4 采集在 Ubuntu 运行。Quest 浏览器在本机为左右眼绘制银色连杆、电机、黑色尼龙零件及官方魔方。模型在启动时一次性下载，随后按场景帧更新 89 个链接。桌面镜像使用 scrcpy 的只读 USB 视频流。

首次安装前端依赖并构建：

```bash
(cd tools/quest_view && npm ci && npm run build)
```

启动仿真，然后在另一个终端连接 Quest 并打开投屏窗口。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --headset
tools/connect_quest_wrist.sh --mirror
```

在 Quest 内选择「右手柄」并点击「进入头显场景」。未配准时可看见面前的手和魔方预览。右手按参考姿态摆好后，按右手柄食指扳机（光学模式用左手捏合），或点网页的「对齐右腕」。Quest 运行时可能限制投屏中的透视背景，手与魔方是应用自身绘制的内容。退出沉浸视图后可重新进入。

连接脚本使用独立的 ADB 服务端口 5041，可用 QUEST_ADB_PORT 改写。USB 重新枚举后如果 ADB 提示 no permissions，请根据 lsusb 确认当前 Oculus 设备的总线和编号，再授予桌面用户对该设备节点的读写权限。

## 仿真接触与手套反馈

源版本为 Motorhand_Teleop_transfer 提交 `a7da417f6f295eb49b5ea2a9d7297f38395b9571` 的 `sim/src/glove_sim`、`sim/config/default.yaml` 和 `hardware/encos_dexhand/configs/sim/glove_mapping.json`。本项目中的代码位于 `tools/motorhand`，运行配置在 `hardware/encos_g4_gateway_sdk/config`。G4 入口按电机 ID 1–12 排序，使用本机已保存的软件零偏和 sim 映射的相对行程限位。参考工程用于 MuJoCo 的接触检测在此换为 SuperDex 对魔方外壳的接触检测；两引擎的接触位置和触感仍需分别验证。

固定中心场景使用连续几何扫描限制穿模，腕部触碰后保留沿表面的平移分量。扫描和受阻角留作场景保护及诊断。真机出力以物理子步结束时的实际对象接触为输入。接触点的力和相对连杆质心的力矩先通过物理引擎的雅可比映射到树关节，再通过闭链导数投影到 J1–J12。掌板和腕部的总接触力可在诊断手查看，直接力矩输出取决于该接触对电机轴的实际传力。

自由抓取的虚拟指节由闭链目标和物理姿态控制器驱动，在物理子步之间保留求解后的姿态与速度。该模式使用 1.5 N·m/rad 的虚拟电机轴刚度，拇指、食指指腹和魔方的接触对使用摩擦系数 1.25、摩擦过渡速度 1 mm/s 和法向阻尼 1 s/m。切回固定中心后重新使用扫描限制指节。场景帧的 `finger_control` 表示当前处理方式，`physics` 为自由抓取，`contact_guard` 为扫描限制。

- 主手反馈合成仿真实际接触反力和同向位置补偿。接触帧携带实际驱动该步的编码器角与仿真回读角。位置补偿使用这两个角、参考阻抗的基础刚度和 0.002 rad 死区，随真机回撤和 20 ms 预测间隙退出。投影反力从 0.01 N·m 增至 0.25 N·m 时，位置补偿权重平滑升至 1。位置项由 `position_compensation_max_nm` 独立限峰，默认每轴 2.0 N·m。
- 接触反力在 0.01–0.03 N·m 之间平滑渐入。各轴速度用自身编码器包时间计算，接触时按 0.45 N·m·s/rad 施加反向速度阻尼，反力方向由物理接触决定。合成反馈使用 0.15 s 渐入、25 Hz 滤波和 60 N·m/s 限斜率。失去接触时该轴命令归零。
- 当最新编码器相对接触帧所用的编码器角，沿该帧反力方向已经回撤 1°–3° 时，手套侧将该轴的旧帧指令平滑衰减至零，同时压低滤波器里残留的力矩。回撤和速度使用限位前的编码器角，仿真目标使用行程内的模型角。新帧跟上后按原斜率恢复。该处理作用于仿真解算后的手套指令，软体接触、形变和反力继续由物理求解产生。
- 每轴最大 10 N·m，按 95% 余量将模型反馈和 G4 电机轴的前馈项分别限在 ±9.5 N·m 以内。MIT 位置、速度、Kp 和 Kd 字段均为零，前馈力矩按现有电机方向和比例换算。
- 投影接触反力按仿真电机轴的 N·m 直接参与合成，角差补偿使用基础刚度。12 轴指令经过统一校验后整批交给 G4，新接触帧可和仍有效的编码器样本合成一批指令。有效期由使用的最旧样本起算。
- 启动时默认请求启用力反馈。已标零、12 轴采集和仿真接触帧有效并连续稳定 180 ms 后，按 0.15 s 渐入。手柄丢失或头显退出时保持腕部位置，反馈继续。想先看仿真时可加 `--preview`。点击「停止力反馈」或「标零并卸力」会主动卸力。采集或接触帧过期时停止提交，新鲜输入恢复后重新渐入。G4 看门狗仍执行 0.10 s 指令有效期。遇到输出故障时暂停提交，排查后可手动停止再启用。
- 网页在恢复输出后保留本次运行的中断次数和上次原因，可区分编码器采集异常、仿真使用的编码器帧过期、接触帧不可用和指令到期。
- 接触力模式下，如果上一批过期不超过两个控制周期，新帧至少还有一个控制周期的有效期，程序按新帧自己的期限从零渐入。接续次数及最近一次超期时长单独记录。新输入已过期、余量不足或中断时间更长时进入原有的连续有效帧检查。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --preview
```

离线验证均使用仿真或伪终端：

```bash
.venv/bin/python tools/tests/test_ve33_contact_stream.py
.venv/bin/python tools/tests/test_ve33_feedback.py
```


## 临时桌面联调

使用项目内生成的固定桌面与 70 mm 测试方块。桌面和方块都有 SuperDex 碰撞体，可测试闭合、伸展和掌部接触。按右手柄对齐后，桌面上表面位于参考手腕下方 80 mm。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --tabletop
```

网页控制台分别显示手套采样年龄、仿真接触帧和腕部样本年龄。每轴显示实测换算角、同一接触帧的仿真角、接触投影力矩、位置补偿、滤波后的合成力矩、已下发的电机轴前馈力矩和驱动器回报电流。头显浏览器通过 `POST /api/v1/wrist` 上传位姿并接收空响应，通过单独的 `GET /api/v1/scene` 读取画面。旧客户端仍可设置 `want_scene: true` 随位姿请求获取场景。


## 本地固定腕部测试

本地抓握默认使用 C++ 步进和物理接触采样。首次运行、修改扩展或更换 SuperDex 安装后，用当前 Python 环境重新编译。构建需要 C++20、CMake 3.26 或更新版本和 Ninja。若虚拟环境缺少后两个工具，可用 `uv pip install --python .venv/bin/python 'cmake>=3.26' ninja` 安装。

```bash
.venv/bin/cmake -S tools/motorhand/native -B tools/motorhand/native/build \
  -D Python_EXECUTABLE="$PWD/.venv/bin/python" -D CMAKE_BUILD_TYPE=Release \
  -G Ninja -D CMAKE_MAKE_PROGRAM="$PWD/.venv/bin/ninja"
.venv/bin/cmake --build tools/motorhand/native/build --parallel
```

```bash
# 不接手套，使用网页演示
.venv/bin/python tools/ve33_g4_view.py --local-grasp --demo

# 接入已标零的 G4
.venv/bin/python tools/ve33_g4_view.py --local-grasp --port /dev/ttyUSB0
```

如需对照旧后端，可加 `--grasp-backend python`。控制接口 `GET /api/v1/control` 中的 `grasp_backend` 会返回活动后端。

本地抓握与 Quest 魔方「自由抓取」共用力反馈路径。手指在每个 1/240 s 物理子步读取最新编码器目标，子步之间保留解算后的姿态和速度。每轮发布最后一个子步的接触力矩及其实际输入，G4 控制进程使用同一个控制器合成并提交纯前馈力矩。网页通过 `GET /api/v1/scene?after=<帧号>` 等待后续帧，超时时返回 204。控制界面中的接触帧龄从该次求解使用的最早输入计算。

在 Ubuntu 浏览器打开 `http://127.0.0.1:8765/` 即可使用固定腕部抓握。掌心场景默认锁定一个 45 mm 软球。六种物体均可调至 20–90 mm，并可修改硬度。软球由四面体弹性体模拟，受压后形变并回弹。软方块与软圆柱用柔顺接触模拟。桌面场景沿用参考仿真的零重力设置，左键拖动预览中的物体，使用旋转滑条调整刚体朝向。锁定物体会保持中心，软球仍能形变。「物体归位」复位物体，「初始化场景」同时复位场景中的物体和虚拟手，手指继续跟随当前手套角度。

「物体」下拉框的「SuperDex 官方资产」包含 17 项。刚性球、四色方块、单杯、软鸭灯、独立插销、链条、Fidget Box 和两版魔方可放在掌心，也可切到桌面。分隔盒与方块、纸杯金字塔、两种插销板和形状配对盒直接进入桌面。分隔盒与方块和纸杯金字塔按桌面工作区等比缩小。官方物体沿用预制件的碰撞、材料和内部约束，切换时以未锁定状态开始，旋转滑条重新摆放预制件。

固定腕部加载 Quest 版魔方时，沿用 Quest 场景的碰撞查询方向，由手部采样点查询外壳 SDF。软体鸭灯使用与本地软球相同的单点表面接触积分。

右手按参考仿真的姿态直立，手指朝世界 +Z，掌面法线朝世界 −Y。掌心物体以腕部坐标计算位置和旋转，桌面保持水平，桌面物体绕世界 +Z 旋转。

接入 G4 后，默认可预览合成反馈，确认已标零和 12 轴在线后可点击「启用力反馈」，或启动时添加 `--feedback`。各轴力矩由仿真物理接触及同向角差共同计算，保留 ±9.5 N·m 限幅及 100 ms 指令有效期。演示模式提供自动或手动抓握。要同时运行两个实例，可以对真机命令加 `--quest-port 8766` 并打开 `http://127.0.0.1:8766/`。

点击「开始记录」后逐控制周期写入时间戳、物体设置、12 轴角度和速度、接触帧的编码器角、接触投影、位置补偿、合成及已发力矩和电流，以及中断和接续诊断。`encoder_rad` 和 `contact_encoder_rad` 分别是当前和接触帧的限位前编码器角。停止记录后可从网页下载 CSV。文件也保存在 `${XDG_STATE_HOME:-$HOME/.local/state}/superdex/recordings/`。

旧版硬球测试入口如下。

```bash
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --palm-test
```

固定腕部场景使用参考仿真 `ball_hard` 的 45 mm 球、硬度 0.92 和掌坐标中心 (72, −5, −29.5) mm。实际手指角仍来自 G4。用 `--ball-size-mm 60` 可将球直径改为 60 mm，范围沿用参考仿真的 25–200 mm。球靠掌一侧保持在 −7 mm。网页显示 12 轴在线和「输出中」后，可合指握球并松开。此模式无需头显对齐，反馈仍受手套与物理帧有效期管理。

## 官方 Puzzle Cube 联调

```bash
MOCHI_AUTO_LOAD="$PWD/assets/prefabs/puzzle_cube_quest/tuning/grip_lift.mochi_scene" .venv/bin/superdex-studio
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0 --official-puzzle --inspect-physics
```

Studio 打开原版 `grip_lift` 场景及同目录官方调参文件。实机版本用 MotorHand 取代原夹块，把魔方放到固定腕部的掌心并固定中心，保留官方角块、接触属性与 56 组定位约束。使用 `--inspect-physics` 时，主进程专注于接触和力反馈，官方 Physics Debugger 在独立进程里显示镜像。点击调试器的播放按钮启动镜像刷新，镜像的暂停和单步独立于实机仿真。网页控制台显示实时反馈参数和数据年龄，其中电机回报 A 来自驱动器的有效电流样本。


## 双手魔方与受力诊断

```bash
npm --prefix tools/quest_view run build
.venv/bin/python tools/ve33_g4_view.py --port /dev/ttyUSB0
```

右手仍用 G4 采集指节角。头显有左手光学追踪时，以指尖和掌心参与魔方接触；也可用左手柄的握持位置当作仿真支撑点。左侧仅参与仿真，两手共享右手参考姿态对齐后的坐标，缺失左手样本时移除支撑点。控制器和光学手能否同时提供由头显的 OpenXR 运行时决定。

浏览器中也能看到桌面预览。上方半透明手取仿真实际右手姿态，颜色按连杆接触合力着色，箭头从该连杆原点指向合力方向。右侧载荷按手指分成五行，标出 J1–J12 的关节名，用横条显示模型名义反馈力矩的大小，满格对应 10 N·m。左侧峰值名义力矩与同轴电流分别来自规划器和有效电机回报。调节「颜色满量程」只改变半透明手上的力矢量显示。
