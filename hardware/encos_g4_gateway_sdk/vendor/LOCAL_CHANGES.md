# 本项目对SDK的最小扩展

2026-11-12：原生驱动阻抗的速度估计需要位置反馈的真实主机接收/解析时间，不能使用快照消费线程的轮询时间。

- `GatewayProtocol.hpp` 增加独立的 `TimedHandSnapshot` 和 `RouteState::TimedSnapshot()`，携带既有 `MotorFeedback::position_time` 与独立 `current_time`。该值是SDK解析位置反馈时的主机 `steady_clock` 微秒时间，不是电机内部采样时刻。
- `HandClientG4.hpp` 增加 `GetTimedSnapshots()`；位置值、样本计数和时间在同一快照锁中发布。消费旧快照或只收到补充电流帧不会刷新位置时间；仅位置补包也不会刷新电流时间。
- 计算快照年龄的 `NowUs()` 移到本轮串口读取与解析之后，避免本轮新位置时间晚于循环开始时间导致无符号减法下溢。
- 原 `HandSnapshot` 布局（88字节）、`GetSnapshots()`、网关协议及Python shim/C ABI保持兼容。原生控制使用新的快照接口；现有Python调用无需新增参数。

离线测试：`Motorhand_FFB/tests/test_encos_gateway_snapshot_native.cpp`。没有为验证此扩展打开真实串口或启动电机。

2026-11-16：共享 Python C ABI 包装层 `../shim/g4_shim.cpp` 改为直接使用 `GetTimedSnapshots()` 的位置接收时间（微秒转纳秒），保持 `G4Joint` 布局及 Python 调用参数不变。原实现依赖连接后启动的 watchdog 补写时间戳，可能在已有 12 轴反馈时仍返回零时间戳，导致标零和只读检查偶发全轴离线。

离线回归：`sim/tests/test_g4_snapshot.py` 使用伪终端运行实际 SDK，在 watchdog 未启动的情况下验证首批反馈时间戳、时钟来源及断流过期行为。此修复保留指令期限 watchdog、软释放及电机控制报文。

2026-11-17：为 SuperDex 只读跟随增加独立入口 `g4_open_reader`。SDK 的 query_only 会话在采集循环中固定生成位置查询，断开时跳过控制会话使用的三轮卸力帧。原有控制入口保留原行为。

2026-11-19：`HandClientG4.hpp` 增加 `SetCommandBatch()`，在同一指令锁中更新 12 轴。SuperDex 的 `g4_set_mit_batch` 先校验各轴参数及共同的命令期限，再一次提交给 SDK IO 线程。伪终端验证了无效批次拒绝、逐轴路由、过期及卸力。

2026-11-20：串口 IO 线程在 Linux 报告挂断或终止性设备错误时锁存 `TransportLost()` 并结束读取。普通无回包、`EAGAIN` 与输入超过反馈期限继续走原超时路径。shim 的 `g4_open_fresh` 创建新的 SDK、指令与看门狗实例，省去固定等待；原生控制使用 12 轴的新位置样本判断就绪。伪终端热插拔回归位于 `tools/tests/test_g4_hotplug.py`。

2026-11-20：记录本次连接最后一次收到有效路由回包的单调时间，并识别同步 EPIPE。CH341 的 USB 端点停顿有时不会产生 tty 挂断。原生后端通过最后回包时间触发持续全链路静默恢复，复用原有重连和卸力流程。

2026-11-22：批次续接增加 SetCommandBatchBefore。在指令互斥锁内校验最迟提交时刻，配合 shim 校验新旧两批输入期限；原普通 SetCommandBatch 接口继续可用。

2026-11-22：加固非阻塞串口收发与断线检测。`PosixSerial::Open` 增加 `O_CLOEXEC`；新增 `WriteAll` 与 `WaitReadableNs`（基于 `ppoll`），在 `HandClientG4::IoLoop` 与 `SendReleaseBatchAll` 中处理 `EINTR`/`EAGAIN` 与部分写并保留半帧尾字节，避免 32 字节协议帧被腰斩；周期等待改为事件驱动 `ppoll` 即时读取并发布快照，同时防护 `g4_transport_lost` 与年龄计算的无符号下溢。

2026-11-23：同步 `LQS_Gateway_GUI v0.4.0`（MIT 力矩量程调整版）改动，将 `GatewayProtocol.hpp` 中 `PackMit()` 的前馈力矩限幅与 12-bit 线性映射量程由 `[-10.0, 10.0]` 放宽至 `[-20.0, 20.0] N·m`，完整保留本地串口与时间戳扩展。
