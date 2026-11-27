# 16-DOF G4 网关驱动本地移植与优化说明

基于厂商 `LQS_Gateway_GUI_v0.5.0`（16-DOF / ±20 N·m MIT）构建 Linux 实时驱动，完整对齐 v0.5.0 硬件协议并移植 `encos_g4_gateway_12dof` 的全部稳定性优化：

1. **严格对齐 v0.5.0 16-DOF 硬件协议**：
   - 全部 16 轴（ID `1..16`）位于 **CAN 1–3** 总线（`bus ∈ {1, 2, 3}`），不在不存在的 CAN 4 发包。
   - 遵循 GW02 板端 12 槽周期表限制与 `routes * (hz + 10) <= 2600` 带宽公式：连接时非阻塞下发 `PollControl(0)` + `PollControl(2)` 关闭残留板端轮询，固定采用主机直发 `GW_CAN_TX`（`mode=2`），并将 16 轴目标频率默认/上限钳制为 **150 Hz**（921600 bps 串口下占空 768 kbps）。
   - MIT 前馈力矩保持 v0.4.0/v0.5.0 的 **±20 N·m** 12-bit 线性映射量程。
2. **完整移植 `encos_g4_gateway_12dof` 实时与容错优化**：
   - `PosixSerial` 非阻塞加固（`O_CLOEXEC`、`WriteAll` 有界完整写入、`ppoll` 事件驱动读取 `WaitReadableNs`、32B 半帧尾字节保留与 `HasHangup`/`IsTerminalErrno` 断线识别）。
   - `TimedHandSnapshot` + `GetTimedSnapshots()`（真实主机 `steady_clock` 位置/电流收包时间，`NowUs()` 延后采集防无符号下溢）。
   - CAN 1–3 动态总线路由自动发现与双总线重号保留告警。
   - 原子 16 轴批次下发 `SetCommandBatchBefore`、CH341 3s 静默断流识别、只读跟随 `g4_16dof_open_reader`、指令期限 watchdog 软卸力与关闭 3 轮 16 轴安全卸力。
3. **独立并列命名**：
   - C++ 客户端类：`lqs::HandClientG4_16dof`（与 `lqs::HandClientG4_12dof` 并列，可同翻译单元并存）。
   - C ABI 与动态库：`g4_16dof_*` / `build/libg4_shim_16dof.so`（与 `g4_12dof_*` / `libg4_shim_12dof.so` 零符号冲突）。
