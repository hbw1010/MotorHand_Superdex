# Encos G4 Gateway 12-DOF Driver (`encos_g4_gateway_12dof`)

基于 `LQS_Gateway_GUI_v0.4.0`（12-DOF / ±20 N·m MIT）的 Linux 实时驱动，与 `hardware/encos_g4_gateway_16dof` 并列维护。

## 关键规格
- 轴数：12 轴（ID `1..12`），挂载于 CAN 1–3（支持离线轴 CAN 1–3 动态自动发现）
- 控制频率：默认 **200 Hz**
- MIT 前馈力矩量程：**±20 N·m**
- C++ 类：`lqs::HandClientG4_12dof`（`vendor/include/HandClientG4_12dof.hpp`，兼容别名 `HandClientG4`）
- C ABI / 动态库：`g4_12dof_*`（兼容 `g4_*`，`shim/g4_api_12dof.h` / `build/libg4_shim_12dof.so`）
- 专属配置：`config/right_sim_mapping_12dof.json`、`config/right_sim_feedback_12dof.json`

## 编译独立 C ABI 动态库

```bash
mkdir -p hardware/encos_g4_gateway_12dof/build
g++ -O2 -std=c++17 -fPIC -shared \
  -Ihardware/encos_g4_gateway_12dof/shim \
  -Ihardware/encos_g4_gateway_12dof/vendor/include \
  hardware/encos_g4_gateway_12dof/shim/g4_shim_12dof.cpp \
  -lpthread -o hardware/encos_g4_gateway_12dof/build/libg4_shim_12dof.so
```
