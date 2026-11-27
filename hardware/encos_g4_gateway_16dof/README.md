# Encos G4 Gateway 16-DOF Driver (`encos_g4_gateway_16dof`)

基于 `LQS_Gateway_GUI_v0.5.0`（16-DOF / ±20 N·m MIT）移植的 Linux 实时驱动，与 `hardware/encos_g4_gateway_12dof` 并列维护。

## 关键规格
- 轴数：16 轴（ID `1..16`），挂载于 CAN 1–3（支持离线轴 CAN 1–3 动态自动发现）
- 控制模式：主机直发 `GW_CAN_TX`（`mode=2`），默认/上限频率 **150 Hz**
- MIT 前馈力矩量程：**±20 N·m**
- C++ 类：`lqs::HandClientG4_16dof`（`vendor/include/HandClientG4_16dof.hpp`）
- C ABI / 动态库：`g4_16dof_*`（`shim/g4_api_16dof.h` / `build/libg4_shim_16dof.so`）
- 专属配置：`config/right_sim_mapping_16dof.json`、`config/right_sim_feedback_16dof.json`

## 编译独立 C ABI 动态库

```bash
mkdir -p hardware/encos_g4_gateway_16dof/build
g++ -O2 -std=c++17 -fPIC -shared \
  -Ihardware/encos_g4_gateway_16dof/shim \
  -Ihardware/encos_g4_gateway_16dof/vendor/include \
  hardware/encos_g4_gateway_16dof/shim/g4_shim_16dof.cpp \
  -lpthread -o hardware/encos_g4_gateway_16dof/build/libg4_shim_16dof.so
```
