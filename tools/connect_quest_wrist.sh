#!/usr/bin/env bash
# 通过 USB 打开头显场景；--mirror 将同一个头显画面投到 Ubuntu。
set -euo pipefail
port=${QUEST_PORT:-8765}
adb_port=${QUEST_ADB_PORT:-5041}
mirror=false
if [[ ${1:-} == --mirror ]]; then mirror=true; shift; fi
if [[ ! "$port" =~ ^[1-9][0-9]{0,4}$ ]] || (( port > 65535 )); then
    echo 'QUEST_PORT 必须是 1 到 65535 的整数' >&2
    exit 2
fi
if (( $# > 1 )); then
    echo '用法：tools/connect_quest_wrist.sh [--mirror] [Quest 的 ADB 序列号]' >&2
    exit 2
fi
if ! curl --fail --silent --max-time 2 "http://127.0.0.1:$port/api/v1/scene" >/dev/null; then
    echo '场景尚未就绪，请先用 --headset 启动仿真。' >&2
    exit 1
fi
adb_cmd=(adb -P "$adb_port")
if (( $# == 1 )); then adb_cmd+=(-s "$1"); else adb_cmd+=(-d); fi
serial=$("${adb_cmd[@]}" get-serialno)
if [[ $("${adb_cmd[@]}" get-state) != device ]]; then
    echo '请在头显允许 USB 调试，并确认 Ubuntu 用户有该 USB 节点的读写权限。' >&2
    exit 1
fi
"${adb_cmd[@]}" reverse "tcp:$port" "tcp:$port"
"${adb_cmd[@]}" shell am start -a android.intent.action.VIEW -d "http://localhost:$port/" -p com.oculus.browser
echo '在 Quest 浏览器点击「进入 Quest 场景」，掌背朝上、指尖向前，按右手柄食指扳机对齐右腕。'
if "$mirror"; then
    scrcpy_bin=${SCRCPY_BIN:-$(command -v scrcpy || true)}
    if [[ -z "$scrcpy_bin" ]]; then scrcpy_bin="$HOME/.local/opt/scrcpy-linux-x86_64-v5.0.1/scrcpy"; fi
    if [[ ! -x "$scrcpy_bin" ]]; then echo '请安装 scrcpy 或通过 SCRCPY_BIN 指定可执行文件。' >&2; exit 1; fi
    export ADB_SERVER_SOCKET="tcp:localhost:$adb_port" ANDROID_ADB_SERVER_PORT="$adb_port" ADB
    ADB=$(command -v adb)
    exec "$scrcpy_bin" -s "$serial" --no-control --no-audio --max-size=1920 --max-fps=30 --window-title='Quest 3S · 头显实时投屏'
fi
