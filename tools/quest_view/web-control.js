import {RotationControl} from './rotation-control.mjs';
import {sizePreview} from './size-preview.mjs';

const panel = document.querySelector('#g4-control');
const rows = document.querySelector('#axes-body');
const feedbackStatus = document.querySelector('#feedback-state');
const timing = document.querySelector('#timing');
const calibration = document.querySelector('#calibration-state');
const commandMessage = document.querySelector('#command-message');
const cubeSettings = document.querySelector('#cube-settings');
const cubeModeControl = document.querySelector('#cube-mode-control');
const cubeMode = document.querySelector('#cube-mode');
const resetScene = document.querySelector('#reset-scene');
const sceneMessage = document.querySelector('#scene-message');
const graspPanel = document.querySelector('#local-grasp');
const graspInputs = [...graspPanel.querySelectorAll('[data-grasp]')];
const graspKind = graspPanel.querySelector('[data-grasp="kind"]');
const rotationInput = graspPanel.querySelector('[data-grasp="angle_deg"]');
const sizeInput = graspPanel.querySelector('[data-grasp="size_mm"]');
const graspHome = document.querySelector('#grasp-home');
const graspMessage = document.querySelector('#grasp-message');
const demoControls = document.querySelector('#demo-controls');
const demoGrip = document.querySelector('#demo-grip');
const demoAuto = document.querySelector('#demo-auto');
const record = document.querySelector('#grasp-record');
const csvLink = document.querySelector('#grasp-csv');
const offsetPanel = document.querySelector('#wrist-offset-cal');
const offsetCurrent = document.querySelector('#offset-cal-current');
const offsetStart = document.querySelector('#offset-cal-start');
const offsetCancel = document.querySelector('#offset-cal-cancel');
const offsetReset = document.querySelector('#offset-cal-reset');
const offsetStatus = document.querySelector('#offset-cal-status');
const taskTabs = document.querySelector('#task-tabs');
const taskButtons = taskTabs ? Array.from(taskTabs.querySelectorAll('[data-task]')) : [];
const taskSwitchStatus = document.querySelector('#task-switch-status');
const buttons = Array.from(panel.querySelectorAll('[data-control]'));
let busy = false, timer = 0;
let sceneRevision;
let loadedTask = null, switchingToTask = null;
let switchingObject = false;
let sizeCancelled = false;
let sizeCommit = Promise.resolve();

const rotation = new RotationControl((kind, angle) => postCommand(
  {action: 'configure_grasp', expected_kind: kind, values: {angle_deg: angle}}, graspMessage));

const number = (value, digits) => Number.isFinite(value) ? value.toFixed(digits) : '—';

function tracks(state) {
  if (state.diagnostic_stale) return `等待新控制快照 · 上次采样 ${number(state.snapshot_age_ms, 0)} ms 前`;
  return [
    `${state.online ?? 0}/12 ${state.demo ? '轴演示' : '轴在线'}`,
    state.wrist ?? '等待定位',
    `采集 ${number(state.motor_ms, 0)} ms`,
    `仿真使用 ${number(state.applied_ms, 0)} ms`,
    `接触帧 ${number(state.contact_ms, 0)} ms`,
    state.fixed_wrist ? '固定腕部' : `定位 ${number(state.wrist_ms, 0)} ms`,
    `同帧最大角差 ${number(state.max_angle_error_rad, 3)} rad`,
  ].join(' · ');
}

function graspValue(input) {
  const key = input.dataset.grasp;
  const out = graspPanel.querySelector(`[data-value="${key}"]`);
  if (out) out.textContent = input.value + (key === 'size_mm' ? ' mm' : key === 'angle_deg' ? '°' : '');
}

function showGrasp(state) {
  const grasp = state.grasp;
  graspPanel.hidden = !grasp;
  if (!grasp) {
    sizePreview.clear();
    return;
  }
  sizePreview.controlSnapshot(grasp);
  document.querySelector('#grasp-clearance').hidden = !grasp.awaiting_hand_clearance;
  if (state.grasp_assets?.length && !graspKind.querySelector('optgroup')) {
    const group = document.createElement('optgroup');
    group.label = 'SuperDex 官方资产';
    for (const {kind, label} of state.grasp_assets) group.append(new Option(label, kind));
    graspKind.append(group);
  }
  for (const input of graspInputs) {
    input.disabled = busy || switchingObject || state.calibrating;
    if (busy || switchingObject || input.matches(':active')) continue;
    const value = grasp[input.dataset.grasp];
    if (input === rotationInput && rotation.waitingForEcho(grasp.kind, value)) continue;
    if (input === sizeInput && sizePreview.target) continue;
    if (input.type === 'checkbox') input.checked = value;
    else input.value = value;
    graspValue(input);
  }
  graspHome.disabled = record.disabled = busy || state.calibrating;
  demoControls.hidden = !state.demo;
  if (state.demo && state.demo_grip >= 0 && !demoGrip.matches(':active')) demoGrip.value = state.demo_grip;
  record.dataset.recording = state.recording ? 'yes' : 'no';
  record.textContent = state.recording ? '停止记录' : '开始记录';
  csvLink.hidden = !state.recording_file || state.recording;
  if (state.recording_error) graspMessage.textContent = state.recording_error;
  else if (state.recording_dropped) graspMessage.textContent = `记录丢失 ${state.recording_dropped} 个采样周期`;
}

function fmtOffsetMm(vec) {
  if (!Array.isArray(vec) || vec.length !== 3) return '—';
  const f = v => `${v >= 0 ? '+' : ''}${(v * 1000).toFixed(1)}`;
  return `(${f(vec[0])}, ${f(vec[1])}, ${f(vec[2])}) mm`;
}

function showWristOffset(state) {
  if (!offsetPanel) return;
  const cal = state.wrist_offset;
  offsetPanel.hidden = Boolean(state.fixed_wrist || !cal);
  if (offsetPanel.hidden) return;
  const badge = cal.calibrated_6dof ? `6DoF 已标定 · 杆臂 ${fmtOffsetMm(cal.offset_m)}` : `未标定 · 偏移 ${fmtOffsetMm(cal.offset_m)}`;
  offsetCurrent.textContent = `当前：${badge}`;
  const active = Boolean(cal.overlay_active);
  offsetStart.hidden = active;
  offsetCancel.hidden = !active;
  offsetStart.disabled = busy || state.calibrating;
  offsetCancel.disabled = busy || !active;
  offsetReset.disabled = busy || active;
  if (active) {
    offsetStatus.textContent = '虚实重合标定中：在头显里把真实右手手套伸进眼前虚拟手重合，按【左手扳机 / X 键】确认并自动保存（【左手侧握】重摆位置，【左手 Y】取消）。';
  } else if (cal.message) {
    offsetStatus.textContent = cal.message;
  } else {
    offsetStatus.textContent = '全程在头显内用左手柄完成：未标定时进入头显自动触发，或随时按【左手 X / 侧握】重标。';
  }
}

function showTaskTabs(state) {
  if (!taskTabs) return;
  const tasks = state.available_tasks;
  const current = state.task;
  taskTabs.hidden = !Array.isArray(tasks) || tasks.length < 2;
  if (taskTabs.hidden) return;
  if (!loadedTask && current) loadedTask = current;
  if (switchingToTask && current === switchingToTask) {
    window.location.replace(`${window.location.pathname}?t=${Date.now()}`);
    return;
  }
  if (!switchingToTask && loadedTask && current && current !== loadedTask) {
    window.location.replace(`${window.location.pathname}?t=${Date.now()}`);
    return;
  }
  for (const btn of taskButtons) {
    const t = btn.dataset.task;
    btn.hidden = !tasks.includes(t);
    btn.setAttribute('aria-pressed', String(t === (switchingToTask ?? current)));
    btn.disabled = busy || Boolean(switchingToTask) || state.calibrating;
  }
  if (!switchingToTask && taskSwitchStatus) taskSwitchStatus.textContent = '';
}

function show(state) {
  showTaskTabs(state);
  const canReset = state.scene_revision !== undefined;
  cubeSettings.hidden = !state.cube_mode && !canReset;
  cubeModeControl.hidden = !state.cube_mode;
  if (!busy && state.cube_mode) cubeMode.value = state.cube_mode;
  cubeMode.disabled = busy || !state.cube_mode || state.calibrating;
  resetScene.disabled = busy || !canReset || state.calibrating;
  if (sceneRevision !== undefined && canReset && state.scene_revision > sceneRevision) {
    sceneMessage.textContent = '场景已初始化';
  }
  sceneRevision = state.scene_revision;
  timing.textContent = tracks(state);
  showWristOffset(state);
  showGrasp(state);
  const mode = state.diagnostic_stale ? '状态待更新' : state.feedback_rearm_required ? '保护停止'
    : state.active ? '输出中' : state.requested ? '待命' : '关闭';
  const lastPause = state.last_feedback_interruption
    ? ` · 中断 ${state.feedback_interruptions} 次 · 上次 ${state.last_feedback_interruption}` : '';
  const handoff = state.feedback_handoffs
    ? ` · 新帧接续 ${state.feedback_handoffs} 次 · 最近超期 ${number(state.last_feedback_handoff_ms, 1)} ms` : '';
  feedbackStatus.textContent = state.diagnostic_stale ? mode : `${mode} · ${state.feedback_message ?? '等待控制进程'}${lastPause}${handoff}`;
  feedbackStatus.dataset.active = state.active ? 'yes' : 'no';
  calibration.textContent = state.demo ? '演示模式 · 无真机输出' : state.calibrating ? '正在标零' : state.zero_error
    ? `标零失败 · ${state.zero_error}` : state.calibrated ? '已标零' : '待标零';
  calibration.title = state.zero_path ?? '';
  const contents = (state.rows ?? []).map(row => {
    const tr = document.createElement('tr');
    const cells = [
      row.id, row.joint, number(row.raw_deg, 2), number(row.input_rad, 4),
      number(row.contact_input_rad, 4), number(row.sim_rad, 4), number(row.sim_minus_input_rad, 4),
      number(row.contact_motor_nm, 3), number(row.position_motor_nm, 3),
      number(row.nominal_nm, 3),
      number(row.sent_motor_nm, 3), number(row.current_a, 3),
      number(row.age_ms, 1), row.state,
    ];
    for (const value of cells) {
      const td = document.createElement('td');
      td.textContent = value;
      tr.append(td);
    }
    return tr;
  });
  rows.replaceChildren(...contents);
  for (const button of buttons) {
    const action = button.dataset.control;
    button.disabled = busy || state.calibrating ||
      (action === 'enable' && (!state.can_feedback || state.requested || state.diagnostic_stale)) ||
      (action === 'disable' && !(state.requested || state.active || state.diagnostic_stale)) ||
      (action === 'align' && !state.can_align) || (action === 'zero' && state.can_zero === false);
  }
}

async function poll() {
  try {
    const response = await fetch('/api/v1/control', {signal:AbortSignal.timeout(900)});
    if (!response.ok) throw Error(response.status);
    show(await response.json());
  } catch {
    if (switchingToTask) {
      if (taskSwitchStatus) taskSwitchStatus.textContent = '正在加载新场景，请稍候…';
    } else {
      feedbackStatus.textContent = '网页控制连接中断';
      feedbackStatus.dataset.active = 'no';
    }
    for (const button of buttons) button.disabled = true;
    for (const btn of taskButtons) btn.disabled = true;
    cubeMode.disabled = resetScene.disabled = true;
    for (const input of graspInputs) input.disabled = true;
    graspHome.disabled = record.disabled = true;
  } finally {
    // 头显帧交换走独立请求，控制面板无需每帧重绘。
    timer = setTimeout(poll, switchingToTask ? 220 : (document.visibilityState === 'hidden' ? 650 : 220));
  }
}

async function postCommand(command, message) {
  message.textContent = '';
  try {
    const reply = await fetch('/api/v1/control', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify(command), signal:AbortSignal.timeout(1200),
    });
    if (!reply.ok) throw Error(reply.status);
    return true;
  } catch {
    message.textContent = '操作未提交，请检查连接后重试。';
    return false;
  }
}

async function submit(command, message) {
  busy = true;
  for (const button of buttons) button.disabled = true;
  cubeMode.disabled = resetScene.disabled = true;
  try {
    return await postCommand(command, message);
  } finally {
    busy = false;
  }
}

for (const button of buttons) {
  button.addEventListener('click', () => {
    const action = button.dataset.control;
    if (action === 'zero' && !confirm('将关闭真机出力并重录零位。请保持手套在参考姿态。')) return;
    submit({action}, commandMessage);
  });
}
cubeMode.addEventListener('change', () => submit({action:'cube_mode', mode:cubeMode.value}, sceneMessage));
resetScene.addEventListener('click', () => submit({action:'reset_scene'}, sceneMessage));
for (const input of graspInputs) {
  input.addEventListener('input', () => {
    graspValue(input);
    if (input === rotationInput && !switchingObject) rotation.update(graspKind.value, Number(input.value));
    if (input === sizeInput && !switchingObject && !sizeCancelled) sizePreview.update(graspKind.value, Number(input.value));
  });
  input.addEventListener('change', async () => {
    const value = input.type === 'checkbox' ? input.checked : input.type === 'range' ? Number(input.value) : input.value;
    if (input === rotationInput) {
      if (!switchingObject) rotation.update(graspKind.value, value, true);
      return;
    }
    const command = {action: 'configure_grasp', values: {[input.dataset.grasp]: value}};
    if (input === sizeInput) {
      if (switchingObject || sizeCancelled) return;
      const target = sizePreview.update(graspKind.value, value);
      sizeCommit = sizeCommit.then(async () => {
        if (sizePreview.target !== target) return;
        const ok = await submit({...command, expected_kind: target.kind}, graspMessage);
        sizePreview.submitted(target, ok);
      });
      return;
    }
    if (input !== graspKind) {
      await submit(command, graspMessage);
      return;
    }
    switchingObject = true;
    sizePreview.clear();
    rotationInput.disabled = graspKind.disabled = true;
    try {
      await Promise.all([rotation.cancel(), sizeCommit]);
      await submit(command, graspMessage);
    } finally {
      switchingObject = false;
    }
  });
}
sizeInput.addEventListener('pointerdown', () => { sizeCancelled = false; });
sizeInput.addEventListener('pointercancel', () => {
  sizeCancelled = true;
  sizePreview.clear();
});
sizeInput.addEventListener('keydown', event => {
  sizeCancelled = event.key === 'Escape';
  if (sizeCancelled) sizePreview.clear();
});
graspHome.addEventListener('click', () => submit({action: 'home_grasp'}, graspMessage));
demoGrip.addEventListener('change', () => submit({action: 'demo_grip', value: Number(demoGrip.value)}, graspMessage));
demoAuto.addEventListener('click', () => submit({action: 'demo_grip', value: null}, graspMessage));
record.addEventListener('click', () => submit({action: record.dataset.recording === 'yes'
  ? 'stop_recording' : 'start_recording'}, graspMessage));
if (offsetPanel) {
  offsetStart.addEventListener('click', () => submit({action: 'start_overlay_cal'}, offsetStatus));
  offsetCancel.addEventListener('click', () => submit({action: 'cancel_overlay_cal'}, offsetStatus));
  offsetReset.addEventListener('click', () => submit({action: 'reset_offset_cal', auto_save: true}, offsetStatus));
}
for (const btn of taskButtons) {
  btn.addEventListener('click', async () => {
    const target = btn.dataset.task;
    if (!target || target === loadedTask || switchingToTask) return;
    switchingToTask = target;
    for (const b of taskButtons) {
      b.disabled = true;
      b.setAttribute('aria-pressed', String(b.dataset.task === target));
    }
    if (taskSwitchStatus) taskSwitchStatus.textContent = `正在切换到「${btn.textContent}」（手套保持在线与标零）…`;
    const ok = await postCommand({action: 'switch_task', task: target}, taskSwitchStatus ?? commandMessage);
    if (!ok) {
      switchingToTask = null;
      for (const b of taskButtons) b.disabled = false;
    }
  });
}

window.addEventListener('keydown', event => {
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) return;
  const tag = event.target?.tagName;
  if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
  if (event.key === 'f' || event.key === 'F') {
    event.preventDefault();
    submit({action: 'toggle_feedback'}, commandMessage);
  } else if (event.code === 'Space') {
    event.preventDefault();
    submit({action: 'disable'}, commandMessage);
  } else if ((event.key === 'r' || event.key === 'R') && !resetScene.disabled) {
    event.preventDefault();
    submit({action: 'reset_scene'}, sceneMessage);
  }
});

if (panel) poll();
window.addEventListener('pagehide', () => {
  clearTimeout(timer);
  rotation.cancel();
  sizePreview.clear();
});
