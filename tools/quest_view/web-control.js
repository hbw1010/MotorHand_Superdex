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
const graspHome = document.querySelector('#grasp-home');
const graspMessage = document.querySelector('#grasp-message');
const demoControls = document.querySelector('#demo-controls');
const demoGrip = document.querySelector('#demo-grip');
const demoAuto = document.querySelector('#demo-auto');
const record = document.querySelector('#grasp-record');
const csvLink = document.querySelector('#grasp-csv');
const buttons = Array.from(panel.querySelectorAll('[data-control]'));
let busy = false, timer = 0;
let sceneRevision;

const number = (value, digits) => Number.isFinite(value) ? value.toFixed(digits) : '—';

function tracks(state) {
  return [
    `${state.online ?? 0}/12 ${state.demo ? '轴演示' : '轴在线'}`,
    state.wrist ?? '等待定位',
    `采集 ${number(state.motor_ms, 0)} ms`,
    `仿真使用 ${number(state.applied_ms, 0)} ms`,
    `接触帧 ${number(state.contact_ms, 0)} ms`,
    state.fixed_wrist ? '固定腕部' : `定位 ${number(state.wrist_ms, 0)} ms`,
    `最大角差 ${number(state.max_angle_error_rad, 3)} rad`,
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
  if (!grasp) return;
  const kindSelect = graspPanel.querySelector('[data-grasp="kind"]');
  if (state.grasp_assets?.length && !kindSelect.querySelector('optgroup')) {
    const group = document.createElement('optgroup');
    group.label = 'SuperDex 官方资产';
    for (const {kind, label} of state.grasp_assets) group.append(new Option(label, kind));
    kindSelect.append(group);
  }
  for (const input of graspInputs) {
    const key = input.dataset.grasp;
    const authored = grasp.asset && (key === 'size_mm' || key === 'hardness');
    input.closest('label').hidden = Boolean(authored);
    input.disabled = busy || state.calibrating || authored || (key === 'layout' && grasp.desk_only);
    if (busy || input.matches(':active')) continue;
    const value = grasp[input.dataset.grasp];
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

function show(state) {
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
  showGrasp(state);
  const mode = state.active ? '输出中' : state.requested ? '待命' : '关闭';
  const lastPause = state.last_feedback_interruption
    ? ` · 中断 ${state.feedback_interruptions} 次 · 上次 ${state.last_feedback_interruption}` : '';
  const handoff = state.feedback_handoffs
    ? ` · 新帧接续 ${state.feedback_handoffs} 次 · 最近超期 ${number(state.last_feedback_handoff_ms, 1)} ms` : '';
  feedbackStatus.textContent = `${mode} · ${state.feedback_message ?? '等待控制进程'}${lastPause}${handoff}`;
  feedbackStatus.dataset.active = state.active ? 'yes' : 'no';
  calibration.textContent = state.demo ? '演示模式 · 无真机输出' : state.calibrating ? '正在标零' : state.zero_error
    ? `标零失败 · ${state.zero_error}` : state.calibrated ? '已标零' : '待标零';
  calibration.title = state.zero_path ?? '';
  const contents = (state.rows ?? []).map(row => {
    const tr = document.createElement('tr');
    const cells = [
      row.id, row.joint, number(row.raw_deg, 2), number(row.input_rad, 4),
      number(row.sim_rad, 4), number(row.contact_motor_nm, 3), number(row.position_motor_nm, 3),
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
      (action === 'enable' && (!state.can_feedback || state.requested)) ||
      (action === 'disable' && !(state.requested || state.active)) ||
      (action === 'align' && !state.can_align) || (action === 'zero' && state.can_zero === false);
  }
}

async function poll() {
  try {
    const response = await fetch('/api/v1/control', {signal:AbortSignal.timeout(900)});
    if (!response.ok) throw Error(response.status);
    show(await response.json());
  } catch {
    feedbackStatus.textContent = '网页控制连接中断';
    feedbackStatus.dataset.active = 'no';
    for (const button of buttons) button.disabled = true;
    cubeMode.disabled = resetScene.disabled = true;
    for (const input of graspInputs) input.disabled = true;
    graspHome.disabled = record.disabled = true;
  } finally {
    // 头显帧交换走独立请求，控制面板无需每帧重绘。
    timer = setTimeout(poll, document.visibilityState === 'hidden' ? 650 : 220);
  }
}

async function submit(command, message) {
  busy = true;
  for (const button of buttons) button.disabled = true;
  cubeMode.disabled = resetScene.disabled = true;
  message.textContent = '';
  try {
    const reply = await fetch('/api/v1/control', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify(command), signal:AbortSignal.timeout(1200),
    });
    if (!reply.ok) throw Error(reply.status);
  } catch {
    message.textContent = '操作未提交，请检查连接后重试。';
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
  input.addEventListener('input', () => graspValue(input));
  input.addEventListener('change', () => {
    const value = input.type === 'checkbox' ? input.checked : input.type === 'range' ? Number(input.value) : input.value;
    submit({action: 'configure_grasp', values: {[input.dataset.grasp]: value}}, graspMessage);
  });
}
graspHome.addEventListener('click', () => submit({action: 'home_grasp'}, graspMessage));
demoGrip.addEventListener('change', () => submit({action: 'demo_grip', value: Number(demoGrip.value)}, graspMessage));
demoAuto.addEventListener('click', () => submit({action: 'demo_grip', value: null}, graspMessage));
record.addEventListener('click', () => submit({action: record.dataset.recording === 'yes'
  ? 'stop_recording' : 'start_recording'}, graspMessage));

if (panel) poll();
window.addEventListener('pagehide', () => clearTimeout(timer));
