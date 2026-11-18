const panel = document.querySelector('#g4-control');
const rows = document.querySelector('#axes-body');
const feedbackStatus = document.querySelector('#feedback-state');
const timing = document.querySelector('#timing');
const calibration = document.querySelector('#calibration-state');
const commandMessage = document.querySelector('#command-message');
const feedbackGain = document.querySelector('#feedback-gain');
const cubeSettings = document.querySelector('#cube-settings');
const cubeModeControl = document.querySelector('#cube-mode-control');
const cubeMode = document.querySelector('#cube-mode');
const resetScene = document.querySelector('#reset-scene');
const sceneMessage = document.querySelector('#scene-message');
const buttons = Array.from(panel.querySelectorAll('[data-control]'));
let busy = false, timer = 0;
let sceneRevision;

const number = (value, digits) => Number.isFinite(value) ? value.toFixed(digits) : '—';

function tracks(state) {
  return [
    `${state.online ?? 0}/12 轴在线`,
    state.wrist ?? '等待定位',
    `采集 ${number(state.motor_ms, 0)} ms`,
    `仿真使用 ${number(state.applied_ms, 0)} ms`,
    `接触帧 ${number(state.contact_ms, 0)} ms`,
    state.fixed_wrist ? '固定腕部' : `定位 ${number(state.wrist_ms, 0)} ms`,
    `最大角差 ${number(state.max_angle_error_rad, 3)} rad`,
  ].join(' · ');
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
  const mode = state.active ? '输出中' : state.requested ? '待命' : '关闭';
  feedbackStatus.textContent = `${mode} · ${state.feedback_message ?? '等待控制进程'}`;
  feedbackStatus.dataset.active = state.active ? 'yes' : 'no';
  feedbackGain.disabled = busy || !Number.isFinite(state.feedback_gain) || state.calibrating;
  if (!busy && Number.isFinite(state.feedback_gain)) feedbackGain.value = String(state.feedback_gain);
  calibration.textContent = state.calibrating ? '正在标零' : state.zero_error
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
      (action === 'align' && !state.can_align);
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
    feedbackGain.disabled = true;
  } finally {
    // 头显帧交换走独立请求，控制面板无需每帧重绘。
    timer = setTimeout(poll, document.visibilityState === 'hidden' ? 650 : 220);
  }
}

async function submit(command, message) {
  busy = true;
  for (const button of buttons) button.disabled = true;
  cubeMode.disabled = resetScene.disabled = true;
  feedbackGain.disabled = true;
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
feedbackGain.addEventListener('change', () => submit({action:'feedback_gain', value:Number(feedbackGain.value)}, commandMessage));

if (panel) poll();
window.addEventListener('pagehide', () => clearTimeout(timer));
