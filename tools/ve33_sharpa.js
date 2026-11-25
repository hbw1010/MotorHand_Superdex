const taskButtons = Array.from(document.querySelectorAll('#task-tabs [data-task]'));
const handModelButtons = Array.from(document.querySelectorAll('#hand-model-tabs [data-hand-model]'));
const taskSwitchStatus = document.querySelector('#task-switch-status');
const statusText = document.querySelector('#sharpa-status-text');
const badges = document.querySelector('#sharpa-badges');
const msgEl = document.querySelector('#sharpa-message');
const gloveHeading = document.querySelector('#sharpa-glove-heading');
const demoAxesCard = document.querySelector('#sharpa-demo-axes-card');
const demoAxesTitle = document.querySelector('#sharpa-demo-axes-title');
const demoAxesMode = document.querySelector('#sharpa-demo-axes-mode');
const demoAxesGrid = document.querySelector('#sharpa-demo-axes-grid');
const demoPresetButtons = Array.from(document.querySelectorAll('[data-sharpa-preset]'));

const btnStop = document.querySelector('#sharpa-stop');
const handSel = document.querySelector('#sharpa-hand');
const hardnessInput = document.querySelector('#sharpa-hardness');
const hardnessOut = document.querySelector('#sharpa-hardness-val');
const rateInput = document.querySelector('#sharpa-rate');

// 真机控制区
const btnDiscover = document.querySelector('#sharpa-discover');
const btnClearLatch = document.querySelector('#sharpa-clear-latch');
const btnInspect = document.querySelector('#sharpa-inspect');
const serialInput = document.querySelector('#sharpa-serial');
const fwInput = document.querySelector('#sharpa-firmware');
const realModeSel = document.querySelector('#sharpa-real-mode');
const skipHomeChk = document.querySelector('#sharpa-skip-home');
const executeChk = document.querySelector('#sharpa-execute');
const btnStartReal = document.querySelector('#sharpa-start-real');

// 离线仿真区
const simModeSel = document.querySelector('#sharpa-sim-mode');
const withObjChk = document.querySelector('#sharpa-with-object');
const simFeedbackChk = document.querySelector('#sharpa-sim-feedback');
const btnStartSim = document.querySelector('#sharpa-start-sim');
const btnSimToReal = document.querySelector('#sharpa-sim-to-real');

// 数据录制 / 回放 / 接管区
const dataTargetSel = document.querySelector('#sharpa-data-target');
const trajSel = document.querySelector('#sharpa-trajectory');
const btnRefreshRec = document.querySelector('#sharpa-refresh-rec');
const btnRecord = document.querySelector('#sharpa-btn-record');
const btnRecordRo = document.querySelector('#sharpa-btn-record-ro');
const btnReplay = document.querySelector('#sharpa-btn-replay');
const takeoverBtn = document.querySelector('#sharpa-takeover-toggle');

const jointsBody = document.querySelector('#sharpa-joints-body');
const gloveBody = document.querySelector('#sharpa-glove-body');
const logsPre = document.querySelector('#sharpa-logs');

let sharpaBusy = false;
let switchingToTask = null;
let loadedHandModel = null;
let switchingToHandModel = null;
let lastState = null;
let knownRecKey = '';
let configApplied = false;
let timer = 0;
let demoAxesDof = 0;
let demoAxisInputs = [];
let demoAxesValues = [];
let demoAxesSendTimer = 0;

const JOINT_LABELS = {
  thumb_cmc_flex: '拇指腕掌屈伸', thumb_cmc_abd: '拇指腕掌外展', thumb_mcp: '拇指掌指', thumb_ip: '拇指指间',
  index_mcp_flex: '食指掌指屈伸', index_pip: '食指近端联动', middle_mcp_flex: '中指掌指屈伸', middle_pip: '中指近端联动',
  ring_mcp_flex: '无名指掌指屈伸', ring_pip: '无名指近端联动', pinky_mcp_flex: '小指掌指屈伸', pinky_pip: '小指近端联动',
  index_mcp_abd: '食指侧摆', middle_mcp_abd: '中指侧摆', ring_mcp_abd: '无名指侧摆', pinky_mcp_abd: '小指侧摆',
};

const fmt = (v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : '—');
const fmtHardness = h => {
  const kp = Math.min(80, (5 + 55 * h) * 2.7);
  return `${h.toFixed(2)} (Kp≈${kp.toFixed(1)})`;
};

function makeBadge(text, cls = '') {
  const span = document.createElement('span');
  span.className = `sharpa-badge ${cls}`.trim();
  span.textContent = text;
  return span;
}

function applyDefaults(cfg) {
  if (configApplied || !cfg) return;
  configApplied = true;
  if (cfg.default_mode && ['force-contact', 'force-linear', 'teleop'].includes(cfg.default_mode)) {
    realModeSel.value = cfg.default_mode;
  }
  if (cfg.default_hand) handSel.value = cfg.default_hand;
  if (cfg.default_serial && !serialInput.value) serialInput.value = cfg.default_serial;
  if (cfg.approved_firmware) fwInput.value = cfg.approved_firmware;
  if (Number.isFinite(cfg.default_hardness)) {
    hardnessInput.value = String(cfg.default_hardness);
    hardnessOut.textContent = fmtHardness(Number(cfg.default_hardness));
  }
  if (Number.isFinite(cfg.default_rate_hz)) rateInput.value = String(cfg.default_rate_hz);
  if (typeof cfg.skip_glove_home_default === 'boolean') skipHomeChk.checked = cfg.skip_glove_home_default;
  if (typeof cfg.sim_with_object_default === 'boolean') withObjChk.checked = cfg.sim_with_object_default;
}

function updateRecordings(recordings) {
  if (!Array.isArray(recordings)) return;
  const key = recordings.map(r => `${r.name}:${r.size_kb}`).join('|');
  if (key === knownRecKey) return;
  knownRecKey = key;
  const prev = trajSel.value;
  trajSel.replaceChildren();
  if (!recordings.length) {
    trajSel.append(new Option('（暂无已录制轨迹）', ''));
    return;
  }
  for (const r of recordings) {
    trajSel.append(new Option(`${r.name} (${r.size_kb} KB · ${r.mtime})`, r.path));
  }
  if (prev && [...trajSel.options].some(o => o.value === prev)) trajSel.value = prev;
}

function render(s) {
  if (!s) return;
  lastState = s;
  applyDefaults(s.web_config);
  const running = Boolean(s.running);
  const runningSim = Boolean(s.running_simulated);
  const frame = s.frame;

  if (running) {
    const kind = runningSim ? '【离线仿真】' : '【Sharpa 真机】';
    const role = s.can_takeover ? (s.takeover ? '人手实时接管中' : '数据回放中') : s.mode;
    const recTag = s.active_output ? ' · 正在录制' : '';
    statusText.textContent = `${kind}${role}${recTag}（PID ${s.pid}）`;
    statusText.style.color = runningSim ? '#9ec5ff' : (s.takeover || !s.can_takeover ? '#8fe7cf' : '#f4ca89');
  } else if (s.last_error) {
    statusText.textContent = `已停止 · ${s.last_error}`;
    statusText.style.color = '#f6a497';
  } else {
    statusText.textContent = s.binary_ready ? '待命 · 可选择启动离线仿真、启动真机或数据录制/回放' : '缺少 motorhand_sharpa 编译产物';
    statusText.style.color = s.binary_ready ? '#a6bfce' : '#f6a497';
  }

  const modelId = s.hand_model || '12dof';
  const dof = s.dof || 12;
  const items = [];
  items.push(makeBadge(modelId === '16dof' ? '型号: 16-DOF (Sharpa 全 22 轴自由)' : '型号: 12-DOF (VE40)', 'ok'));
  items.push(makeBadge(s.binary_ready ? 'Sharpa SDK 5.0.11 已就绪' : '未编译真机二进制', s.binary_ready ? 'ok' : 'err'));
  items.push(makeBadge(s.glove_port ? `手套串口 ${s.glove_port.split('/').pop()}` : '手套：演示/无串口', s.glove_port ? 'ok' : 'warn'));
  if (frame) {
    items.push(makeBadge(frame.simulated ? '当前：离线仿真' : `当前：真机 ${frame.serial || ''}`, frame.simulated ? 'warn' : 'ok'));
    items.push(makeBadge(frame.enabled ? 'Sharpa 已使能' : 'Sharpa 未使能', frame.enabled ? 'ok' : 'warn'));
    if (frame.fault) items.push(makeBadge(`Sharpa 故障 ${frame.fault}`, 'err'));
    if (frame.feedback_mode) {
      const fbLabel = frame.feedback_mode === 'driver-impedance' ? '位置阻抗(Pos/Kp/Kd)'
        : frame.feedback_mode === 'torque' ? '纯力矩前馈' : '力反馈关闭';
      items.push(makeBadge(`机制: ${fbLabel}`, frame.feedback_mode === 'driver-impedance' ? 'ok' : ''));
    }
    items.push(makeBadge(frame.force_commanded ? '手套力反馈输出中' : '手套无出力', frame.force_commanded ? 'ok' : ''));
    if (Number.isFinite(s.frame_age_ms)) items.push(makeBadge(`延迟 ${s.frame_age_ms.toFixed(0)} ms`));
    if (frame.load_source) items.push(makeBadge(`负载源: ${frame.load_source}`));
    if (frame.feedback_window) {
      const fw = frame.feedback_window;
      items.push(makeBadge(`几何 ${fw['几何有效帧'] ?? 0}/${fw['接管帧'] ?? 0} · 峰值轴 J${fw['全手峰值轴'] ?? '—'}`));
    }
  }
  if (s.active_output) items.push(makeBadge(`录制中: ${s.active_output.split('/').pop()}`, 'warn'));
  badges.replaceChildren(...items);
  if (gloveHeading) {
    gloveHeading.textContent = dof === 16
      ? 'MotorHand 16 轴位置阻抗力反馈（含 J13–J16 四指侧摆 · MIT Pos / Kp / Kd）'
      : 'MotorHand 12 轴位置阻抗力反馈（MIT Pos / Kp / Kd）';
  }
  renderDemoAxes(s);

  updateRecordings(s.recordings);
  if (s.discovered_devices?.length && !serialInput.value) {
    serialInput.value = s.discovered_devices[0].serial || '';
  }

  // 真机按钮：即使仿真正在运行，也允许直接点「启动真机」一键停仿真切入真机！
  const allowStartReal = !sharpaBusy && s.binary_ready && (!running || runningSim);
  btnDiscover.disabled = sharpaBusy || running || !s.binary_ready;
  btnClearLatch.disabled = sharpaBusy || running || !s.binary_ready;
  btnInspect.disabled = sharpaBusy || running || !s.binary_ready;
  btnStartReal.disabled = !allowStartReal;
  btnStartReal.textContent = runningSim ? '停止仿真并启动真机' : '启动真机';

  // 仿真按钮 & 仿真启动后的真机切换按钮
  btnStartSim.disabled = sharpaBusy || running || !s.sim_binary_ready;
  btnSimToReal.hidden = !runningSim;
  btnSimToReal.disabled = !allowStartReal;

  btnStop.disabled = sharpaBusy || !running;
  handSel.disabled = serialInput.disabled = fwInput.disabled = realModeSel.disabled = simModeSel.disabled = running && !runningSim;

  // 数据录制 / 数据回放 / 接管按钮
  const isRecording = running && Boolean(s.active_output);
  btnRecord.disabled = sharpaBusy || (running && !isRecording) || !s.sim_binary_ready;
  btnRecord.textContent = isRecording ? '停止录制并保存' : '数据录制（遥操并保存 .jsonl）';
  btnRecordRo.disabled = sharpaBusy || running || !s.binary_ready;
  btnReplay.disabled = sharpaBusy || running || !s.sim_binary_ready || !trajSel.value;

  const canTakeover = Boolean(s.can_takeover);
  const taken = Boolean(s.takeover);
  takeoverBtn.disabled = sharpaBusy || !canTakeover;
  takeoverBtn.dataset.active = taken ? 'yes' : 'no';
  takeoverBtn.textContent = !canTakeover
    ? '人手接管 Sharpa（回放中可用）'
    : taken ? '接管中 · 点击恢复回放 (Space)' : '人手接管 Sharpa (Space)';

  if (frame && Array.isArray(frame.joint_names)) {
    const pos = frame.position_rad || [];
    const tgt = frame.hand_target_rad || [];
    const loads = frame.load_nm || [];
    const contacts = frame.contact || [];
    const trs = frame.joint_names.map((name, idx) => {
      const tr = document.createElement('tr');
      const load = Number.isFinite(loads[idx]) ? loads[idx] : null;
      const pct = load === null ? 0 : Math.min(100, Math.abs(load) / 0.5 * 100);
      const tdIdx = document.createElement('td'); tdIdx.textContent = String(idx + 1);
      const tdName = document.createElement('td'); tdName.textContent = name;
      const tdPos = document.createElement('td'); tdPos.textContent = fmt(pos[idx], 3);
      const tdTgt = document.createElement('td'); tdTgt.textContent = fmt(tgt[idx], 3);
      const tdLoad = document.createElement('td');
      const wrap = document.createElement('span'); wrap.className = 'sharpa-bar-wrap';
      const val = document.createElement('span'); val.textContent = load === null ? '—' : load.toFixed(3);
      const bg = document.createElement('span'); bg.className = 'sharpa-bar-bg';
      const fill = document.createElement('span');
      fill.className = `sharpa-bar-fill${pct > 55 ? ' hot' : ''}`;
      fill.style.width = `${pct.toFixed(0)}%`;
      bg.append(fill); wrap.append(val, bg); tdLoad.append(wrap);
      const tdContact = document.createElement('td');
      tdContact.textContent = contacts[idx] ? '接触' : '—';
      if (contacts[idx]) tdContact.style.color = '#f4ca89';
      tr.append(tdIdx, tdName, tdPos, tdTgt, tdLoad, tdContact);
      return tr;
    });
    jointsBody.replaceChildren(...trs);
  }

  if (frame && Array.isArray(frame.glove_motor_rad)) {
    const gNm = Array.isArray(frame.glove_nominal_nm) ? frame.glove_nominal_nm : [];
    const gTgt = Array.isArray(frame.glove_driver_target_rad) ? frame.glove_driver_target_rad
      : Array.isArray(frame.glove_motor_target_rad) ? frame.glove_motor_target_rad : [];
    const gKp = Array.isArray(frame.glove_driver_kp) ? frame.glove_driver_kp : [];
    const gKd = Array.isArray(frame.glove_driver_kd) ? frame.glove_driver_kd : [];
    const gJoints = Array.isArray(s.demo_axes_joints) ? s.demo_axes_joints : [];
    const trs = frame.glove_motor_rad.map((rad, idx) => {
      const tr = document.createElement('tr');
      const kp = gKp[idx];
      const jname = gJoints[idx] ? ` · ${JOINT_LABELS[gJoints[idx]] || gJoints[idx]}` : '';
      const cells = [
        `J${idx + 1}${jname}`,
        fmt(rad, 3),
        fmt(gTgt[idx], 3),
        fmt(kp, 1),
        fmt(gKd[idx], 2),
        fmt(gNm[idx], 2),
      ];
      cells.forEach((txt, cidx) => {
        const td = document.createElement('td');
        td.textContent = txt;
        if (cidx === 0) td.style.textAlign = 'left';
        if (cidx === 0 && idx >= 12) td.style.color = '#86deed';
        if (cidx === 3 && Number.isFinite(kp) && kp > 0.5) td.style.color = '#8fe7cf';
        tr.append(td);
      });
      return tr;
    });
    gloveBody.replaceChildren(...trs);
  }

  if (Array.isArray(s.logs)) {
    logsPre.textContent = s.logs.length ? s.logs.join('\n') : '尚未启动 Sharpa 会话。';
    logsPre.scrollTop = logsPre.scrollHeight;
  }
}

async function sendSharpaDemoAxes(active, values) {
  try {
    await fetch('/api/v1/demo-axes', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(active ? {active: true, values} : {active: false}),
      signal: AbortSignal.timeout(1500),
    });
  } catch {
    // ignore transient poll overlap
  }
}

function scheduleSharpaDemoAxesSend() {
  clearTimeout(demoAxesSendTimer);
  demoAxesSendTimer = setTimeout(() => sendSharpaDemoAxes(true, [...demoAxesValues]), 25);
}

function buildSharpaPreset(name, dof) {
  const v = new Array(dof).fill(0);
  if (name === 'open') return v;
  if (name === 'fist') {
    const base = [0.65, 0.90, 0.65, 0.70, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05, 1.05];
    for (let i = 0; i < 12; i++) v[i] = base[i];
    return v;
  }
  if (name === 'spread_out' && dof === 16) {
    v[12] = 0.30; v[13] = 0.10; v[14] = -0.10; v[15] = -0.30;
    return v;
  }
  if (name === 'spread_in' && dof === 16) {
    v[12] = -0.18; v[13] = -0.04; v[14] = 0.06; v[15] = 0.18;
    return v;
  }
  if (name === 'pinch_index') {
    v[0] = 0.75; v[1] = 0.95; v[2] = 0.55; v[3] = 0.45; v[4] = 0.95; v[5] = 0.85;
    return v;
  }
  if (name === 'pinch_middle') {
    v[0] = 0.90; v[1] = 1.15; v[2] = 0.65; v[3] = 0.55; v[6] = 1.00; v[7] = 0.90;
    return v;
  }
  return v;
}

function renderDemoAxes(s) {
  if (!demoAxesCard) return;
  const dof = s.dof || 12;
  const canDrive = Boolean(s.running && s.running_simulated && (s.mode === 'sim' || s.mode === 'takeover'));
  demoAxesCard.hidden = !canDrive;
  if (!canDrive) return;
  if (demoAxesTitle) {
    demoAxesTitle.textContent = dof === 16
      ? '全 16 轴交互仿真控制（含 J13–J16 四指侧摆 → 驱动 Sharpa 全 22 轴）'
      : '全 12 轴交互仿真控制（假手套测试）';
  }
  if (demoAxesMode) {
    demoAxesMode.textContent = s.demo_axes_active ? '手动全轴控制中' : '自动演示波形中';
    demoAxesMode.style.color = s.demo_axes_active ? '#8fe7cf' : '#a6bfce';
  }
  for (const btn of demoPresetButtons) {
    btn.hidden = btn.dataset['16Only'] === '1' && dof !== 16;
    if (btn.dataset.sharpaPreset === 'auto') btn.classList.toggle('active', !s.demo_axes_active);
  }
  const limits = Array.isArray(s.demo_axes_limits) && s.demo_axes_limits.length === dof
    ? s.demo_axes_limits
    : Array.from({length: dof}, (_, i) => (i >= 12 ? [-0.61, 0.61] : [0, 1.57]));
  const joints = Array.isArray(s.demo_axes_joints) && s.demo_axes_joints.length === dof
    ? s.demo_axes_joints
    : Array.from({length: dof}, (_, i) => `J${i + 1}`);
  if (demoAxesDof !== dof) {
    demoAxesDof = dof;
    demoAxesValues = new Array(dof).fill(0);
    demoAxisInputs = [];
    demoAxesGrid.replaceChildren();
    for (let i = 0; i < dof; i++) {
      const [lo, hi] = limits[i];
      const item = document.createElement('div');
      item.className = `demo-axis-item${i >= 12 ? ' abd-axis' : ''}`;
      const head = document.createElement('div');
      head.className = 'demo-axis-head';
      const title = document.createElement('strong');
      title.textContent = `J${i + 1} · ${JOINT_LABELS[joints[i]] || joints[i]}`;
      const out = document.createElement('output');
      out.textContent = '0.0°';
      head.append(title, out);
      const input = document.createElement('input');
      input.type = 'range';
      input.min = String(lo);
      input.max = String(hi);
      input.step = '0.01';
      input.value = '0';
      input.addEventListener('input', () => {
        const val = Number(input.value);
        demoAxesValues[i] = val;
        out.textContent = `${(val * 180 / Math.PI).toFixed(1)}°`;
        scheduleSharpaDemoAxesSend();
      });
      item.append(head, input);
      demoAxesGrid.append(item);
      demoAxisInputs.push({input, out, lo, hi});
    }
  }
  const srcVals = s.demo_axes_active && Array.isArray(s.demo_axes)
    ? s.demo_axes
    : (Array.isArray(s.frame?.glove_motor_rad) ? s.frame.glove_motor_rad : []);
  for (let i = 0; i < demoAxisInputs.length; i++) {
    const {input, out} = demoAxisInputs[i];
    if (input.matches(':active')) continue;
    const val = Number.isFinite(srcVals[i]) ? srcVals[i] : 0;
    demoAxesValues[i] = val;
    input.value = String(val);
    out.textContent = `${(val * 180 / Math.PI).toFixed(1)}°`;
  }
}

async function postSharpa(payload) {
  sharpaBusy = true;
  msgEl.textContent = '';
  try {
    const res = await fetch('/api/v1/sharpa', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload),
      signal: AbortSignal.timeout(15000),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      msgEl.textContent = data.error || `请求失败 (${res.status})`;
      if (data.status) render(data.status);
      return null;
    }
    if (data.status) render(data.status);
    return data;
  } catch (err) {
    msgEl.textContent = `Sharpa 控制通信失败：${err.message || err}`;
    return null;
  } finally {
    sharpaBusy = false;
  }
}

async function startRealHardware({recordOutput = false} = {}) {
  const mode = realModeSel.value;
  const execute = Boolean(executeChk.checked);
  if (!execute) {
    msgEl.textContent = '请先勾选「确认允许真机出力 (--execute)」后再启动真机。';
    return;
  }
  const switching = Boolean(lastState?.running_simulated);
  const title = switching ? '停止当前离线仿真并启动 Sharpa 真机' : '启动 Sharpa 真机';
  if (!confirm(`即将${title}（模式：${mode}，SN：${serialInput.value || '未填'}${recordOutput ? '，同时录制轨迹' : ''}）。\n请确认机械手与手套周围安全，随时可按 Esc 或点击红色按钮安全停机。`)) {
    return;
  }
  await postSharpa({
    action: 'start',
    mode,
    hand: handSel.value,
    serial: serialInput.value,
    firmware: fwInput.value,
    hardness: Number(hardnessInput.value),
    rate: Number(rateInput.value),
    record_output: recordOutput,
    skip_glove_home: Boolean(skipHomeChk.checked),
    execute: true,
    feedback: mode === 'force-contact' || mode === 'force-linear',
    replace_running: switching,
  });
}

async function startSimulation({recordOutput = false} = {}) {
  const mode = simModeSel.value;
  const feedback = mode === 'sim-teleop' && Boolean(simFeedbackChk.checked);
  const execute = feedback ? Boolean(executeChk.checked) : false;
  if (feedback && !execute) {
    msgEl.textContent = '仿真时开启手套真机力反馈需要勾选「确认允许真机出力 (--execute)」。';
    return;
  }
  await postSharpa({
    action: 'start',
    mode,
    hand: handSel.value,
    hardness: Number(hardnessInput.value),
    rate: Number(rateInput.value),
    record_output: recordOutput,
    with_object: Boolean(withObjChk.checked),
    feedback,
    execute,
  });
}

async function poll() {
  try {
    const res = await fetch('/api/v1/control', {signal: AbortSignal.timeout(1200)});
    if (!res.ok) throw Error(res.status);
    const data = await res.json();
    if (data.task && data.task !== 'sharpa_real') {
      window.location.replace(`${window.location.pathname}?t=${Date.now()}`);
      return;
    }
    const curModel = data.hand_model || data.sharpa?.hand_model || '12dof';
    if (!loadedHandModel) loadedHandModel = curModel;
    if (switchingToHandModel && curModel === switchingToHandModel) {
      window.location.replace(`${window.location.pathname}?m=${curModel}&t=${Date.now()}`);
      return;
    }
    if (!switchingToHandModel && loadedHandModel && curModel !== loadedHandModel) {
      window.location.replace(`${window.location.pathname}?m=${curModel}&t=${Date.now()}`);
      return;
    }
    for (const b of handModelButtons) {
      const m = b.dataset.handModel;
      b.setAttribute('aria-pressed', String(m === (switchingToHandModel ?? curModel)));
      b.disabled = sharpaBusy || Boolean(switchingToTask) || Boolean(switchingToHandModel);
    }
    if (Array.isArray(data.available_tasks)) {
      for (const b of taskButtons) {
        b.hidden = !data.available_tasks.includes(b.dataset.task);
        b.disabled = Boolean(switchingToTask) || Boolean(switchingToHandModel);
      }
    }
    if (data.sharpa) render(data.sharpa);
  } catch {
    if (switchingToTask || switchingToHandModel) {
      taskSwitchStatus.textContent = switchingToHandModel ? '正在切换手部自由度型号并重载 Sharpa 服务，请稍候…' : '正在切换并重新连接手套，请稍候…';
    }
  } finally {
    timer = setTimeout(poll, (switchingToTask || switchingToHandModel) ? 220 : (document.visibilityState === 'hidden' ? 650 : 200));
  }
}

for (const btn of taskButtons) {
  btn.addEventListener('click', async () => {
    const target = btn.dataset.task;
    if (!target || target === 'sharpa_real' || switchingToTask || switchingToHandModel) return;
    switchingToTask = target;
    for (const b of taskButtons) {
      b.disabled = true;
      b.setAttribute('aria-pressed', String(b.dataset.task === target));
    }
    taskSwitchStatus.textContent = `正在安全退出 Sharpa 并切换到「${btn.textContent}」…`;
    try {
      const res = await fetch('/api/v1/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'switch_task', task: target}),
        signal: AbortSignal.timeout(8000),
      });
      if (!res.ok) throw Error(res.status);
    } catch {
      switchingToTask = null;
      taskSwitchStatus.textContent = '切换失败，请重试。';
      for (const b of taskButtons) b.disabled = false;
    }
  });
}

for (const btn of handModelButtons) {
  btn.addEventListener('click', async () => {
    const target = btn.dataset.handModel;
    if (!target || target === loadedHandModel || switchingToHandModel || switchingToTask) return;
    switchingToHandModel = target;
    for (const b of handModelButtons) {
      b.disabled = true;
      b.setAttribute('aria-pressed', String(b.dataset.handModel === target));
    }
    taskSwitchStatus.textContent = `正在切换手部型号至 ${btn.textContent}（自动停机并重载 3D 与重定向）…`;
    try {
      const res = await fetch('/api/v1/hand-model', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({hand_model: target}),
        signal: AbortSignal.timeout(8000),
      });
      if (!res.ok) throw Error(res.status);
    } catch {
      switchingToHandModel = null;
      taskSwitchStatus.textContent = '手部型号切换失败，请重试。';
      for (const b of handModelButtons) b.disabled = false;
    }
  });
}

for (const btn of demoPresetButtons) {
  btn.addEventListener('click', async () => {
    const preset = btn.dataset.sharpaPreset;
    if (preset === 'auto') {
      await sendSharpaDemoAxes(false, null);
      return;
    }
    const dof = demoAxesDof || (lastState?.dof ?? 12);
    const vals = buildSharpaPreset(preset, dof);
    for (let i = 0; i < demoAxisInputs.length; i++) {
      const {input, out, lo, hi} = demoAxisInputs[i];
      const clamped = Math.max(lo, Math.min(hi, vals[i]));
      demoAxesValues[i] = clamped;
      input.value = String(clamped);
      out.textContent = `${(clamped * 180 / Math.PI).toFixed(1)}°`;
    }
    await sendSharpaDemoAxes(true, [...demoAxesValues]);
  });
}

hardnessInput.addEventListener('input', () => {
  hardnessOut.textContent = fmtHardness(Number(hardnessInput.value));
});
btnRefreshRec.addEventListener('click', () => postSharpa({action: 'refresh'}));

btnDiscover.addEventListener('click', async () => {
  msgEl.textContent = '正在扫描 Sharpa 真机…';
  const data = await postSharpa({action: 'discover'});
  if (data?.devices) {
    if (data.devices.length) {
      const d = data.devices[0];
      serialInput.value = d.serial || '';
      if (d.hand === 'left' || d.hand === 'right') handSel.value = d.hand;
      if (d.firmware) fwInput.value = d.firmware;
      msgEl.textContent = `已发现 ${data.devices.length} 台真机：${d.serial} (${d.pn || ''} · 固件 ${d.firmware || ''})`;
    } else {
      msgEl.textContent = '未发现 Sharpa 真机（请检查网线/供电，或点击右侧「启动仿真」测试）。';
    }
  }
});

btnClearLatch.addEventListener('click', async () => {
  msgEl.textContent = '正在清除网线断连锁存 (0x0907)…';
  const data = await postSharpa({action: 'clear_link_latch', serial: serialInput.value});
  if (data?.result) {
    msgEl.textContent = data.result.has_fault
      ? `已清 0x0907，但仍有其他故障：${data.result.fault_details}`
      : '已清除网线锁存 (0x0907)，设备当前无报警！';
  }
});

btnInspect.addEventListener('click', async () => {
  msgEl.textContent = '正在执行 Sharpa 22 轴只读体检…';
  const data = await postSharpa({
    action: 'inspect',
    serial: serialInput.value,
    hand: handSel.value,
    firmware: fwInput.value,
  });
  if (data?.inspect) {
    const info = data.inspect;
    msgEl.textContent = `体检通过：${info.serial} · 固件 ${info.firmware} · 使能=${info.enabled ? '是' : '否'} · 故障=${info.fault}`;
  }
});

btnStartReal.addEventListener('click', () => startRealHardware({recordOutput: false}));
btnSimToReal.addEventListener('click', () => startRealHardware({recordOutput: false}));
btnStartSim.addEventListener('click', () => startSimulation({recordOutput: false}));
btnStop.addEventListener('click', () => postSharpa({action: 'stop'}));

btnRecord.addEventListener('click', async () => {
  if (lastState?.running && lastState?.active_output) {
    await postSharpa({action: 'stop'});
    msgEl.textContent = '录制已停止并保存到轨迹列表。';
    return;
  }
  if (dataTargetSel.value === 'sim') {
    await startSimulation({recordOutput: true});
  } else {
    await startRealHardware({recordOutput: true});
  }
});

btnRecordRo.addEventListener('click', async () => {
  if (!serialInput.value.trim()) {
    msgEl.textContent = '只读录制需要填写或发现 Sharpa SN。';
    return;
  }
  await postSharpa({
    action: 'start',
    mode: 'record',
    hand: handSel.value,
    serial: serialInput.value,
    firmware: fwInput.value,
    rate: Number(rateInput.value),
    record_output: true,
    execute: false,
  });
});

btnReplay.addEventListener('click', async () => {
  const traj = trajSel.value;
  if (!traj) {
    msgEl.textContent = '请先录制并选择一条 .jsonl 轨迹。';
    return;
  }
  const isSim = dataTargetSel.value === 'sim';
  if (!isSim) {
    if (!executeChk.checked) {
      msgEl.textContent = '真机数据回放并允许接管前，请先勾选「确认允许真机出力 (--execute)」。';
      return;
    }
    if (!confirm(`即将启动 Sharpa 真机数据回放（支持随时按 Space 人手接管）：\n轨迹：${traj.split('/').pop()}\nSN：${serialInput.value || '未填'}`)) {
      return;
    }
  }
  await postSharpa({
    action: 'start',
    mode: 'takeover',
    simulation: isSim,
    fake_glove: isSim && simModeSel.value === 'sim',
    hand: handSel.value,
    serial: isSim ? '' : serialInput.value,
    firmware: fwInput.value,
    hardness: Number(hardnessInput.value),
    rate: Number(rateInput.value),
    trajectory: traj,
    skip_glove_home: Boolean(skipHomeChk.checked),
    with_object: Boolean(withObjChk.checked),
    execute: !isSim,
    feedback: true,
  });
});

takeoverBtn.addEventListener('click', () => postSharpa({action: 'toggle_takeover'}));

window.addEventListener('keydown', event => {
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) return;
  const tag = event.target?.tagName;
  if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
  if (!lastState?.running) return;
  if (event.code === 'Space' && lastState.can_takeover) {
    event.preventDefault();
    postSharpa({action: 'toggle_takeover'});
  } else if (event.key === 'Escape') {
    event.preventDefault();
    postSharpa({action: 'stop'});
  }
});

poll();
window.addEventListener('pagehide', () => clearTimeout(timer));
