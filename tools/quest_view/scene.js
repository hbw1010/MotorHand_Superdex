import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {ForceDiagnostics, LeftHandView} from './diagnostics.js';
import {LocalGraspView} from './local-grasp.js';
import './web-control.js';

const button = document.querySelector('#start');
const status = document.querySelector('#status');
const sourceSelect = document.querySelector('#source');
const preview = document.querySelector('#preview');
const forceScale = document.querySelector('#force-scale');
const showDiagnostics = document.querySelector('#show-diagnostics');
const scaleValue = document.querySelector('#scale-value');
const previewColor = new THREE.Color('#182331');
const scene = new THREE.Scene();
const world = new THREE.Group();
scene.add(world);
const camera = new THREE.PerspectiveCamera(60, 1, .01, 20);
const groups = {right_hand: new Map(), puzzle_cube: new Map(), objects: new Map()};
const xrBasis = new THREE.Matrix4().set(0,-1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1);
let renderer, session, space, latest, align = false, sending = false, seq = 0, lastSend = 0;
let previewPlaced = false;
let mode = 'controller';
let diagnostics, leftView, graspView, controls, shown, lastPoll = 0, polling = false;
const wristJoint = 'wrist';
const leftJointNames = [wristJoint, 'thumb-metacarpal', 'thumb-phalanx-distal', 'thumb-tip'];
for (const finger of ['index', 'middle', 'ring', 'pinky']) {
  leftJointNames.push(`${finger}-finger-phalanx-proximal`, `${finger}-finger-phalanx-distal`, `${finger}-finger-tip`);
}

forceScale.addEventListener('input', () => {
  scaleValue.textContent = `${forceScale.value} N`;
  shown = null;
});

function isController(s, handedness) {
  return s.handedness === handedness && s.targetRayMode === 'tracked-pointer' && s.gripSpace && !s.hand &&
    !s.profiles.some(profile => profile.includes('hand-select') || profile.startsWith('generic-hand'));
}

function leftInput(frame, ref, sources) {
  const left = sources.find(source => source.handedness === 'left' && source.hand);
  if (left) {
    const points = [];
    for (const joint of leftJointNames) {
      const space = left.hand.get(joint);
      const pose = space && frame.getJointPose(space, ref);
      if (!pose || pose.emulatedPosition) return null;
      const p = pose.transform.position;
      points.push([p.x, p.y, p.z]);
    }
    return {kind:'hand', points};
  }
  const controller = sources.find(source => isController(source, 'left'));
  const pose = controller && frame.getPose(controller.gripSpace, ref);
  if (!pose || pose.emulatedPosition) return null;
  const p = pose.transform.position;
  return {kind:'controller', points:[[p.x, p.y, p.z]]};
}

function applyPose(node, pose) {
  node.position.fromArray(pose);
  node.quaternion.fromArray(pose, 3);
}

function placePreview(viewer) {
  const heading = new THREE.Quaternion();
  const p = viewer.transform.position;
  const q = viewer.transform.orientation;
  const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(heading.set(q.x,q.y,q.z,q.w));
  const yaw = Math.atan2(-forward.x, -forward.z);
  heading.setFromAxisAngle(new THREE.Vector3(0,1,0), yaw);
  world.position.set(0, -.14, -.40).applyQuaternion(heading).add(new THREE.Vector3(p.x,p.y,p.z));
  world.quaternion.copy(heading).multiply(new THREE.Quaternion().setFromRotationMatrix(xrBasis));
}

function pauseWrist() {
  fetch('/api/v1/wrist', {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({space, seq:++seq, pose:null, paused:true}), keepalive:true})
    .catch(() => {}); // 后端按最后一次真实位姿的时间卸力。
}

function sendWrist(t, frame, ref) {
  if (!sending && t - lastSend >= 16 && session.visibilityState !== "hidden") {
    const sources = Array.from(session.inputSources);
    let tracked, sourceStatus;
    if (mode === 'controller') {
      const controller = sources.find(source => isController(source, 'right'));
      if (controller) tracked = frame.getPose(controller.gripSpace, ref);
      else if (sources.some(source => source.handedness === 'right' && source.hand)) sourceStatus = 'right_hand_only';
      else if (sources.some(source => source.handedness === 'right')) sourceStatus = 'no_controller_grip';
      else sourceStatus = 'no_right_controller';
    } else {
      const wrist = sources.find(s => s.handedness === 'right' && s.hand)?.hand.get('wrist');
      if (wrist) tracked = frame.getJointPose(wrist, ref);
      else sourceStatus = 'no_right_hand';
    }
    let pose = null;
    if (!sourceStatus) sourceStatus = tracked?.emulatedPosition ? 'emulated_position' : 'pose_unavailable';
    if (tracked && !tracked.emulatedPosition) {
      const {position:p, orientation:q} = tracked.transform;
      pose = [p.x,p.y,p.z,q.x,q.y,q.z,q.w];
      sourceStatus = "tracked";
    }
    const left = leftInput(frame, ref, sources);
    const requestAlign = align && pose !== null;
    const spaceSent = space;
    const sessionSent = session;
    sending = true; lastSend = t;
    fetch('/api/v1/wrist', {method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify({space, seq:++seq, pose, left, source_status:sourceStatus, align:requestAlign}), signal:AbortSignal.timeout(100)})
      .then(response => {
        if (response.ok && requestAlign && session === sessionSent && space === spaceSent) align = false;
      })
      .catch(() => {}).finally(() => { if (session === sessionSent) sending = false; });
  }
}

function pollScene(t, interval) {
  if (polling || t - lastPoll < interval) return;
  polling = true;
  lastPoll = t;
  const sessionSent = session;
  const after = graspView && !session ? latest?.frame : undefined;
  const path = after === undefined ? '/api/v1/scene' : `/api/v1/scene?after=${after}`;
  fetch(path, {signal:AbortSignal.timeout(session ? 150 : 700)})
    .then(response => response.status === 200 ? response.json() : null)
    .then(frame => {
      if (session === sessionSent && frame?.version === 1) latest = frame;
    })
    .catch(() => {}).finally(() => { polling = false; });
}

function applyFrame() {
  if (!latest || shown === latest) return;
  for (const [group, nodes] of Object.entries(groups)) {
    for (const [name, node] of nodes) {
      const pose = latest[group]?.[name];
      if (pose) applyPose(node, pose);
    }
  }
  diagnostics.update(latest, Number(forceScale.value));
  leftView?.update(latest.left_hand_tracking, latest.left_contact_probes);
  if (graspView && latest.grasp_object) {
    graspView.update(latest.grasp_object);
    groups.objects.get('table').visible = latest.grasp_object.layout === 'desk';
  }
  shown = latest;
  world.visible = true;
}

function draw(t, frame) {
  if (!frame || !session) return;
  const ref = renderer.xr.getReferenceSpace();
  if (!ref) return;
  sendWrist(t, frame, ref);
  pollScene(t, 25);
  const viewer = frame.getViewerPose(ref);
  if (!viewer) return;
  if (!previewPlaced) { placePreview(viewer); previewPlaced = true; }
  applyFrame();
  if (latest?.registration?.space === space) applyPose(world, latest.registration.xr_from_world);
  diagnostics.anchor.visible = Boolean(latest && showDiagnostics.checked);
  if (diagnostics.anchor.visible) diagnostics.place(viewer);
  renderer.render(scene, camera);
}

function drawDesktop(t) {
  if (session) return;
  pollScene(t, graspView ? 0 : 50);
  applyFrame();
  controls.update();
  diagnostics.anchor.visible = Boolean(latest && showDiagnostics.checked);
  if (diagnostics.anchor.visible) diagnostics.place({transform:{position:camera.position, orientation:camera.quaternion}}, camera.aspect);
  renderer.render(scene, camera);
}

function startDesktop() {
  scene.background = previewColor;
  world.position.set(0, -.045, .08);
  world.quaternion.setFromRotationMatrix(xrBasis);
  controls.enabled = true;
  renderer.setAnimationLoop(drawDesktop);
}

async function enter() {
  button.disabled = true;
  try {
    mode = sourceSelect.value;
    sourceSelect.disabled = true;
    status.textContent = "正在进入 Quest 沉浸场景";
    session = await navigator.xr.requestSession('immersive-ar', mode === 'hand'
      ? {requiredFeatures:['hand-tracking']}
      : {optionalFeatures:['hand-tracking']});
    controls.enabled = false;
    scene.background = null;
    session.addEventListener('visibilitychange', () => {
      if (session?.visibilityState === 'hidden') pauseWrist();
    });
    session.addEventListener('end', () => {
      pauseWrist();
      renderer.setAnimationLoop(null);
      session = null; latest = null; shown = null; button.disabled = false; sourceSelect.disabled = false;
      setTimeout(startDesktop, 0);
      status.textContent = '已退出沉浸场景。';
    }, {once:true});
    space = crypto.randomUUID(); seq = 0; lastSend = -Infinity; sending = false; latest = null; shown = null;
    align = false; previewPlaced = false; world.visible = false;
    session.addEventListener('select', e => {
      if (mode === 'controller' ? isController(e.inputSource, 'right') : e.inputSource.handedness === 'left') align = true;
    });
    await renderer.xr.setSession(session);
    renderer.xr.getReferenceSpace().addEventListener('reset', () => {
      space = crypto.randomUUID(); align = false; previewPlaced = false;
    });
    renderer.setAnimationLoop(draw);
    status.textContent = "沉浸场景运行中";
  } catch (err) {
    if (session) await session.end();
    status.textContent = String(err); button.disabled = false; sourceSelect.disabled = false;
  }
}

async function prepare() {
  renderer = new THREE.WebGLRenderer({alpha:true, antialias:true});
  renderer.setClearColor(0,0);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  preview.append(renderer.domElement);
  const resize = () => {
    if (session) return;
    const width = preview.clientWidth, height = preview.clientHeight;
    if (!width || !height) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(preview);
  resize();
  camera.position.set(.24, .17, .40);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, -.01, -.04);
  controls.minDistance = .22;
  controls.maxDistance = 1.15;
  controls.enableDamping = true;
  controls.update();
  renderer.xr.enabled = true;
  renderer.xr.setReferenceSpaceType('local');
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  const env = new RoomEnvironment();
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(env, .04).texture;
  env.dispose(); pmrem.dispose();
  const model = await new GLTFLoader().loadAsync('/api/v1/model.glb');
  world.add(model.scene);
  model.scene.traverse(node => {
    for (const [group, nodes] of Object.entries(groups)) {
      if (!node.isMesh && node.name.startsWith(group + '__')) nodes.set(node.name.slice(group.length + 2), node);
    }
  });
  const tabletop = groups.objects.has('table') && groups.objects.has('test_box');
  const palmTest = groups.objects.has('palm_ball');
  const localGrasp = groups.objects.has('table') && !tabletop && !groups.puzzle_cube.size;
  if (groups.right_hand.size !== 73 || (!tabletop && !palmTest && !localGrasp && groups.puzzle_cube.size !== 16)) throw Error('模型链接不完整');
  document.querySelector('h1').textContent = palmTest ? 'MotorHand 掌心测试' : tabletop ? 'MotorHand 桌面测试' : '右手与二阶魔方';
  diagnostics = new ForceDiagnostics(scene, groups.right_hand, {localGrasp});
  if (!localGrasp) leftView = new LeftHandView(world);
  if (localGrasp) {
    button.hidden = sourceSelect.closest('label').hidden = document.querySelector('#xr-hint').hidden = true;
    document.querySelector('h1').textContent = 'MotorHand 本地抓握';
    camera.position.set(.43, .22, .24);
    controls.target.set(.014, .098, .055);
    controls.update();
    graspView = new LocalGraspView(world, camera, renderer.domElement, controls);
  }
  startDesktop();
  if (localGrasp) {
    status.textContent = '固定腕部已就绪。桌面模式可拖动抓握物体。';
    return;
  }
  if (!navigator.xr || !await navigator.xr.isSessionSupported('immersive-ar')) {
    status.textContent = '桌面预览已连接。头显交互需要支持沉浸 AR 的浏览器。';
    return;
  }
  status.textContent = palmTest ? '固定腕部与掌心球已就绪。' : tabletop ? '桌面与测试方块已就绪。' : '手部与魔方已就绪。';
  button.disabled = false; button.onclick = enter;
}
prepare().catch(err => { status.textContent = '加载失败：' + err; });
