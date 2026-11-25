import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const container = document.querySelector('#sharpa-preview');
const viewerStatus = document.querySelector('#sharpa-3d-status');
const resetCamBtn = document.querySelector('#sharpa-reset-cam');

if (container) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#131e2b');
  const world = new THREE.Group();
  // MuJoCo Z-up -> Three.js Y-up
  world.rotation.x = -Math.PI / 2;
  scene.add(world);

  const grid = new THREE.GridHelper(0.9, 18, 0x2b455c, 0x1c2e3e);
  grid.position.y = -0.02;
  scene.add(grid);

  const camera = new THREE.PerspectiveCamera(45, 16 / 8, 0.01, 20);
  const renderer = new THREE.WebGLRenderer({antialias: true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.08;
  container.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.minDistance = 0.2;
  controls.maxDistance = 1.8;

  const resetCamera = () => {
    camera.position.set(0.0, 0.28, 0.62);
    controls.target.set(0.0, 0.16, 0.0);
    controls.update();
  };
  resetCamera();
  if (resetCamBtn) resetCamBtn.addEventListener('click', resetCamera);

  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  pmrem.dispose();

  const keyLight = new THREE.DirectionalLight(0xffffff, 1.3);
  keyLight.position.set(1.2, 2.0, 1.5);
  scene.add(keyLight);
  const fillLight = new THREE.DirectionalLight(0x86deed, 0.45);
  fillLight.position.set(-1.2, 0.8, -1.0);
  scene.add(fillLight);

  // 仿真抓握球（与 sharpa_device.cpp 的 35mm 球完全对应）
  const ballGeo = new THREE.SphereGeometry(0.035, 32, 24);
  const ballMat = new THREE.MeshStandardMaterial({
    color: 0xd95f2b, roughness: 0.42, metalness: 0.1, transparent: true, opacity: 0.92,
  });
  const graspBall = new THREE.Mesh(ballGeo, ballMat);
  graspBall.visible = false;
  world.add(graspBall);

  const bodyNodes = new Map();
  const bodyMaterials = new Map();
  let loadedKey = null;
  let loadedGloveModel = '12dof';
  let lastSeq = -1;
  let polling = false;

  const resize = () => {
    const w = Math.max(200, container.clientWidth);
    const h = Math.max(200, container.clientHeight);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  resize();
  new ResizeObserver(resize).observe(container);

  async function loadModel(hand, gloveModel) {
    const label = gloveModel === '16dof' ? '16-DOF' : '12-DOF (VE40)';
    if (viewerStatus) viewerStatus.textContent = `正在加载 MotorHand ${label} + Sharpa 3D 模型…`;
    for (const node of bodyNodes.values()) world.remove(node);
    bodyNodes.clear();
    bodyMaterials.clear();
    const gltf = await new GLTFLoader().loadAsync(
      `/api/v1/sharpa-model.glb?hand=${hand}&glove_model=${gloveModel}&v=${Date.now()}`
    );
    gltf.scene.traverse(obj => {
      if (obj.name && obj.name.startsWith('body__')) {
        const bname = obj.name.slice(6);
        bodyNodes.set(bname, obj);
        const mats = [];
        obj.traverse(child => {
          if (child.isMesh && child.material) {
            child.material = child.material.clone();
            mats.push({mat: child.material, baseEmissive: child.material.emissive.clone()});
          }
        });
        bodyMaterials.set(bname, mats);
      }
    });
    // 把所有 body 节点直接挂到 world（MuJoCo/Pinocchio xpos/xquat 为世界坐标）
    for (const node of bodyNodes.values()) {
      world.add(node);
    }
    loadedKey = `${hand}:${gloveModel}`;
    loadedGloveModel = gloveModel;
    if (viewerStatus) viewerStatus.textContent = `3D 并排视图就绪 · 左：MotorHand ${label} ｜ 右：Sharpa WaveSE`;
  }

  function applyScene(data) {
    if (!data || !data.bodies) return;
    for (const [bname, pose] of Object.entries(data.bodies)) {
      const node = bodyNodes.get(bname);
      if (!node) continue;
      node.position.set(pose[0], pose[1], pose[2]);
      node.quaternion.set(pose[3], pose[4], pose[5], pose[6]);
    }
    if (Array.isArray(data.object_pos) && data.object_pos.length === 3) {
      graspBall.position.set(data.object_pos[0], data.object_pos[1], data.object_pos[2]);
    }
    graspBall.visible = Boolean(data.show_object);

    const hotFingers = new Set(Array.isArray(data.contact_fingers) ? data.contact_fingers : []);
    for (const [bname, mats] of bodyMaterials.entries()) {
      if (!bname.startsWith('sharpa_')) continue;
      let hit = false;
      for (const f of hotFingers) {
        if (bname.includes(f)) {
          hit = true;
          break;
        }
      }
      for (const {mat, baseEmissive} of mats) {
        if (hit) mat.emissive.setHex(0x883a08);
        else mat.emissive.copy(baseEmissive);
      }
    }
    if (viewerStatus) {
      const label = loadedGloveModel === '16dof' ? '16-DOF (全 22 轴自由)' : '12-DOF (VE40)';
      const tag = data.running ? (data.simulated ? '离线仿真同步中' : '真机实时同步中') : '待命预览（启动仿真或真机后实时驱动）';
      const hitStr = hotFingers.size ? ` · 接触受力: ${[...hotFingers].join(', ')}` : '';
      viewerStatus.textContent = `左：MotorHand ${label} ｜ 右：Sharpa WaveSE · ${tag}${hitStr}`;
    }
  }

  async function pollScene() {
    if (polling) return;
    polling = true;
    try {
      const res = await fetch('/api/v1/sharpa-scene', {signal: AbortSignal.timeout(1200)});
      if (res.ok) {
        const data = await res.json();
        const hand = data.hand || 'right';
        const gloveModel = data.glove_model || '12dof';
        const key = `${hand}:${gloveModel}`;
        if (key !== loadedKey && data.ready) {
          await loadModel(hand, gloveModel);
          lastSeq = -1;
        }
        if (data.ready && data.seq !== lastSeq) {
          lastSeq = data.seq;
          applyScene(data);
        }
      }
    } catch {
      // ignore transient switch/network timeouts
    } finally {
      polling = false;
      const delay = document.visibilityState === 'hidden' ? 500 : 33;
      setTimeout(pollScene, delay);
    }
  }

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
  pollScene();
}
