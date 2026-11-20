import * as THREE from 'three';

// 顺序与右手 joint_space.json 的独立关节和 G4 电机 ID 一致。
const FINGER_AXES = [
  ['拇指', [[1, '腕掌屈伸'], [2, '腕掌外展'], [3, '掌指关节'], [4, '指间关节']]],
  ['食指', [[5, '掌指屈伸'], [6, '近端 / 远端指间联动']]],
  ['中指', [[7, '掌指屈伸'], [8, '近端 / 远端指间联动']]],
  ['无名指', [[9, '掌指屈伸'], [10, '近端 / 远端指间联动']]],
  ['小指', [[11, '掌指屈伸'], [12, '近端 / 远端指间联动']]],
];
const GHOST_ORIENTATION = new THREE.Quaternion().setFromRotationMatrix(
  new THREE.Matrix4().set(0,1,0,0, 1,0,0,0, 0,0,-1,0, 0,0,0,1)
);
const COOL = new THREE.Color('#1e9abc');
const HOT = new THREE.Color('#ff6342');
const FONT = 'system-ui, "Noto Sans CJK SC", sans-serif';
const LOAD_BAR_MAX_NM = 10;

function card(width, height, x, y) {
  const canvas = document.createElement('canvas');
  canvas.width = 640;
  canvas.height = Math.round(640 * height / width);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({map: texture, transparent:true, depthTest:false, depthWrite:false}));
  mesh.position.set(x, y, 0);
  mesh.renderOrder = 8;
  return {canvas, texture, mesh, ctx:canvas.getContext('2d')};
}

function text(ctx, value, x, y, size = 30, color = '#e6f3f7', weight = '500') {
  ctx.fillStyle = color;
  ctx.font = `${weight} ${size}px ${FONT}`;
  ctx.fillText(value, x, y);
}

function background({canvas, ctx}, title, subtitle) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath();
  ctx.roundRect(6, 6, canvas.width-12, canvas.height-12, 23);
  ctx.fillStyle = 'rgba(8, 19, 32, .92)';
  ctx.fill();
  ctx.strokeStyle = '#41617a';
  ctx.lineWidth = 2;
  ctx.stroke();
  text(ctx, title, 28, 56, 36, '#f4fbff', '700');
  text(ctx, subtitle, 28, 94, 24, '#9fbcd1');
}

function motorReadout(card, frame) {
  const fb = frame.motor_feedback;
  const tracking = frame.wrist_detail ?? '等待腕部定位';
  const timing = (frame.time_monotonic_s * 1e9 - frame.motor_received_ns) / 1e6;
  const mode = fb?.active ? '真机出力已启用' : fb?.requested ? '已请求启用 · 等待有效帧' : '仿真反馈预览';
  background(card, 'MotorHand', mode);
  text(card.ctx, '右腕', 29, 145, 26, '#95b5cc', '700');
  text(card.ctx, tracking, 163, 145, 26);
  if (frame.scene_kind === 'local_grasp') {
    text(card.ctx, '接触', 29, 197, 26, '#95b5cc', '700');
    text(card.ctx, frame.hand_cube_contacts?.grasp_object?.length ? '物体受力' : '自由空间', 163, 197, 26);
  } else {
    text(card.ctx, '左手', 29, 197, 26, '#95b5cc', '700');
    const left = frame.left_hand_tracking?.kind;
    text(card.ctx, left === 'hand' ? '光学追踪' : left === 'controller' ? '手柄支撑' : '等待追踪', 163, 197, 26);
  }
  text(card.ctx, '手套输入', 29, 249, 26, '#95b5cc', '700');
  text(card.ctx, frame.motor_received_ns && Number.isFinite(timing) ? `${Math.max(0, timing).toFixed(0)} ms` : '—', 163, 249, 26);
  const nominal = fb?.nominal_nm ?? [];
  const available = nominal.some(Number.isFinite);
  text(card.ctx, '受力轴', 29, 301, 26, '#95b5cc', '700');
  const loaded = nominal.flatMap((n, i) => Number.isFinite(n) && Math.abs(n) >= .01 ? [i + 1] : []);
  text(card.ctx, loaded.length ? loaded.join(' / ') : available ? '无' : '等待有效接触帧', 163, 301, 26);
  const valid = nominal.map((v,i) => [v,i]).filter(([v]) => Number.isFinite(v) && v !== 0);
  valid.sort(([a],[b]) => Math.abs(b)-Math.abs(a));
  const [torque, axis] = valid[0] ?? [];
  const current = fb?.current_a?.[axis];
  text(card.ctx, '名义力矩', 29, 353, 26, '#95b5cc', '700');
  const peak = axis === undefined ? (available ? '0.00 N·m' : '—') : `轴 ${axis+1}  ·  ${torque.toFixed(2)} N·m`;
  text(card.ctx, peak, 163, 353, 26);
  text(card.ctx, '同轴回报', 29, 405, 26, '#95b5cc', '700');
  text(card.ctx, Number.isFinite(current) ? `${current.toFixed(3)} A` : '—', 163, 405, 26);
  card.texture.needsUpdate = true;
}

function jointLoads(card, frame) {
  background(card, '接触载荷', 'J1–J12 · 模型名义力矩（N·m）');
  const {ctx} = card;
  const nominal = frame.motor_feedback?.nominal_nm ?? [];
  FINGER_AXES.forEach(([finger, axes], index) => {
    const y = 130 + index * 66;
    const width = 514 / axes.length;
    if (index) {
      ctx.fillStyle = '#355066';
      ctx.fillRect(26, y - 26, 588, 1);
    }
    text(ctx, finger, 27, y + 12, 24, '#dcecf4', '700');
    axes.forEach(([id, name], column) => {
      const x = 108 + column * width;
      const value = nominal[id - 1];
      const loaded = Number.isFinite(value) && Math.abs(value) >= .01;
      text(ctx, `J${id}`, x, y, 25, loaded ? '#ffc38d' : '#a7cbe0', '700');
      const barX = x + 50;
      const ratio = Number.isFinite(value) ? Math.min(Math.abs(value) / LOAD_BAR_MAX_NM, 1) : 0;
      ctx.fillStyle = '#284056';
      ctx.fillRect(barX, y - 17, 70, 14);
      if (ratio > 0) {
        ctx.fillStyle = loaded ? '#ff9259' : '#67bdd9';
        ctx.fillRect(barX, y - 17, 70 * ratio, 14);
      }
      text(ctx, name, x, y + 27, 23, '#a4bfd1');
    });
  });
  card.texture.needsUpdate = true;
}

export class ForceDiagnostics {
  constructor(scene, links, {localGrasp = false} = {}) {
    this.anchor = new THREE.Group();
    this.anchor.visible = false;
    scene.add(this.anchor);
    this.leftCard = card(.288, .210, -.263, .010);
    this.rightCard = card(.288, .210, .263, .010);
    this.anchor.add(this.leftCard.mesh, this.rightCard.mesh);
    this.ghost = new THREE.Group();
    this.ghost.quaternion.copy(GHOST_ORIENTATION);
    this.ghost.position.y = localGrasp ? -.073 : -.067;
    this.ghost.scale.setScalar(localGrasp ? .95 : 1.12);
    this.anchor.add(this.ghost);
    this.parts = new Map();

    for (const [name, link] of links) {
      const mesh = link.clone(true);
      const material = new THREE.MeshBasicMaterial({
        color: COOL, transparent:true, opacity:.24, depthWrite:false, side:THREE.DoubleSide,
      });
      mesh.traverse(child => { if (child.isMesh) child.material = material; });
      this.ghost.add(mesh);
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(), .028, '#ffb974');
      arrow.visible = false;
      this.ghost.add(arrow);
      this.parts.set(name, {mesh, material, arrow});
    }
    this.inverse = new THREE.Quaternion();
    this.wrist = new THREE.Vector3();
    this.vector = new THREE.Vector3();
  }

  place(viewer, desktopAspect = null) {
    const {position, orientation} = viewer.transform;
    this.anchor.quaternion.set(orientation.x, orientation.y, orientation.z, orientation.w);
    const desktop = desktopAspect !== null;
    this.anchor.scale.setScalar(desktop ? Math.min(1, desktopAspect * .62) : 1);
    this.anchor.position.set(0, desktop ? .141 : .188, desktop ? -.46 : -.60).applyQuaternion(this.anchor.quaternion);
    this.anchor.position.x += position.x;
    this.anchor.position.y += position.y;
    this.anchor.position.z += position.z;
  }

  update(frame, fullScale) {
    const base = frame.right_hand?.base_link;
    if (!base) return;
    this.wrist.fromArray(base);
    this.inverse.fromArray(base, 3).invert();
    for (const [name, {mesh, material, arrow}] of this.parts) {
      const pose = frame.right_hand[name];
      if (!pose) continue;
      mesh.position.fromArray(pose).sub(this.wrist).applyQuaternion(this.inverse);
      mesh.quaternion.fromArray(pose, 3).premultiply(this.inverse);
      const force = frame.right_contact_force_n?.[name] ?? [0,0,0];
      this.vector.fromArray(force).applyQuaternion(this.inverse);
      const magnitude = this.vector.length();
      const intensity = Math.min(magnitude / fullScale, 1);
      material.color.copy(COOL).lerp(HOT, Math.sqrt(intensity));
      material.opacity = .24 + .63 * Math.sqrt(intensity);
      arrow.visible = magnitude > .05;
      if (arrow.visible) {
        arrow.position.copy(mesh.position);
        arrow.setDirection(this.vector.normalize());
        arrow.setLength(.008 + .031 * Math.sqrt(intensity), .006, .004);
      }
    }
    jointLoads(this.rightCard, frame);
    motorReadout(this.leftCard, frame);
  }
}


export class LeftHandView {
  constructor(world) {
    this.group = new THREE.Group();
    world.add(this.group);
    const material = new THREE.MeshStandardMaterial({color:'#826ec4', roughness:.62});
    const outline = new THREE.MeshBasicMaterial({color:'#d9b8ff', transparent:true, opacity:.38});
    const joint = new THREE.SphereGeometry(1, 12, 8);
    const bone = new THREE.CylinderGeometry(1, 1, 1, 10);
    const add = (shape, appearance) => {
      const mesh = new THREE.Mesh(shape, appearance);
      this.group.add(mesh);
      return mesh;
    };
    this.joints = Array.from({length:16}, () => add(joint, material));
    this.bones = Array.from({length:15}, () => add(bone, material));
    this.probes = Array.from({length:6}, () => add(joint, outline));
    this.direction = new THREE.Vector3();
    this.from = new THREE.Vector3();
    this.to = new THREE.Vector3();
    this.axis = new THREE.Vector3(0,1,0);
  }

  update(tracking, colliders) {
    this.group.visible = Boolean(tracking);
    if (!tracking) return;
    const points = tracking.points;
    this.joints.forEach((joint, i) => {
      joint.visible = i < points.length;
      if (joint.visible) {
        joint.position.fromArray(points[i]);
        joint.scale.setScalar(i === 0 ? (tracking.kind === 'hand' ? .012 : .016) : .006);
      }
    });
    this.bones.forEach((bone, i) => {
      bone.visible = tracking.kind === 'hand';
      if (!bone.visible) return;
      this.from.fromArray(points[i % 3 === 0 ? 0 : i]);
      this.to.fromArray(points[i+1]);
      this.direction.subVectors(this.to, this.from);
      const length = this.direction.length();
      bone.position.copy(this.from).lerp(this.to, .5);
      bone.quaternion.setFromUnitVectors(this.axis, this.direction.normalize());
      bone.scale.set(i % 3 === 0 ? .008 : .005, length, i % 3 === 0 ? .008 : .005);
    });
    this.probes.forEach((marker, i) => {
      const contact = colliders?.[i];
      marker.visible = Boolean(contact);
      if (contact) marker.position.fromArray(contact);
      marker.scale.setScalar(i === 5 ? .020 : .008);
    });
  }
}
