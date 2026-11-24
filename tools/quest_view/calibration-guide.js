import * as THREE from 'three';

const FONT = 'system-ui, "Noto Sans CJK SC", sans-serif';
const GREEN = '#3be898';
const CYAN = '#74dded';
const AMBER = '#f7be68';
const RED = '#ff6b57';

function makeHudCard(width, height, canvasW, canvasH) {
  const canvas = document.createElement('canvas');
  canvas.width = canvasW;
  canvas.height = canvasH;
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({map: texture, transparent: true, depthTest: false, depthWrite: false}),
  );
  mesh.renderOrder = 12;
  return {canvas, texture, mesh, ctx: canvas.getContext('2d')};
}

function drawText(ctx, value, x, y, size = 24, color = '#edf6fa', weight = '500', align = 'left') {
  ctx.fillStyle = color;
  ctx.font = `${weight} ${size}px ${FONT}`;
  ctx.textAlign = align;
  ctx.textBaseline = 'middle';
  ctx.fillText(value, x, y);
}

function fmtOffsetMm(vec) {
  if (!Array.isArray(vec) || vec.length !== 3) return '—';
  const f = v => `${v >= 0 ? '+' : ''}${(v * 1000).toFixed(1)}`;
  return `(${f(vec[0])}, ${f(vec[1])}, ${f(vec[2])}) mm`;
}

export class CalibrationGuide {
  constructor(scene, world) {
    // 1. 世界坐标系内：虚实重合标定时的手背辅助框 + 标定后短暂显示的金色杆臂预览线
    this.worldGroup = new THREE.Group();
    this.worldGroup.visible = false;
    world.add(this.worldGroup);

    // 虚拟手基准圆环（帮助在 AR 透视下快速把真实手套手腕/手背中心对准虚拟手）
    this.alignRing = new THREE.Mesh(
      new THREE.RingGeometry(0.032, 0.037, 40),
      new THREE.MeshBasicMaterial({color: 0x3be898, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false}),
    );
    this.alignRing.position.set(0.025, 0, 0.032);
    // 手背指尖方向箭头（+X）
    this.fwdArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.025, 0, 0.032), 0.075, 0x74dded, 0.018, 0.012,
    );
    this.worldGroup.add(this.alignRing, this.fwdArrow);

    // 标定完成后短暂显示实际杆臂向量（base_link → 手柄位置）
    this.leverGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.leverLine = new THREE.Line(
      this.leverGeo,
      new THREE.LineBasicMaterial({color: 0xf7be68, transparent: true, opacity: 0.95}),
    );
    this.ctrlDot = new THREE.Mesh(
      new THREE.SphereGeometry(0.009, 14, 10),
      new THREE.MeshBasicMaterial({color: 0xf7be68, transparent: true, opacity: 0.9}),
    );
    this.worldGroup.add(this.leverLine, this.ctrlDot);

    // 2. 视场 HUD：虚实重合引导卡放在视野上方，绝不遮挡眼前重合中的虚拟手
    this.hudAnchor = new THREE.Group();
    this.hudAnchor.visible = false;
    scene.add(this.hudAnchor);

    this.overlayCard = makeHudCard(0.50, 0.21, 820, 344);
    this.overlayCard.mesh.position.set(0, 0.175, 0);
    this.overlayCard.mesh.visible = false;

    this.bannerCard = makeHudCard(0.50, 0.068, 820, 112);
    this.bannerCard.mesh.position.set(0, -0.185, 0);
    this.bannerCard.mesh.visible = false;

    this.hudAnchor.add(this.overlayCard.mesh, this.bannerCard.mesh);

    this._basePos = new THREE.Vector3();
    this._baseQuat = new THREE.Quaternion();
    this._vec = new THREE.Vector3();
    this._lastRenderKey = '';
  }

  place(viewer, desktopAspect = null) {
    if (!this.hudAnchor.visible) return;
    const {position, orientation} = viewer.transform;
    this.hudAnchor.quaternion.set(orientation.x, orientation.y, orientation.z, orientation.w);
    const desktop = desktopAspect !== null;
    this.hudAnchor.scale.setScalar(desktop ? Math.min(0.82, desktopAspect * 0.50) : 1.0);
    this.hudAnchor.position.set(0, desktop ? 0.01 : 0.0, desktop ? -0.54 : -0.60)
      .applyQuaternion(this.hudAnchor.quaternion);
    this.hudAnchor.position.x += position.x;
    this.hudAnchor.position.y += position.y;
    this.hudAnchor.position.z += position.z;
  }

  update(frame, inXrSession, ctrlTracked = false, localWarning = '') {
    const cal = frame?.wrist_offset;
    const isFixed = frame?.scene_kind === 'local_grasp' || frame?.scene_kind === 'palm_test';
    if (!cal || isFixed) {
      this.worldGroup.visible = false;
      this.hudAnchor.visible = false;
      return;
    }

    const base = frame.right_hand?.base_link;
    const overlayActive = Boolean(cal.overlay_active);
    const showToast = !overlayActive && Boolean(cal.recent_feedback && cal.message);
    this.worldGroup.visible = Boolean(base && (overlayActive || showToast));

    if (this.worldGroup.visible && base) {
      this._basePos.fromArray(base);
      this._baseQuat.fromArray(base, 3);

      this.alignRing.visible = overlayActive;
      this.fwdArrow.visible = overlayActive;

      const leverVec = cal.offset_m;
      const leverNorm = Array.isArray(leverVec) ? Math.hypot(leverVec[0], leverVec[1], leverVec[2]) : 0;
      const showArrow = !overlayActive && showToast && leverNorm > 0.003;
      this.leverLine.visible = showArrow;
      this.ctrlDot.visible = showArrow;
      if (showArrow) {
        this._vec.fromArray(leverVec).applyQuaternion(this._baseQuat).add(this._basePos);
        const posAttr = this.leverGeo.attributes.position;
        posAttr.setXYZ(0, this._basePos.x, this._basePos.y, this._basePos.z);
        posAttr.setXYZ(1, this._vec.x, this._vec.y, this._vec.z);
        posAttr.needsUpdate = true;
        this.ctrlDot.position.copy(this._vec);
      }
    }

    const needsAlign = !overlayActive && typeof frame.wrist_detail === 'string' && frame.wrist_detail.includes('对齐');
    const showIdleHint = !overlayActive && !needsAlign && !showToast && inXrSession;

    this.overlayCard.mesh.visible = overlayActive;
    this.bannerCard.mesh.visible = !overlayActive && (needsAlign || showToast || showIdleHint);
    this.hudAnchor.visible = this.overlayCard.mesh.visible || this.bannerCard.mesh.visible;
    if (!this.hudAnchor.visible) return;

    const fb = frame.motor_feedback;
    const fbLabel = fb?.active ? '已开启' : fb?.requested ? '待命' : '已关闭';
    const key = overlayActive
      ? `O:${ctrlTracked}:${localWarning}:${cal.message}`
      : `B:${needsAlign}:${showToast}:${showIdleHint}:${fbLabel}:${cal.calibrated_6dof}:${cal.message}:${JSON.stringify(cal.offset_m)}`;
    if (key === this._lastRenderKey) return;
    this._lastRenderKey = key;

    if (overlayActive) {
      this._drawOverlayCard(ctrlTracked, localWarning || cal.message);
    } else if (needsAlign) {
      this._drawBanner(
        `【左手扳机】对齐桌面   |   【左手 Y】力反馈(${fbLabel})   |   【左手 X】虚实标定`,
        CYAN, 'rgba(10, 29, 46, 0.92)',
      );
    } else if (showToast) {
      const isOk = cal.message.includes('保存') || cal.message.includes('应用') || cal.message.includes('已开启');
      this._drawBanner(cal.message, isOk ? GREEN : AMBER, 'rgba(10, 29, 46, 0.94)');
    } else if (showIdleHint) {
      const border = fb?.active ? GREEN : '#c7e2f0';
      this._drawBanner(
        `左手柄：Y = 开关力反馈（${fbLabel}）   |   扳机 = 对齐桌面   |   X/侧握 = 虚实标定`,
        border, 'rgba(8, 20, 34, 0.84)',
      );
    }
  }

  _drawBanner(label, borderColor, bgColor) {
    const {canvas, ctx, texture} = this.bannerCard;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    ctx.roundRect(8, 8, canvas.width - 16, canvas.height - 16, 22);
    ctx.fillStyle = bgColor;
    ctx.fill();
    ctx.strokeStyle = borderColor;
    ctx.lineWidth = 3;
    ctx.stroke();
    drawText(ctx, label, canvas.width / 2, canvas.height / 2, 24, '#f3fbff', '600', 'center');
    texture.needsUpdate = true;
  }

  _drawOverlayCard(ctrlTracked, note) {
    const {canvas, ctx, texture} = this.overlayCard;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.beginPath();
    ctx.roundRect(8, 8, canvas.width - 16, canvas.height - 16, 24);
    ctx.fillStyle = 'rgba(8, 20, 34, 0.92)';
    ctx.fill();
    ctx.strokeStyle = ctrlTracked ? GREEN : AMBER;
    ctx.lineWidth = 3.5;
    ctx.stroke();

    // 标题与右手柄追踪状态胶囊
    drawText(ctx, '虚实重合标定（全程左手柄操作）', 32, 46, 30, '#f4fbff', '700');
    ctx.beginPath();
    ctx.roundRect(canvas.width - 238, 24, 206, 42, 21);
    ctx.fillStyle = ctrlTracked ? 'rgba(59, 232, 152, 0.22)' : 'rgba(247, 190, 104, 0.22)';
    ctx.fill();
    drawText(ctx, ctrlTracked ? '● 右手柄已追踪' : '○ 等待右手柄追踪', canvas.width - 135, 45, 21, ctrlTracked ? GREEN : AMBER, '700', 'center');

    // 三步直观引导
    drawText(ctx, '1. 把真实右手手套伸进眼前静止的虚拟手里，手背与五指完全重合', 34, 106, 23, '#edf7fc', '600');
    drawText(ctx, '2. 如果虚拟手位置不顺手，按【左手侧握键】把它重新摆到眼前', 34, 146, 23, '#cde3f0', '500');
    drawText(ctx, '3. 虚实重合后稳住右手，按【左手扳机 / X 键】确认并自动保存', 34, 186, 23, ctrlTracked ? GREEN : '#cde3f0', '700');

    if (note) {
      const warn = note.includes('未检测') || note.includes('过大') || note.includes('失败');
      drawText(ctx, note, canvas.width / 2, 232, 20, warn ? RED : '#ffd38a', '600', 'center');
    }

    // 底部按键栏
    ctx.beginPath();
    ctx.roundRect(28, 260, canvas.width - 56, 56, 15);
    ctx.fillStyle = ctrlTracked ? 'rgba(59, 232, 152, 0.22)' : 'rgba(32, 56, 75, 0.90)';
    ctx.fill();
    ctx.strokeStyle = ctrlTracked ? GREEN : '#486d88';
    ctx.lineWidth = 2;
    ctx.stroke();
    drawText(
      ctx,
      '【左手扳机 / X】确认重合保存   |   【左手侧握】重摆位置   |   【左手 Y】取消',
      canvas.width / 2, 288, 21, '#effcf6', '700', 'center',
    );
    texture.needsUpdate = true;
  }
}
