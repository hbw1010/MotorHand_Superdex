import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import {LocalGraspView} from '../local-grasp.js';
import {sizePreview} from '../size-preview.mjs';

test('尺寸线框独立缩放，硬体、变形网格及组合的实体和拾取均取物理帧', async () => {
  const world = new THREE.Group();
  const view = new LocalGraspView(world, null, new EventTarget(), {});
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  view.loader.loadAsync = async () => ({scene: mesh});
  const pose = [2, 3, 4, 0, 0, 0, 1];
  const objects = [
    {kind:'ball', pose},
    {kind:'ball', pose, vertices:[1,3,4, 3,3,4, 2,4,4], indices:[0,1,2]},
    {kind:'official_test', pose, parts:[{pose:[5,2,3,0,0,0,1], model:'sample.glb', scale:[1,1,1]}]},
  ];
  let revision = 0;
  for (const data of objects) {
    const object = {...data, hardness:1, size_mm:50, mesh_revision:++revision};
    view.update(object);
    await Promise.resolve();
    const bounds = () => {
      world.updateMatrixWorld(true);
      return new THREE.Box3().setFromObject(view.mesh);
    };
    const before = bounds();
    const target = sizePreview.update(object.kind, 150);
    assert.ok(bounds().equals(before), '预览期间实体用于拾取的世界包围盒须保持不变');
    const outline = view.sizeOutline;
    assert.ok(outline?.isLineSegments && outline.parent === world);
    assert.ok(!view.mesh.getObjectById(outline.id), '拾取实体的子树中不能包含目标框');
    assert.ok(before.getCenter(new THREE.Vector3()).distanceTo(outline.box.getCenter(new THREE.Vector3())) < 1e-6);
    assert.ok(before.getSize(new THREE.Vector3()).multiplyScalar(3)
      .distanceTo(outline.box.getSize(new THREE.Vector3())) < 1e-6);
    let disposed = 0;
    outline.geometry.addEventListener('dispose', () => disposed++);
    outline.material.addEventListener('dispose', () => disposed++);
    sizePreview.submitted(target, true);
    sizePreview.controlSnapshot({...object, size_mm:150});
    view.update(object);
    assert.equal(view.sizeOutline, outline);
    assert.ok(bounds().equals(before));
    view.update({...object, size_mm:150, mesh_revision:++revision});
    assert.equal(view.sizeOutline, null);
    assert.equal(disposed, 2);
    assert.equal(sizePreview.target, null);
  }
});

test('请求失败、取消操作或换物体时放弃待应用尺寸', () => {
  const cube = {kind:'box', size_mm:100};
  const target = sizePreview.update('box', 150);
  assert.equal(sizePreview.ratio(cube), 1.5);
  sizePreview.submitted(target, false);
  assert.equal(sizePreview.ratio(cube), 1);
  sizePreview.update('box', 20);
  sizePreview.clear();
  assert.equal(sizePreview.ratio(cube), 1);
  sizePreview.update('box', 20);
  sizePreview.sceneSnapshot({kind:'ball', size_mm:45});
  assert.equal(sizePreview.target, null);
});

test('尺寸线框独立于查看角度，拖动或提交失败不会扩大实际可拾取区域', () => {
  const world = new THREE.Group();
  world.position.set(.2, .1, -.3);
  world.rotation.y = .7;
  const view = new LocalGraspView(world, null, new EventTarget(), {});
  const object = {kind:'box', size_mm:50, hardness:1, mesh_revision:1, pose:[1, 2, 3, 0, 0, 0, 1]};
  view.update(object);
  const target = sizePreview.update(object.kind, 150);
  const outline = view.sizeOutline;
  const c = outline.box.getCenter(new THREE.Vector3());
  const d = outline.box.getSize(new THREE.Vector3());
  assert.ok(c.distanceTo(new THREE.Vector3(1, 2, 3)) < 1e-6);
  assert.ok(d.distanceTo(new THREE.Vector3(.15, .15, .15)) < 1e-6);
  world.updateMatrixWorld(true);
  const origin = world.localToWorld(new THREE.Vector3(1.06, 2, 3.5));
  const direction = world.localToWorld(new THREE.Vector3(1.06, 2, 3)).sub(origin).normalize();
  const ray = new THREE.Raycaster(origin, direction);
  assert.deepEqual(ray.intersectObject(view.mesh, true), []);
  sizePreview.submitted(target, false);
  assert.equal(view.sizeOutline, null);
  assert.equal(outline.parent, null);
});

test('没有新物理帧时，尺寸预览和异步部件加载也会请求重绘', async () => {
  let renders = 0;
  const view = new LocalGraspView(new THREE.Group(), null, new EventTarget(), {}, () => { renders++; });
  let loaded;
  view.loader.loadAsync = () => new Promise(resolve => { loaded = resolve; });
  const pose = [0, 0, 0, 0, 0, 0, 1];
  view.update({kind:'official_test', size_mm:50, mesh_revision:1, pose,
               parts:[{model:'test.glb', pose, scale:[1,1,1]}]});
  assert.equal(renders, 0);
  sizePreview.update('official_test', 75);
  assert.equal(renders, 1);
  loaded({scene:new THREE.Mesh(new THREE.BoxGeometry())});
  await Promise.resolve();
  assert.equal(renders, 2);
  sizePreview.clear();
  assert.equal(renders, 3);
});
