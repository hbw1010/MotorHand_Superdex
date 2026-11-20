import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import {LocalGraspView} from '../local-grasp.js';
import {sizePreview} from '../size-preview.mjs';

test('硬体、软体及官方组合均绕可见中心缩放，物理尺寸确认后清除临时缩放', async () => {
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
    const after = bounds();
    assert.ok(before.getCenter(new THREE.Vector3()).distanceTo(after.getCenter(new THREE.Vector3())) < 1e-6);
    assert.ok(before.getSize(new THREE.Vector3()).multiplyScalar(3).distanceTo(after.getSize(new THREE.Vector3())) < 1e-6);
    sizePreview.submitted(target, true);
    sizePreview.controlSnapshot({...object, size_mm:150});
    view.update(object);
    assert.equal(view.sizeGroup.scale.x, 3);
    view.update({...object, size_mm:150, mesh_revision:++revision});
    assert.equal(view.sizeGroup.scale.x, 1);
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
