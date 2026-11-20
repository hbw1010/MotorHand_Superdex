import assert from 'node:assert/strict';
import test from 'node:test';
import {ForceReadout} from '../quest_view/force-readout.mjs';

test('两轴力矩接近时保持观察轴，明显超过或原轴卸力时立即切换', () => {
  const display = new ForceReadout();
  assert.equal(display.update([-.60, .58], 1, 1).axis, 0);
  assert.equal(display.update([-.59, .62], 1, 1).axis, 0);
  assert.equal(display.update([-.52, .81], 1, 1).axis, 1);
  assert.equal(display.update([-.08, 0], 1, 1).axis, 0);
});

test('轻触高亮使用进入和退出阈值，失效和换物体清除历史', () => {
  const display = new ForceReadout();
  assert.deepEqual(display.update([.031, .012], 1, 1).loaded, [0]);
  assert.deepEqual(display.update([.011, .021], 1, 1).loaded, [0]);
  assert.deepEqual(display.update([.011, .021], 1, 2).loaded, []);
  assert.deepEqual(display.update([.031, .021], 1, 2).loaded, [0]);
  assert.deepEqual(display.update([null, null], 1, 2), {available:false, axis:null, loaded:[]});
  assert.deepEqual(display.update([.011, .021], 1, 2).loaded, []);
  assert.deepEqual(display.update([0, 0], 1, 2), {available:true, axis:null, loaded:[]});
});
