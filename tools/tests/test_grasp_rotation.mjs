import assert from 'node:assert/strict';
import test from 'node:test';
import {RotationControl} from '../quest_view/rotation-control.mjs';

test('连续拖动合并在途角度，松手发送最终值并等待控制快照回显', async () => {
  const sends = [], releases = [];
  const control = new RotationControl((kind, angle) => {
    sends.push([kind, angle]);
    return new Promise(resolve => releases.push(resolve));
  });
  control.update('cup', 10);
  control.update('cup', 25);
  control.update('cup', 60, true);
  assert.deepEqual(sends, [['cup', 10]]);
  assert.equal(control.waitingForEcho('cup', 10), true);
  releases.shift()(true);
  await new Promise(setImmediate);
  assert.deepEqual(sends, [['cup', 10], ['cup', 60]]);
  releases.shift()(true);
  await new Promise(setImmediate);
  assert.equal(control.waitingForEcho('cup', 10), true);
  assert.equal(control.waitingForEcho('cup', 60), false);
});

test('拖动过程中即发布，切换物体取消待发角度', async () => {
  const sends = [], releases = [];
  let resolveSend;
  const control = new RotationControl((kind, angle) => {
    sends.push([kind, angle]);
    resolveSend?.();
    return new Promise(resolve => releases.push(resolve));
  });
  control.update('ball', 0);
  control.update('ball', 5);
  control.update('ball', 10);
  const secondSend = new Promise(resolve => { resolveSend = resolve; });
  releases.shift()(true);
  await secondSend;
  assert.deepEqual(sends, [['ball', 0], ['ball', 10]]);
  control.update('ball', 15);
  const done = control.cancel();
  releases.shift()(true);
  await done;
  control.update('cube', -45, true);
  assert.deepEqual(sends, [['ball', 0], ['ball', 10], ['cube', -45]]);
  releases.shift()(true);
});
