// 按最新物理网格的实际尺寸计算倍率，重建后自动回到 1。
export class SizePreview extends EventTarget {
  update(kind, size) {
    clearTimeout(this.timer);
    this.target = {kind, size};
    this.dispatchEvent(new Event('change'));
    return this.target;
  }

  submitted(target, ok) {
    if (this.target !== target) return;
    if (!ok) return this.clear();
    target.submitted = true;
    this.timer = setTimeout(() => this.clear(), 5000);
    this.finishIfApplied();
  }

  controlSnapshot(grasp) {
    if (!this.target) return;
    if (grasp.kind !== this.target.kind) return this.clear();
    this.target.controlMatches = grasp.size_mm === this.target.size;
    this.finishIfApplied();
  }

  sceneSnapshot(object) {
    if (!this.target) return;
    if (object.kind !== this.target.kind) return this.clear();
    this.target.sceneMatches = object.size_mm === this.target.size;
    this.finishIfApplied();
  }

  finishIfApplied() {
    const target = this.target;
    if (target?.submitted && target.controlMatches && target.sceneMatches) this.clear();
  }

  ratio(object) {
    return this.target?.kind === object.kind && object.size_mm > 0 ? this.target.size / object.size_mm : 1;
  }

  clear() {
    clearTimeout(this.timer);
    if (!this.target) return;
    this.target = null;
    this.dispatchEvent(new Event('change'));
  }
}
export const sizePreview = new SizePreview();
