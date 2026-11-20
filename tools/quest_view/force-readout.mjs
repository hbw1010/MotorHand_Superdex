// 仅用于诊断卡的高亮和观察轴选择。数值仍取当前反馈帧。
export class ForceReadout {
  constructor() {
    this.loaded = [];
    this.axis = null;
  }

  update(nominal, sceneRevision, contactRevision) {
    if (sceneRevision !== this.sceneRevision || contactRevision !== this.contactRevision) {
      this.loaded = [];
      this.axis = null;
    }
    this.sceneRevision = sceneRevision;
    this.contactRevision = contactRevision;

    const magnitude = nominal.map(value => Number.isFinite(value) ? Math.abs(value) : 0);
    this.loaded = magnitude.flatMap((value, axis) =>
      value >= (this.loaded.includes(axis) ? .01 : .03) ? [axis] : []);
    const max = Math.max(0, ...magnitude);
    const current = this.axis === null ? 0 : magnitude[this.axis];
    if (max === 0) {
      this.axis = null;
    } else if (!current || max > current + Math.max(.03, current * .10) ||
               (this.loaded.length && !this.loaded.includes(this.axis))) {
      this.axis = magnitude.indexOf(max);
    }
    return {available: nominal.some(Number.isFinite), axis: this.axis, loaded: this.loaded};
  }
}
