// 旋转目标可在上一条请求传输期间更新，仅提交最新角度。
export class RotationControl {
  constructor(send) {
    this.send = send;
    this.pending = null;
    this.inFlight = null;
    this.target = null;
    this.timer = null;
    this.nextSend = 0;
  }

  update(kind, angle, final = false) {
    if (!this.target || kind !== this.target.kind || angle !== this.target.angle) {
      this.target = this.pending = {kind, angle};
    }
    if (final) this.nextSend = 0;
    this.flush();
  }

  flush() {
    clearTimeout(this.timer);
    if (this.inFlight || !this.pending) return;
    const delay = this.nextSend - performance.now();
    if (delay > 0) {
      this.timer = setTimeout(() => this.flush(), delay);
      return;
    }
    const target = this.pending;
    this.pending = null;
    this.nextSend = performance.now() + 40;
    this.inFlight = this.send(target.kind, target.angle).then(ok => {
      if (this.target === target) {
        // 控制快照以较低频率发布，等待已提交的最终值被服务端回显。
        if (ok) target.expiresAt = performance.now() + 900;
        else this.target = null;
      }
    }).finally(() => {
      this.inFlight = null;
      this.flush();
    });
  }

  waitingForEcho(kind, angle) {
    if (this.target?.kind !== kind) return false;
    const target = this.target;
    if (target.expiresAt === undefined) return true;
    if (target.angle !== angle && performance.now() < target.expiresAt) return true;
    this.target = null;
    return false;
  }

  cancel() {
    clearTimeout(this.timer);
    this.target = this.pending = null;
    return this.inFlight;
  }
}
