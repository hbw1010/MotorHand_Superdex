import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';

export class LocalGraspView {
  constructor(world, camera, canvas, controls) {
    this.world = world;
    this.camera = camera;
    this.canvas = canvas;
    this.controls = controls;
    this.ray = new THREE.Raycaster();
    this.plane = new THREE.Plane();
    this.point = new THREE.Vector3();
    this.offset = new THREE.Vector3();
    this.pointer = null;
    this.revision = -1;
    this.material = new THREE.MeshStandardMaterial({roughness: .51});
    this.loader = new GLTFLoader();
    this.models = new Map();
    canvas.addEventListener('pointerdown', event => this.begin(event), true);
    canvas.addEventListener('pointermove', event => this.move(event), true);
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      canvas.addEventListener(type, event => this.end(event), true);
    }
  }

  update(object) {
    this.object = object;
    if (this.revision !== object.mesh_revision) {
      this.revision = object.mesh_revision;
      if (this.mesh) {
        this.world.remove(this.mesh);
        this.mesh.geometry?.dispose();
      }
      if (object.parts) {
        this.mesh = new THREE.Group();
        for (const part of object.parts) this.mesh.add(this.loadPart(part));
        this.world.add(this.mesh);
      } else {
        this.mesh = this.createPrimitive(object);
        this.world.add(this.mesh);
      }
    }
    if (object.parts) {
      for (const [index, part] of object.parts.entries()) {
        const node = this.mesh.children[index];
        node.position.fromArray(part.pose);
        node.quaternion.fromArray(part.pose, 3);
      }
    } else if (object.vertices) {
      const geometry = this.mesh.geometry;
      const position = geometry.getAttribute('position');
      position.array.set(object.vertices);
      position.needsUpdate = true;
      geometry.computeVertexNormals();
      geometry.computeBoundingSphere();
    } else {
      this.mesh.position.fromArray(object.pose);
      this.mesh.quaternion.fromArray(object.pose, 3);
    }
  }

  createPrimitive(object) {
    const size = object.size_mm / 1000;
    let geometry;
    if (object.vertices) {
      geometry = new THREE.BufferGeometry();
      const attr = new THREE.Float32BufferAttribute(object.vertices, 3);
      attr.setUsage(THREE.DynamicDrawUsage);
      geometry.setAttribute('position', attr);
      geometry.setIndex(object.indices);
    } else if (object.kind.startsWith('box')) {
      geometry = new THREE.BoxGeometry(size, size, size);
    } else if (object.kind.startsWith('cylinder')) {
      geometry = new THREE.CylinderGeometry(size / 2, size / 2, size * 1.4, 32);
    } else {
      geometry = new THREE.SphereGeometry(size / 2, 32, 24);
    }
    this.material.color.set(object.kind.endsWith('_soft') ? '#38b8ca' : '#ef9d43');
    return new THREE.Mesh(geometry, this.material);
  }

  loadPart(part) {
    const node = new THREE.Group();
    const basis = new THREE.Group();
    basis.rotation.x = Math.PI / 2;
    const [x, y, z] = part.scale;
    basis.scale.set(x, z, y);
    node.add(basis);
    if (!this.models.has(part.model)) {
      const path = part.model.split('/').map(encodeURIComponent).join('/');
      this.models.set(part.model, this.loader.loadAsync(`/api/v1/grasp-model/${path}`));
    }
    this.models.get(part.model).then(({scene}) => basis.add(scene.clone(true))).catch(() => {
      this.models.delete(part.model);
    });
    return node;
  }

  aim(event) {
    const rect = this.canvas.getBoundingClientRect();
    this.ray.setFromCamera({x: (event.clientX - rect.left) * 2 / rect.width - 1,
      y: 1 - (event.clientY - rect.top) * 2 / rect.height}, this.camera);
  }

  begin(event) {
    if (event.button !== 0 || this.object?.layout !== 'desk' || this.pointer !== null) return;
    this.aim(event);
    const collision = this.ray.intersectObject(this.mesh, true)[0];
    if (!collision) return;
    const center = this.world.localToWorld(new THREE.Vector3().fromArray(this.object.pose));
    this.offset.copy(center).sub(collision.point);
    this.plane.setFromNormalAndCoplanarPoint(this.camera.getWorldDirection(this.point), collision.point);
    this.pointer = event.pointerId;
    this.controls.enabled = false;
    this.canvas.setPointerCapture(event.pointerId);
    event.stopImmediatePropagation();
  }

  move(event) {
    if (this.pointer !== event.pointerId) return;
    event.stopImmediatePropagation();
    this.aim(event);
    if (!this.ray.ray.intersectPlane(this.plane, this.point)) return;
    const target = this.world.worldToLocal(this.point.add(this.offset));
    this.send(target.toArray().map(value => Math.max(-.15, Math.min(.45, value))));
  }

  end(event) {
    if (this.pointer !== event.pointerId) return;
    this.pointer = null;
    this.controls.enabled = true;
    this.send(null);
  }

  async send(target) {
    this.queued = target;
    if (this.sending) return;
    this.sending = true;
    try {
      while (this.queued !== undefined) {
        const drag = this.queued;
        this.queued = undefined;
        await fetch('/api/v1/control', {method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({action: 'configure_grasp', values: {drag}}),
          signal: AbortSignal.timeout(700)}).catch(() => {});
      }
    } finally {
      this.sending = false;
    }
  }
}
