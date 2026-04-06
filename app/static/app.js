import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";

const els = {
  gcodeInput: document.getElementById("gcodeInput"),
  runBtn: document.getElementById("runBtn"),
  clearPathBtn: document.getElementById("clearPathBtn"),
  meshInput: document.getElementById("meshInput"),
  meshList: document.getElementById("meshList"),
  selectedMesh: document.getElementById("selectedMesh"),
  meshTx: document.getElementById("meshTx"),
  meshTy: document.getElementById("meshTy"),
  meshTz: document.getElementById("meshTz"),
  meshRx: document.getElementById("meshRx"),
  meshRy: document.getElementById("meshRy"),
  meshRz: document.getElementById("meshRz"),
  applyMeshTranslateBtn: document.getElementById("applyMeshTranslateBtn"),
  applyMeshRotateBtn: document.getElementById("applyMeshRotateBtn"),
  deleteMeshBtn: document.getElementById("deleteMeshBtn"),
  status: document.getElementById("status"),
  serial: document.getElementById("serial"),
  renderRoot: document.getElementById("renderRoot"),
};

const serialBuffer = [];
const meshVisuals = new Map();
const meshMeta = new Map();
let lastState = null;
let activeMotion = null;
const pendingMotions = [];
let selectedMeshName = null;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c1614);
scene.up.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(
  55,
  els.renderRoot.clientWidth / Math.max(1, els.renderRoot.clientHeight),
  0.1,
  2000
);
camera.position.set(260, -220, 220);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(els.renderRoot.clientWidth, els.renderRoot.clientHeight);
els.renderRoot.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 30);
// Keep orbit interaction stable by avoiding the singular pole orientations.
controls.minPolarAngle = 0.03;
controls.maxPolarAngle = Math.PI - 0.03;
controls.update();

scene.add(new THREE.HemisphereLight(0xb8ffd8, 0x122722, 0.8));
const sun = new THREE.DirectionalLight(0xffffff, 0.9);
sun.position.set(200, -150, 280);
scene.add(sun);

const grid = new THREE.GridHelper(220, 22, 0x77d19a, 0x355b4a);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

const xAxisLine = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-120, 0, 0.02), new THREE.Vector3(120, 0, 0.02)]),
  new THREE.LineBasicMaterial({ color: 0xff6b6b })
);
scene.add(xAxisLine);

const yAxisLine = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, -120, 0.02), new THREE.Vector3(0, 120, 0.02)]),
  new THREE.LineBasicMaterial({ color: 0x5bc0be })
);
scene.add(yAxisLine);

function createAxisLabel(text, color) {
  const canvas = document.createElement("canvas");
  canvas.width = 128;
  canvas.height = 64;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "rgba(0, 0, 0, 0.35)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = color;
  ctx.font = "bold 28px monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, canvas.width / 2, canvas.height / 2);

  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(9, 4.5, 1);
  return sprite;
}

const scaleLabels = new THREE.Group();
for (let tick = -100; tick <= 100; tick += 20) {
  const xLabel = createAxisLabel(`${tick}`, "#ffd7d7");
  xLabel.position.set(tick, -4, 0.5);
  scaleLabels.add(xLabel);

  const yLabel = createAxisLabel(`${tick}`, "#d8ffff");
  yLabel.position.set(4, tick, 0.5);
  scaleLabels.add(yLabel);
}
scene.add(scaleLabels);

// Draw an explicit XYZ vector triad at machine origin (0, 0, 0).
const originAxes = new THREE.AxesHelper(45);
originAxes.position.set(0, 0, 0);
scene.add(originAxes);

const originMarker = new THREE.Mesh(
  new THREE.SphereGeometry(1.9, 12, 12),
  new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x223333, emissiveIntensity: 0.5 })
);
originMarker.position.set(0, 0, 0);
scene.add(originMarker);

const bed = new THREE.Mesh(
  new THREE.BoxGeometry(220, 220, 2),
  new THREE.MeshStandardMaterial({ color: 0x1f3a33, metalness: 0.2, roughness: 0.85 })
);
bed.position.set(0, 0, -1);
scene.add(bed);

const toolhead = new THREE.Group();
const arrowFrame = new THREE.Group();

const ARROW_HEAD_LENGTH = 9.5;
const ARROW_SHAFT_LENGTH = 16;

const arrowShaft = new THREE.Mesh(
  new THREE.CylinderGeometry(1.7, 1.7, ARROW_SHAFT_LENGTH, 16),
  new THREE.MeshStandardMaterial({ color: 0x85f5b1, metalness: 0.35, roughness: 0.3 })
);
arrowShaft.rotation.z = -Math.PI / 2;
arrowShaft.position.x = -(ARROW_HEAD_LENGTH + (ARROW_SHAFT_LENGTH / 2));
arrowFrame.add(arrowShaft);

const arrowHead = new THREE.Mesh(
  new THREE.ConeGeometry(3.8, ARROW_HEAD_LENGTH, 18),
  new THREE.MeshStandardMaterial({ color: 0xf0fff8, metalness: 0.15, roughness: 0.4 })
);
arrowHead.rotation.z = -Math.PI / 2;
// Tip is anchored at local origin so XYZ coordinates match the visible tip.
arrowHead.position.x = -(ARROW_HEAD_LENGTH / 2);
arrowFrame.add(arrowHead);

const arrowHub = new THREE.Mesh(
  new THREE.SphereGeometry(2.5, 16, 16),
  new THREE.MeshStandardMaterial({ color: 0xffcf7f, metalness: 0.2, roughness: 0.45 })
);
arrowHub.position.x = -ARROW_HEAD_LENGTH;
arrowFrame.add(arrowHub);

// Point the arrow 45 degrees downward; toolhead Z rotation still follows A-axis heading.
arrowFrame.rotation.y = Math.PI / 4;
toolhead.add(arrowFrame);

scene.add(toolhead);

let pathLine = null;

function mapAAxisToSceneRotation(aDeg) {
  // Make positive machine A rotate clockwise when viewed from +Z.
  return -aDeg;
}

function setToolheadPose(x, y, z, a) {
  toolhead.position.set(x, y, z);
  toolhead.rotation.z = THREE.MathUtils.degToRad(mapAAxisToSceneRotation(a));
}

function setStatus(state) {
  els.status.textContent = JSON.stringify(state, null, 2);
}

function appendSerial(line) {
  serialBuffer.push(line);
  while (serialBuffer.length > 240) {
    serialBuffer.shift();
  }
  els.serial.textContent = serialBuffer.join("\n");
  els.serial.scrollTop = els.serial.scrollHeight;
}

function echoInputGcode(text) {
  const lines = text
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter((line) => line.length > 0);

  for (const line of lines) {
    appendSerial(`> ${line}`);
  }
}

function updatePath(points) {
  if (pathLine) {
    scene.remove(pathLine);
    pathLine.geometry.dispose();
    pathLine.material.dispose();
    pathLine = null;
  }

  if (!points || points.length < 2) {
    return;
  }

  const verts = points.map((p) => new THREE.Vector3(p.x, p.y, p.z));
  const geo = new THREE.BufferGeometry().setFromPoints(verts);
  const mat = new THREE.LineBasicMaterial({ color: 0x7be495 });
  pathLine = new THREE.Line(geo, mat);
  scene.add(pathLine);
}

function applyState(state) {
  lastState = state;
  const p = state.position;

  const motionInFlight = activeMotion !== null || pendingMotions.length > 0;
  if (!motionInFlight) {
    setToolheadPose(p.X, p.Y, p.Z, p.A);
  }

  updatePath(state.path);
  setStatus(state);
}

function displacementFromProfile(profile, t) {
  const distance = profile?.distance ?? 0;
  if (distance <= 0) {
    return 0;
  }

  const accel = profile.accel;
  const tAccel = profile.t_accel;
  const tCruise = profile.t_cruise;
  const tTotal = profile.t_total;
  const vCruise = profile.cruise_speed;

  const tc = Math.max(0, Math.min(t, tTotal));
  if (tc <= tAccel) {
    return 0.5 * accel * tc * tc;
  }

  const dAccel = 0.5 * accel * tAccel * tAccel;
  if (tc <= tAccel + tCruise) {
    return dAccel + vCruise * (tc - tAccel);
  }

  const td = tc - tAccel - tCruise;
  const dBeforeDecel = dAccel + vCruise * tCruise;
  const d = dBeforeDecel + vCruise * td - 0.5 * accel * td * td;
  return Math.max(0, Math.min(distance, d));
}

function startMotion(motion) {
  pendingMotions.push(motion);
  if (!activeMotion) {
    activateNextMotion(performance.now());
  }
}

function activateNextMotion(nowMs) {
  while (pendingMotions.length > 0) {
    const nextMotion = pendingMotions.shift();
    const duration = Number(nextMotion?.duration_s || 0);
    if (!Number.isFinite(duration) || duration <= 1e-6) {
      const end = nextMotion?.end;
      if (end) {
        setToolheadPose(end.X, end.Y, end.Z, end.A);
      }
      continue;
    }

    activeMotion = {
      motion: nextMotion,
      startedAtMs: nowMs,
    };
    return;
  }

  activeMotion = null;
}

function updateMotion(nowMs) {
  if (!activeMotion && pendingMotions.length > 0) {
    activateNextMotion(nowMs);
  }

  if (!activeMotion) {
    return;
  }

  const { motion, startedAtMs } = activeMotion;
  const elapsed = (nowMs - startedAtMs) / 1000;
  const t = Math.max(0, Math.min(elapsed, motion.duration_s || 0));

  const start = motion.start;
  const linear = motion.linear || {};
  const rotary = motion.rotary || {};

  const linearDisp = displacementFromProfile(linear, t);
  const linearDistance = linear.distance || 0;
  let x = start.X;
  let y = start.Y;
  let z = start.Z;
  if (linearDistance > 1e-9) {
    x += (linear.dx / linearDistance) * linearDisp;
    y += (linear.dy / linearDistance) * linearDisp;
    z += (linear.dz / linearDistance) * linearDisp;
  }

  const rotaryDisp = displacementFromProfile(rotary, t);
  const da = rotary.da || 0;
  let a = start.A;
  if (Math.abs(da) > 1e-9) {
    a += Math.sign(da) * rotaryDisp;
  }

  setToolheadPose(x, y, z, a);

  if (elapsed >= (motion.duration_s || 0)) {
    if (motion.end) {
      setToolheadPose(motion.end.X, motion.end.Y, motion.end.Z, motion.end.A);
    }
    activeMotion = null;
    activateNextMotion(nowMs);
  }
}

async function queueGcode(text) {
  const response = await fetch("/api/gcode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ gcode: text }),
  });
  if (!response.ok) {
    const t = await response.text();
    appendSerial(`host:error ${t}`);
    return;
  }
  const data = await response.json();
  appendSerial(`host:queued ${data.queued} line(s)`);
}

async function uploadMesh(file) {
  const fd = new FormData();
  fd.append("file", file);

  const response = await fetch("/api/mesh", {
    method: "POST",
    body: fd,
  });

  if (!response.ok) {
    const text = await response.text();
    appendSerial(`host:mesh upload failed ${text}`);
    return;
  }

  await refreshMeshList();
}

function applyTransformToObject(object, transform) {
  if (!object || !transform) {
    return;
  }
  object.position.set(transform.tx || 0, transform.ty || 0, transform.tz || 0);
  object.rotation.set(
    THREE.MathUtils.degToRad(transform.rx || 0),
    THREE.MathUtils.degToRad(transform.ry || 0),
    THREE.MathUtils.degToRad(transform.rz || 0)
  );
}

function removeMeshVisual(name) {
  const object = meshVisuals.get(name);
  if (!object) {
    return;
  }

  if (object.traverse) {
    object.traverse((node) => {
      if (node.isMesh) {
        node.geometry?.dispose?.();
      }
    });
  }
  scene.remove(object);
  meshVisuals.delete(name);
}

function loadMeshVisual(name, transform) {
  if (meshVisuals.has(name)) {
    applyTransformToObject(meshVisuals.get(name), transform);
    return;
  }

  const ext = name.split(".").pop().toLowerCase();
  const url = `/meshes/${encodeURIComponent(name)}`;

  const material = new THREE.MeshStandardMaterial({
    color: 0x91c4ff,
    transparent: true,
    opacity: 0.55,
    metalness: 0.05,
    roughness: 0.8,
    side: THREE.DoubleSide,
  });

  if (ext === "stl") {
    const loader = new STLLoader();
    loader.load(url, (geometry) => {
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, material);
      mesh.userData.meshName = name;
      applyTransformToObject(mesh, transform);
      scene.add(mesh);
      meshVisuals.set(name, mesh);
    });
    return;
  }

  if (ext === "obj") {
    const loader = new OBJLoader();
    loader.load(url, (object) => {
      object.traverse((node) => {
        if (node.isMesh) {
          node.material = material;
        }
      });
      object.userData.meshName = name;
      applyTransformToObject(object, transform);
      scene.add(object);
      meshVisuals.set(name, object);
    });
    return;
  }

  if (ext === "ply") {
    const loader = new PLYLoader();
    loader.load(url, (geometry) => {
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, material);
      mesh.userData.meshName = name;
      applyTransformToObject(mesh, transform);
      scene.add(mesh);
      meshVisuals.set(name, mesh);
    });
  }
}

function setSelectedMesh(name) {
  selectedMeshName = name || null;

  els.meshList.querySelectorAll("li").forEach((li) => {
    li.classList.toggle("selected", li.dataset.name === selectedMeshName);
  });

  if (selectedMeshName && meshMeta.has(selectedMeshName)) {
    const m = meshMeta.get(selectedMeshName);
    const t = m.transform || { tx: 0, ty: 0, tz: 0, rx: 0, ry: 0, rz: 0 };
    els.selectedMesh.value = selectedMeshName;
    els.meshTx.value = t.tx ?? 0;
    els.meshTy.value = t.ty ?? 0;
    els.meshTz.value = t.tz ?? 0;
    els.meshRx.value = t.rx ?? 0;
    els.meshRy.value = t.ry ?? 0;
    els.meshRz.value = t.rz ?? 0;
  }
}

async function patchSelectedMesh(payload, actionLabel) {
  if (!selectedMeshName) {
    appendSerial("host:select a mesh first");
    return;
  }

  const response = await fetch(`/api/mesh/${encodeURIComponent(selectedMeshName)}`, {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    appendSerial(`host:error mesh ${actionLabel} failed`);
    return;
  }

  const updated = await response.json();
  meshMeta.set(updated.name, updated);
  loadMeshVisual(updated.name, updated.transform);
  setSelectedMesh(updated.name);
  appendSerial(`host:mesh ${actionLabel} ${updated.name}`);
}

async function applySelectedMeshTranslation() {
  await patchSelectedMesh(
    {
      tx: Number.parseFloat(els.meshTx.value) || 0,
      ty: Number.parseFloat(els.meshTy.value) || 0,
      tz: Number.parseFloat(els.meshTz.value) || 0,
    },
    "translated"
  );
}

async function applySelectedMeshRotation() {
  await patchSelectedMesh(
    {
      rx: Number.parseFloat(els.meshRx.value) || 0,
      ry: Number.parseFloat(els.meshRy.value) || 0,
      rz: Number.parseFloat(els.meshRz.value) || 0,
    },
    "rotated"
  );
}

async function deleteSelectedMesh() {
  if (!selectedMeshName) {
    appendSerial("host:select a mesh first");
    return;
  }

  const response = await fetch(`/api/mesh/${encodeURIComponent(selectedMeshName)}`, {
    method: "DELETE",
  });

  if (!response.ok) {
    appendSerial("host:error mesh delete failed");
    return;
  }

  removeMeshVisual(selectedMeshName);
  meshMeta.delete(selectedMeshName);
  selectedMeshName = null;
  await refreshMeshList();
  appendSerial("host:mesh deleted");
}

async function refreshMeshList() {
  const response = await fetch("/api/meshes");
  if (!response.ok) {
    return;
  }

  const meshes = await response.json();
  const names = new Set(meshes.map((m) => m.name));

  for (const [name] of meshVisuals) {
    if (!names.has(name)) {
      removeMeshVisual(name);
    }
  }

  meshMeta.clear();
  els.meshList.innerHTML = "";
  els.selectedMesh.innerHTML = "";

  meshes.forEach((m) => {
    meshMeta.set(m.name, m);

    const li = document.createElement("li");
    li.textContent = `${m.name} (${m.vertices}v)`;
    li.dataset.name = m.name;
    li.addEventListener("click", () => setSelectedMesh(m.name));
    els.meshList.appendChild(li);

    const opt = document.createElement("option");
    opt.value = m.name;
    opt.textContent = m.name;
    els.selectedMesh.appendChild(opt);

    loadMeshVisual(m.name, m.transform);
  });

  if (!meshes.length) {
    selectedMeshName = null;
    return;
  }

  const next = meshes.some((m) => m.name === selectedMeshName) ? selectedMeshName : meshes[0].name;
  setSelectedMesh(next);
}

function connectWs() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${protocol}://${window.location.host}/ws`);

  ws.onmessage = (event) => {
    const payload = JSON.parse(event.data);
    if (payload.type === "serial") {
      appendSerial(payload.message);
    }
    if (payload.type === "state") {
      applyState(payload.state);
    }
    if (payload.type === "motion") {
      startMotion(payload.motion);
    }
  };

  ws.onclose = () => {
    appendSerial("host:ws disconnected; retrying...");
    setTimeout(connectWs, 1000);
  };
}

function animate() {
  updateMotion(performance.now());
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

window.addEventListener("resize", () => {
  const w = els.renderRoot.clientWidth;
  const h = Math.max(1, els.renderRoot.clientHeight);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
});

els.runBtn.addEventListener("click", async () => {
  const text = els.gcodeInput.value;
  echoInputGcode(text);
  await queueGcode(text);
});

async function clearMovementLines() {
  const response = await fetch("/api/path/clear", { method: "POST" });
  if (!response.ok) {
    appendSerial("host:error unable to clear path");
    return;
  }
  appendSerial("host:path cleared");
}

els.clearPathBtn?.addEventListener("click", clearMovementLines);

els.meshInput.addEventListener("change", async (event) => {
  const [file] = event.target.files;
  if (!file) {
    return;
  }
  await uploadMesh(file);
  event.target.value = "";
});

els.selectedMesh?.addEventListener("change", () => {
  setSelectedMesh(els.selectedMesh.value || null);
});

els.applyMeshTranslateBtn?.addEventListener("click", applySelectedMeshTranslation);
els.applyMeshRotateBtn?.addEventListener("click", applySelectedMeshRotation);
els.deleteMeshBtn?.addEventListener("click", deleteSelectedMesh);

(async function init() {
  const response = await fetch("/api/state");
  if (response.ok) {
    applyState(await response.json());
  }

  await refreshMeshList();
  connectWs();
  animate();

  if (!lastState) {
    appendSerial("host:waiting for simulator state...");
  }
})();
