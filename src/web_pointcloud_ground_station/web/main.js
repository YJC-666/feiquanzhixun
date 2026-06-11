import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

const ROSLIB = window.ROSLIB;

const DEFAULTS = {
  topics: {
    cloud: "/pointlio/cloud_registered",
    odom: "/pointlio/odom",
    camera: "/camera/image/compressed",
  },
  render: {
    recentFrames: 16,
    maxRawPerFrame: 20000,
    keyframeHistory: 12,
    keyframeBudget: 1200000,
    keyframeCell: 0.015,
    renderBudget: 300000,
    voxelSize: 0.1,
    outlierRadius: 35.0,
    zMax: 3.5,
    depthColorMin: -0.3,
    depthColorMax: 3.0,
    alpha: 0.82,
    wireAlpha: 0.55,
  },
  motion: {
    stationaryDistance: 0.1,
    stationaryTimeMs: 3000,
  },
  camera: {
    followDistance: 2.0,
    targetZOffset: 0.3,
  },
  robotModel: {
    positionResolution: 0.05,
  },
};

const DATATYPE = {
  INT8: 1,
  UINT8: 2,
  INT16: 3,
  UINT16: 4,
  INT32: 5,
  UINT32: 6,
  FLOAT32: 7,
  FLOAT64: 8,
};

const $ = (id) => document.getElementById(id);

const ui = {
  canvas: $("viewport"),
  connPill: $("conn-pill"),
  wsUrl: $("ws-url"),
  connectBtn: $("connect-btn"),
  disconnectBtn: $("disconnect-btn"),
  cloudTopic: $("cloud-topic"),
  odomTopic: $("odom-topic"),
  cameraTopic: $("camera-topic"),
  applyTopicsBtn: $("apply-topics-btn"),
  cameraFrame: document.querySelector(".camera-frame"),
  cameraFeed: $("camera-feed"),
  detectionFeed: $("detection-feed"),
  visibleVoxels: $("visible-voxels"),
  cameraMode: $("camera-mode"),
  robotPosition: $("robot-position"),
  frameCount: $("frame-count"),
  rawPerFrame: $("raw-per-frame"),
  recentPoints: $("recent-points"),
  keyframePoints: $("keyframe-points"),
  rawTotal: $("raw-total"),
  odomState: $("odom-state"),
  followToggle: $("follow-toggle"),
  frontCullToggle: $("front-cull-toggle"),
  wireToggle: $("wire-toggle"),
  resetViewBtn: $("reset-view-btn"),
  saveViewBtn: $("save-view-btn"),
  voxelSize: $("voxel-size"),
  voxelSizeValue: $("voxel-size-value"),
  renderBudget: $("render-budget"),
  renderBudgetValue: $("render-budget-value"),
  alpha: $("alpha"),
  alphaValue: $("alpha-value"),
  zMax: $("z-max"),
  zMaxValue: $("z-max-value"),
  eventLog: $("event-log"),
  safetyStatus: $("safety-status"),
};

const state = {
  ros: null,
  topics: { ...DEFAULTS.topics },
  subscriptions: [],
  recentFrames: [],
  keyframe: [],
  keyframeCounter: 0,
  stats: {
    frames: 0,
    rawPerFrame: 0,
    rawTotal: 0,
    visibleVoxels: 0,
  },
  robot: {
    hasOdom: false,
    x: 0,
    y: 0,
    z: 0,
    qx: 0,
    qy: 0,
    qz: 0,
    qw: 1,
    fwdX: 1,
    fwdY: 0,
    fwdZ: 0,
    lastMoveX: 0,
    lastMoveY: 0,
    lastMoveZ: 0,
    lastMoveAt: performance.now(),
  },
  render: { ...DEFAULTS.render },
  follow: true,
  frontCull: true,
  showWire: true,
  connected: false,
  needsRenderUpdate: false,
  localPlanner: {
    group: null,
  },
};

function defaultRosbridgeUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const host = window.location.hostname || "localhost";
  return `${protocol}://${host}:9090`;
}

function logEvent(message) {
  const row = document.createElement("div");
  const now = new Date().toLocaleTimeString();
  row.textContent = `[${now}] ${message}`;
  ui.eventLog.prepend(row);
  while (ui.eventLog.children.length > 80) {
    ui.eventLog.removeChild(ui.eventLog.lastChild);
  }
}

function setConnectionState(mode, text) {
  ui.connPill.classList.remove("online", "offline", "connecting");
  ui.connPill.classList.add(mode);
  ui.connPill.textContent = text;
}

async function loadRuntimeConfig() {
  try {
    const response = await fetch("./runtime-config.json", { cache: "no-store" });
    if (!response.ok) return;
    const config = await response.json();
    applyRuntimeConfig(config);
    logEvent("已加载 ROS 运行配置");
  } catch (error) {
    logEvent(`运行配置未加载：${error?.message || error}`);
  }
}

function applyRuntimeConfig(config) {
  if (!config || typeof config !== "object") return;

  if (config.topics && typeof config.topics === "object") {
    DEFAULTS.topics.cloud = String(config.topics.cloud || DEFAULTS.topics.cloud);
    DEFAULTS.topics.odom = String(config.topics.odom || DEFAULTS.topics.odom);
    DEFAULTS.topics.camera = String(config.topics.camera || DEFAULTS.topics.camera);
    state.topics = { ...DEFAULTS.topics };
  }

  const renderMap = {
    recent_frames: "recentFrames",
    max_raw_per_frame: "maxRawPerFrame",
    keyframe_history: "keyframeHistory",
    keyframe_budget: "keyframeBudget",
    keyframe_cell: "keyframeCell",
    render_budget: "renderBudget",
    voxel_size: "voxelSize",
    outlier_radius: "outlierRadius",
    z_max: "zMax",
    depth_color_min: "depthColorMin",
    depth_color_max: "depthColorMax",
    point_alpha: "alpha",
    wire_alpha: "wireAlpha",
  };
  applyMappedNumbers(config.render, renderMap, DEFAULTS.render, state.render);

  const motionMap = {
    stationary_distance: "stationaryDistance",
  };
  applyMappedNumbers(config.motion, motionMap, DEFAULTS.motion);
  if (Number.isFinite(Number(config.motion?.stationary_time))) {
    DEFAULTS.motion.stationaryTimeMs = Number(config.motion.stationary_time) * 1000;
  }

  const cameraMap = {
    follow_distance: "followDistance",
    target_z_offset: "targetZOffset",
  };
  applyMappedNumbers(config.camera, cameraMap, DEFAULTS.camera);
}

function applyMappedNumbers(source, map, ...targets) {
  if (!source || typeof source !== "object") return;
  Object.entries(map).forEach(([sourceKey, targetKey]) => {
    const value = Number(source[sourceKey]);
    if (!Number.isFinite(value)) return;
    targets.forEach((target) => {
      if (target) target[targetKey] = value;
    });
  });
}

function loadUiDefaults() {
  ui.wsUrl.value = localStorage.getItem("web-gs.ws") || defaultRosbridgeUrl();
  ui.cloudTopic.value = localStorage.getItem("web-gs.cloudTopic") || state.topics.cloud;
  ui.odomTopic.value = localStorage.getItem("web-gs.odomTopic") || state.topics.odom;
  ui.cameraTopic.value = localStorage.getItem("web-gs.cameraTopic") || state.topics.camera;

  state.topics.cloud = ui.cloudTopic.value.trim() || DEFAULTS.topics.cloud;
  state.topics.odom = ui.odomTopic.value.trim() || DEFAULTS.topics.odom;
  state.topics.camera = ui.cameraTopic.value.trim() || DEFAULTS.topics.camera;

  syncRenderControls();
}

function syncRenderControls() {
  ui.voxelSize.value = Math.round(state.render.voxelSize * 100).toString();
  ui.renderBudget.value = state.render.renderBudget.toString();
  ui.alpha.value = Math.round(state.render.alpha * 100).toString();
  ui.zMax.value = Math.round(state.render.zMax * 100).toString();
  updateControlLabels();
}

function updateControlLabels() {
  ui.voxelSizeValue.textContent = `${state.render.voxelSize.toFixed(2)}m`;
  ui.renderBudgetValue.textContent = String(state.render.renderBudget);
  ui.alphaValue.textContent = state.render.alpha.toFixed(2);
  ui.zMaxValue.textContent = `${state.render.zMax.toFixed(1)}m`;
}

const renderer = new THREE.WebGLRenderer({ canvas: ui.canvas, antialias: true, powerPreference: "high-performance" });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x02080d, 1);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x02080d, 12, 80);

const camera = new THREE.PerspectiveCamera(55, 1, 0.02, 220);
camera.up.set(0, 0, 1);
camera.position.set(3.8, -5.2, 3.2);

const controls = new OrbitControls(camera, ui.canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, DEFAULTS.camera.targetZOffset);
controls.maxPolarAngle = Math.PI * 0.49;
controls.minDistance = 0.35;
controls.maxDistance = 80;

scene.add(new THREE.AmbientLight(0xffffff, 0.65));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.25);
keyLight.position.set(8, -6, 12);
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight(0xffffff, 0.55);
fillLight.position.set(-6, 4, -5);
scene.add(fillLight);

const grid = new THREE.GridHelper(24, 48, 0x31515f, 0x172832);
grid.rotation.x = Math.PI * 0.5;
scene.add(grid);

const axes = new THREE.AxesHelper(1.2);
scene.add(axes);

const robotGroup = buildRobotMarker();
const plannerVizGroup = new THREE.Group();
plannerVizGroup.name = "plannerViz";
robotGroup.add(plannerVizGroup);
scene.add(robotGroup);

const cubeGeometry = new THREE.BoxGeometry(1, 1, 1);
let voxelMeshes = [];
let wireMesh = null;

function buildRobotMarker() {
  const group = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.48, 0.22, 0.14),
    new THREE.MeshStandardMaterial({ color: 0xeaf6fb, roughness: 0.36, metalness: 0.12 }),
  );
  body.position.set(0, 0, 0.07);
  const head = new THREE.Mesh(
    new THREE.BoxGeometry(0.16, 0.14, 0.12),
    new THREE.MeshStandardMaterial({ color: 0x111820, roughness: 0.5, metalness: 0.18 }),
  );
  head.position.set(0.32, 0, 0.09);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.18), 0.65, 0xff4d5a, 0.18, 0.08);
  group.add(body, head, arrow);
  return group;
}

function resizeRenderer() {
  const width = ui.canvas.clientWidth || 1;
  const height = ui.canvas.clientHeight || 1;
  const needsResize = ui.canvas.width !== Math.floor(width * renderer.getPixelRatio()) ||
    ui.canvas.height !== Math.floor(height * renderer.getPixelRatio());
  if (needsResize) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

function connectRos() {
  if (!ROSLIB) {
    setConnectionState("offline", "缺少库");
    logEvent("ROSLIB 未加载，无法连接 ROSBridge");
    return;
  }

  disconnectRos(false);
  const url = ui.wsUrl.value.trim() || defaultRosbridgeUrl();
  localStorage.setItem("web-gs.ws", url);
  setConnectionState("connecting", "连接中");
  logEvent(`连接 ${url}`);

  const ros = new ROSLIB.Ros({ url });
  state.ros = ros;

  ros.on("connection", () => {
    state.connected = true;
    setConnectionState("online", "在线");
    logEvent("ROSBridge 已连接");
    subscribeAll();
  });

  ros.on("error", (error) => {
    state.connected = false;
    setConnectionState("offline", "错误");
    logEvent(`ROSBridge 错误：${error?.message || error}`);
  });

  ros.on("close", () => {
    state.connected = false;
    setConnectionState("offline", "离线");
    clearSubscriptions();
    logEvent("ROSBridge 已断开");
  });
}

function disconnectRos(report = true) {
  clearSubscriptions();
  if (state.ros) {
    try {
      state.ros.close();
    } catch (_) {
      // no-op
    }
  }
  state.ros = null;
  state.connected = false;
  setConnectionState("offline", "离线");
  if (report) logEvent("已手动断开");
}

function clearSubscriptions() {
  state.subscriptions.forEach((topic) => {
    try {
      topic.unsubscribe();
    } catch (_) {
      // no-op
    }
  });
  state.subscriptions = [];
}

function subscribeAll() {
  if (!state.ros) return;
  clearSubscriptions();

  const cloudTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: state.topics.cloud,
    messageType: "sensor_msgs/PointCloud2",
    throttle_rate: 120,
    queue_length: 1,
  });
  cloudTopic.subscribe(handlePointCloud);

  const odomTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: state.topics.odom,
    messageType: "nav_msgs/Odometry",
    throttle_rate: 50,
    queue_length: 1,
  });
  odomTopic.subscribe(handleOdometry);

  const cameraTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: state.topics.camera,
    messageType: "sensor_msgs/CompressedImage",
    throttle_rate: 160,
    queue_length: 1,
  });
  cameraTopic.subscribe(handleCameraImage);

  const detectionTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/web/camera/detection/compressed",
    messageType: "sensor_msgs/CompressedImage",
    throttle_rate: 160,
    queue_length: 1,
  });
  detectionTopic.subscribe(handleDetectionImage);

  const candidatesTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/local_planner_candidates",
    messageType: "visualization_msgs/MarkerArray",
    throttle_rate: 30,
    queue_length: 1,
  });
  candidatesTopic.subscribe(handleCandidates);

  const safetyBoxTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/local_planner_safety_box",
    messageType: "visualization_msgs/Marker",
    throttle_rate: 30,
    queue_length: 1,
  });
  safetyBoxTopic.subscribe(handleSafetyBox);

  const statusTopic = new ROSLIB.Topic({
    ros: state.ros,
    name: "/dog_safety_mux/status",
    messageType: "std_msgs/String",
    throttle_rate: 10,
    queue_length: 1,
  });
  statusTopic.subscribe(handleSafetyStatus);

  state.subscriptions = [cloudTopic, odomTopic, cameraTopic, detectionTopic, candidatesTopic, safetyBoxTopic, statusTopic];
  logEvent(`订阅 ${state.topics.cloud} / ${state.topics.odom} / ${state.topics.camera}`);
}

function applyTopics() {
  state.topics.cloud = ui.cloudTopic.value.trim() || DEFAULTS.topics.cloud;
  state.topics.odom = ui.odomTopic.value.trim() || DEFAULTS.topics.odom;
  state.topics.camera = ui.cameraTopic.value.trim() || DEFAULTS.topics.camera;
  localStorage.setItem("web-gs.cloudTopic", state.topics.cloud);
  localStorage.setItem("web-gs.odomTopic", state.topics.odom);
  localStorage.setItem("web-gs.cameraTopic", state.topics.camera);
  if (state.connected) subscribeAll();
}

function handleOdometry(msg) {
  const pose = msg.pose?.pose;
  if (!pose) return;

  const next = {
    x: Number(pose.position.x) || 0,
    y: Number(pose.position.y) || 0,
    z: Number(pose.position.z) || 0,
    qx: Number(pose.orientation.x) || 0,
    qy: Number(pose.orientation.y) || 0,
    qz: Number(pose.orientation.z) || 0,
    qw: Number(pose.orientation.w) || 1,
  };

  const dx = next.x - state.robot.lastMoveX;
  const dy = next.y - state.robot.lastMoveY;
  const dz = next.z - state.robot.lastMoveZ;
  const movedSq = dx * dx + dy * dy + dz * dz;
  if (!state.robot.hasOdom || movedSq > DEFAULTS.motion.stationaryDistance ** 2) {
    state.robot.lastMoveX = next.x;
    state.robot.lastMoveY = next.y;
    state.robot.lastMoveZ = next.z;
    state.robot.lastMoveAt = performance.now();
  }

  Object.assign(state.robot, next);
  state.robot.fwdX = 1 - 2 * (next.qy * next.qy + next.qz * next.qz);
  state.robot.fwdY = 2 * (next.qx * next.qy + next.qw * next.qz);
  state.robot.fwdZ = 2 * (next.qx * next.qz - next.qw * next.qy);
  state.robot.hasOdom = true;

  const visualPose = robotVisualPose(next);
  robotGroup.position.set(visualPose.x, visualPose.y, visualPose.z);
  robotGroup.quaternion.set(next.qx, next.qy, next.qz, next.qw);
}

function snapToRobotResolution(value) {
  const step = DEFAULTS.robotModel.positionResolution;
  return Math.round(value / step) * step;
}

function robotVisualPose(robot = state.robot) {
  return {
    x: snapToRobotResolution(robot.x || 0),
    y: snapToRobotResolution(robot.y || 0),
    z: snapToRobotResolution(robot.z || 0),
  };
}

function handleCameraImage(msg) {
  const bytes = normalizeBinaryData(msg.data);
  if (!bytes || bytes.length === 0) return;
  const mime = String(msg.format || "jpeg").toLowerCase().includes("png") ? "image/png" : "image/jpeg";
  ui.cameraFeed.src = `data:${mime};base64,${uint8ToBase64(bytes)}`;
  ui.cameraFrame.classList.add("active");
}

function handleDetectionImage(msg) {
  const bytes = normalizeBinaryData(msg.data);
  if (!bytes || bytes.length === 0) return;
  const mime = String(msg.format || "jpeg").toLowerCase().includes("png") ? "image/png" : "image/jpeg";
  ui.detectionFeed.src = `data:${mime};base64,${uint8ToBase64(bytes)}`;
}

function handlePointCloud(msg) {
  const points = parsePointCloud2(msg);
  if (!points.length) return;

  if (shouldSkipForStationaryGate()) {
    return;
  }

  state.stats.frames += 1;
  state.stats.rawPerFrame = points.length;
  state.stats.rawTotal += points.length;

  if (state.recentFrames.length >= DEFAULTS.render.recentFrames) {
    const evicted = state.recentFrames.pop();
    mergeIntoKeyframe(evicted);
  }
  state.recentFrames.unshift(points);

  state.keyframeCounter += 1;
  if (state.keyframeCounter >= DEFAULTS.render.keyframeHistory) {
    state.keyframeCounter = 0;
    spatialDownsampleKeyframe();
  }

  state.needsRenderUpdate = true;
}

function handleCandidates(msg) {
  const group = plannerVizGroup;
  if (!group) return;

  while (group.children.length > 0) {
    const child = group.children[0];
    group.remove(child);
    if (child.geometry) child.geometry.dispose();
    if (child.material) child.material.dispose();
  }

  const markers = msg.markers;
  if (!markers || !markers.length) return;

  markers.forEach((marker) => {
    if (marker.type !== 4) return;
    const pts = marker.points;
    if (!pts || pts.length < 2) return;

    const positions = new Float32Array(pts.length * 3);
    for (let i = 0; i < pts.length; i += 1) {
      positions[i * 3] = Number(pts[i].x) || 0;
      positions[i * 3 + 1] = Number(pts[i].y) || 0;
      positions[i * 3 + 2] = Number(pts[i].z) || 0;
    }

    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const r = Number(marker.color?.r) || 1;
    const g = Number(marker.color?.g) || 1;
    const b = Number(marker.color?.b) || 1;
    const a = Number(marker.color?.a) || 0.7;

    const mat = new THREE.LineBasicMaterial({
      color: new THREE.Color(r, g, b),
      transparent: a < 1,
      opacity: a,
      linewidth: 1,
      depthTest: true,
    });

    const line = new THREE.Line(geom, mat);
    group.add(line);
  });
}

function handleSafetyBox(msg) {
  const group = plannerVizGroup;
  if (!group) return;

  const existing = group.getObjectByName("safetyBoxWire");
  if (existing) {
    group.remove(existing);
    if (existing.geometry) existing.geometry.dispose();
    if (existing.material) existing.material.dispose();
  }

  const sx = Number(msg.scale?.x) || 1;
  const sy = Number(msg.scale?.y) || 1;
  const sz = Number(msg.scale?.z) || 1;
  const px = Number(msg.pose?.position?.x) || 0;
  const py = Number(msg.pose?.position?.y) || 0;
  const pz = Number(msg.pose?.position?.z) || 0;

  const boxGeom = new THREE.BoxGeometry(sx, sy, sz);
  const edgeGeom = new THREE.EdgesGeometry(boxGeom);
  boxGeom.dispose();

  const mat = new THREE.LineBasicMaterial({
    color: 0xffff00,
    transparent: true,
    opacity: 0.45,
    linewidth: 1,
    depthTest: true,
  });

  const wireframe = new THREE.LineSegments(edgeGeom, mat);
  wireframe.name = "safetyBoxWire";
  wireframe.position.set(px, py, pz);
  group.add(wireframe);
}

function handleSafetyStatus(msg) {
  if (!msg || !msg.data) return;
  const text = String(msg.data);
  ui.safetyStatus.textContent = text;
  // color-code: stop/blocked → red, adjust/slow → yellow, clear → green
  ui.safetyStatus.classList.remove("status-ok", "status-warn", "status-danger");
  if (/stop|blocked/i.test(text)) {
    ui.safetyStatus.classList.add("status-danger");
  } else if (/adjust|slow/i.test(text)) {
    ui.safetyStatus.classList.add("status-warn");
  } else {
    ui.safetyStatus.classList.add("status-ok");
  }
}

function shouldSkipForStationaryGate() {
  if (!state.robot.hasOdom) return false;
  const dx = state.robot.x - state.robot.lastMoveX;
  const dy = state.robot.y - state.robot.lastMoveY;
  const dz = state.robot.z - state.robot.lastMoveZ;
  const distSq = dx * dx + dy * dy + dz * dz;
  const elapsed = performance.now() - state.robot.lastMoveAt;
  return elapsed > DEFAULTS.motion.stationaryTimeMs && distSq < DEFAULTS.motion.stationaryDistance ** 2;
}

function parsePointCloud2(msg) {
  const data = normalizeBinaryData(msg.data);
  if (!data || !msg.fields || !msg.point_step) return [];

  const fields = new Map(msg.fields.map((field) => [field.name, field]));
  const xField = fields.get("x");
  const yField = fields.get("y");
  const zField = fields.get("z");
  const intensityField = fields.get("intensity");

  if (!isFloat32Field(xField) || !isFloat32Field(yField) || !isFloat32Field(zField)) {
    logEvent("PointCloud2 字段不完整或不是 FLOAT32 x/y/z");
    return [];
  }

  const pointStep = Number(msg.point_step);
  const total = Number(msg.width || 0) * Number(msg.height || 0);
  const stride = Math.max(1, Math.floor(total / DEFAULTS.render.maxRawPerFrame));
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const littleEndian = !msg.is_bigendian;
  const points = [];
  const maxRadiusSq = state.render.outlierRadius * state.render.outlierRadius;

  for (let i = 0; i < total; i += stride) {
    const base = i * pointStep;
    if (base + pointStep > data.byteLength) break;

    const x = view.getFloat32(base + xField.offset, littleEndian);
    const y = view.getFloat32(base + yField.offset, littleEndian);
    const z = view.getFloat32(base + zField.offset, littleEndian);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    if (x * x + y * y + z * z > maxRadiusSq) continue;

    let intensity = 0;
    if (isFloat32Field(intensityField) && base + intensityField.offset + 4 <= data.byteLength) {
      intensity = view.getFloat32(base + intensityField.offset, littleEndian);
    }
    points.push({ x, y, z, intensity });
  }

  return points;
}

function isFloat32Field(field) {
  return Boolean(field && Number(field.datatype) === DATATYPE.FLOAT32);
}

function normalizeBinaryData(data) {
  if (!data) return null;
  if (data instanceof Uint8Array) return data;
  if (data instanceof ArrayBuffer) return new Uint8Array(data);
  if (Array.isArray(data)) return Uint8Array.from(data);
  if (typeof data === "string") return base64ToUint8Array(data);
  if (typeof data === "object" && Array.isArray(data.bytes)) return Uint8Array.from(data.bytes);
  return null;
}

function base64ToUint8Array(base64) {
  const binary = atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
}

function uint8ToBase64(bytes) {
  let binary = "";
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    binary += String.fromCharCode(...bytes.subarray(i, i + chunk));
  }
  return btoa(binary);
}

function mergeIntoKeyframe(points) {
  if (!points || !points.length) return;
  state.keyframe.push(...points);
  if (state.keyframe.length > state.render.keyframeBudget * 1.25) {
    spatialDownsampleKeyframe();
  }
}

function spatialDownsampleKeyframe() {
  if (state.keyframe.length <= state.render.keyframeBudget) return;
  const cell = state.render.keyframeCell;
  const occupied = new Set();
  const filtered = [];

  for (const point of state.keyframe) {
    const key = `${Math.floor(point.x / cell)},${Math.floor(point.y / cell)},${Math.floor(point.z / cell)}`;
    if (occupied.has(key)) continue;
    occupied.add(key);
    filtered.push(point);
    if (filtered.length >= state.render.keyframeBudget) break;
  }

  state.keyframe = filtered;
}

function rebuildVoxels() {
  const points = [];
  for (const frame of state.recentFrames) points.push(...frame);
  points.push(...state.keyframe);

  const stride = Math.max(1, Math.floor(points.length / state.render.renderBudget));
  const voxelSize = state.render.voxelSize;
  const invVoxel = 1 / voxelSize;
  const voxels = new Map();

  for (let i = 0; i < points.length; i += stride) {
    const p = points[i];
    if (p.z > state.render.zMax) continue;
    if (state.frontCull && state.robot.hasOdom) {
      const dot = (p.x - state.robot.x) * state.robot.fwdX +
        (p.y - state.robot.y) * state.robot.fwdY +
        (p.z - state.robot.z) * state.robot.fwdZ;
      if (dot <= 0) continue;
    }

    const ix = Math.floor(p.x * invVoxel);
    const iy = Math.floor(p.y * invVoxel);
    const iz = Math.floor(p.z * invVoxel);
    const key = `${ix},${iy},${iz}`;
    const voxel = voxels.get(key);
    if (voxel) {
      voxel.sumZ += p.z;
      voxel.count += 1;
    } else {
      voxels.set(key, { ix, iy, iz, sumZ: p.z, count: 1 });
    }
  }

  installVoxelMeshes([...voxels.values()]);
  state.stats.visibleVoxels = voxels.size;
  state.needsRenderUpdate = false;
}

function installVoxelMeshes(voxels) {
  voxelMeshes.forEach((mesh) => {
    scene.remove(mesh);
    mesh.material.dispose();
  });
  voxelMeshes = [];
  if (wireMesh) {
    scene.remove(wireMesh);
    wireMesh.material.dispose();
    wireMesh = null;
  }

  const count = voxels.length;
  if (count === 0) return;

  const colorBuckets = [
    { max: 0.125, color: 0x1f3f8f, voxels: [] },
    { max: 0.25, color: 0x236c9f, voxels: [] },
    { max: 0.375, color: 0x249a9a, voxels: [] },
    { max: 0.5, color: 0x2c9a69, voxels: [] },
    { max: 0.625, color: 0x579b3a, voxels: [] },
    { max: 0.75, color: 0xa89535, voxels: [] },
    { max: 0.875, color: 0xa5672c, voxels: [] },
    { max: 1.001, color: 0xa23a32, voxels: [] },
  ];

  voxels.forEach((voxel) => {
    const bucket = colorBuckets[colorBucketIndex(voxel.sumZ / voxel.count)];
    bucket.voxels.push(voxel);
  });

  const matrix = new THREE.Matrix4();
  const half = state.render.voxelSize * 0.5;
  const writeMatrix = (mesh, voxel, index) => {
    const x = voxel.ix * state.render.voxelSize + half;
    const y = voxel.iy * state.render.voxelSize + half;
    const z = voxel.iz * state.render.voxelSize + half;
    matrix.compose(
      new THREE.Vector3(x, y, z),
      new THREE.Quaternion(),
      new THREE.Vector3(state.render.voxelSize, state.render.voxelSize, state.render.voxelSize),
    );
    mesh.setMatrixAt(index, matrix);
  };

  colorBuckets.forEach((bucket) => {
    if (bucket.voxels.length === 0) return;
    const material = new THREE.MeshBasicMaterial({
      color: bucket.color,
      transparent: state.render.alpha < 1,
      opacity: state.render.alpha,
      toneMapped: false,
      depthWrite: true,
    });
    const mesh = new THREE.InstancedMesh(cubeGeometry, material, bucket.voxels.length);
    bucket.voxels.forEach((voxel, index) => writeMatrix(mesh, voxel, index));
    mesh.instanceMatrix.needsUpdate = true;
    voxelMeshes.push(mesh);
    scene.add(mesh);
  });

  const wireMaterial = new THREE.MeshBasicMaterial({
    color: 0xe9f2f6,
    transparent: true,
    opacity: state.render.wireAlpha,
    wireframe: true,
    depthWrite: false,
  });
  wireMesh = new THREE.InstancedMesh(cubeGeometry, wireMaterial, count);
  voxels.forEach((voxel, index) => writeMatrix(wireMesh, voxel, index));
  wireMesh.instanceMatrix.needsUpdate = true;
  wireMesh.visible = state.showWire;
  scene.add(wireMesh);
}

function colorBucketIndex(z) {
  const min = state.render.depthColorMin;
  const max = state.render.depthColorMax;
  const t = clamp((z - min) / Math.max(0.1, max - min), 0, 1);
  return Math.min(7, Math.floor(t * 8));
}

function heightColor(z) {
  const colors = [0x1f3f8f, 0x236c9f, 0x249a9a, 0x2c9a69, 0x579b3a, 0xa89535, 0xa5672c, 0xa23a32];
  return new THREE.Color(colors[colorBucketIndex(z)]);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function resetView() {
  const target = new THREE.Vector3(state.robot.x, state.robot.y, state.robot.z + DEFAULTS.camera.targetZOffset);
  controls.target.copy(target);
  camera.position.set(target.x + 3.8, target.y - 5.2, target.z + 2.9);
  controls.update();
  logEvent("视角已重置");
}

function saveView() {
  localStorage.setItem("web-gs.camera", JSON.stringify({
    position: camera.position.toArray(),
    target: controls.target.toArray(),
    follow: state.follow,
  }));
  logEvent("视角已保存到浏览器");
}

function loadSavedView() {
  const raw = localStorage.getItem("web-gs.camera");
  if (!raw) return;
  try {
    const saved = JSON.parse(raw);
    if (Array.isArray(saved.position)) camera.position.fromArray(saved.position);
    if (Array.isArray(saved.target)) controls.target.fromArray(saved.target);
    if (typeof saved.follow === "boolean") {
      state.follow = saved.follow;
      ui.followToggle.checked = saved.follow;
    }
  } catch (_) {
    // ignore invalid localStorage
  }
}

function updateFollowCamera() {
  if (!state.follow || !state.robot.hasOdom) return;
  const visualPose = robotVisualPose();
  const target = new THREE.Vector3(
    visualPose.x,
    visualPose.y,
    visualPose.z + DEFAULTS.camera.targetZOffset,
  );
  controls.target.lerp(target, 0.08);
}

function updateStatsPanel() {
  const recentCount = state.recentFrames.reduce((sum, frame) => sum + frame.length, 0);
  const visualPose = robotVisualPose();
  ui.visibleVoxels.textContent = String(state.stats.visibleVoxels);
  ui.cameraMode.textContent = state.follow ? "FOLLOW" : "FREE";
  ui.robotPosition.textContent = `x=${visualPose.x.toFixed(2)} y=${visualPose.y.toFixed(2)} z=${visualPose.z.toFixed(2)}`;
  ui.frameCount.textContent = String(state.stats.frames);
  ui.rawPerFrame.textContent = String(state.stats.rawPerFrame);
  ui.recentPoints.textContent = String(recentCount);
  ui.keyframePoints.textContent = String(state.keyframe.length);
  ui.rawTotal.textContent = String(state.stats.rawTotal);
  ui.odomState.textContent = state.robot.hasOdom ? "有效" : "无";
}

function bindUi() {
  ui.connectBtn.addEventListener("click", connectRos);
  ui.disconnectBtn.addEventListener("click", () => disconnectRos(true));
  ui.applyTopicsBtn.addEventListener("click", applyTopics);
  ui.followToggle.addEventListener("change", () => {
    state.follow = ui.followToggle.checked;
  });
  ui.frontCullToggle.addEventListener("change", () => {
    state.frontCull = ui.frontCullToggle.checked;
    state.needsRenderUpdate = true;
  });
  ui.wireToggle.addEventListener("change", () => {
    state.showWire = ui.wireToggle.checked;
    if (wireMesh) wireMesh.visible = state.showWire;
  });
  ui.resetViewBtn.addEventListener("click", resetView);
  ui.saveViewBtn.addEventListener("click", saveView);
  ui.canvas.addEventListener("dblclick", resetView);

  ui.voxelSize.addEventListener("input", () => {
    state.render.voxelSize = Number(ui.voxelSize.value) / 100;
    updateControlLabels();
    state.needsRenderUpdate = true;
  });
  ui.renderBudget.addEventListener("input", () => {
    state.render.renderBudget = Number(ui.renderBudget.value);
    updateControlLabels();
    state.needsRenderUpdate = true;
  });
  ui.alpha.addEventListener("input", () => {
    state.render.alpha = Number(ui.alpha.value) / 100;
    updateControlLabels();
    voxelMeshes.forEach((mesh) => {
      mesh.material.opacity = state.render.alpha;
      mesh.material.transparent = state.render.alpha < 1;
      mesh.material.needsUpdate = true;
    });
  });
  ui.zMax.addEventListener("input", () => {
    state.render.zMax = Number(ui.zMax.value) / 100;
    updateControlLabels();
    state.needsRenderUpdate = true;
  });
}

function animationLoop() {
  requestAnimationFrame(animationLoop);
  resizeRenderer();
  if (state.needsRenderUpdate) rebuildVoxels();
  updateFollowCamera();
  controls.update();
  updateStatsPanel();
  renderer.render(scene, camera);
}

async function bootstrap() {
  await loadRuntimeConfig();
  loadUiDefaults();
  bindUi();
  loadSavedView();
  setConnectionState("offline", "离线");
  logEvent("Web 地面站已就绪");
  animationLoop();
  connectRos();
}

bootstrap();