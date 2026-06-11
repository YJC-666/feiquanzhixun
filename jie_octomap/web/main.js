import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

const ROSLIB = window.ROSLIB;
if (!ROSLIB) {
  throw new Error("ROSLIB 加载失败，请检查 CDN 网络访问，或改为本地提供 roslib。");
}

const wsInput = document.getElementById("ws-url");
const advancedSettings = document.getElementById("advanced-settings");
const connectBtn = document.getElementById("connect-btn");
const reconnectBtn = document.getElementById("reconnect-btn");
const connStatus = document.getElementById("conn-status");
const relocalizationStatus = document.getElementById("relocalization-status");
const robotTfStatus = document.getElementById("robot-tf-status");
const selectionStatus = document.getElementById("selection-status");
const mapStatus = document.getElementById("map-status");
const canvas = document.getElementById("viewport");
const navigationConfirmModal = document.getElementById("navigation-confirm-modal");
const navigationConfirmStartBtn = document.getElementById("navigation-confirm-start");
const navigationConfirmCancelBtn = document.getElementById("navigation-confirm-cancel");
const setNavigateBtn = document.getElementById("set-navigate-btn");
const setStartBtn = document.getElementById("set-start-btn");
const setGoalBtn = document.getElementById("set-goal-btn");
const stopNavigationBtn = document.getElementById("stop-navigation-btn");
const navModeToggleBtn = document.getElementById("nav-mode-toggle-btn");
const navModeStatus = document.getElementById("nav-mode-status");
const navModeSummary = document.getElementById("nav-mode-summary");
const goalToleranceInput = document.getElementById("goal-tolerance-input");
const saveGoalToleranceBtn = document.getElementById("save-goal-tolerance-btn");
const goalToleranceStatus = document.getElementById("goal-tolerance-status");
const goalToleranceSummary = document.getElementById("goal-tolerance-summary");
const resetViewBtn = document.getElementById("reset-view-btn");
const joystickPad = document.getElementById("motion-joystick");
const joystickKnob = document.getElementById("motion-joystick-knob");
const manualVelocityDisplay = document.getElementById("manual-velocity-display");
const unitreeStatus = document.getElementById("unitree-status");
const dogModeStatus = document.getElementById("dog-mode-status");
const localAvoidanceStatus = document.getElementById("local-avoidance-status");
const applyLocalAvoidanceBtn = document.getElementById("apply-local-avoidance-btn");
const saveLocalAvoidanceBtn = document.getElementById("save-local-avoidance-btn");
const settingsTabButtons = Array.from(document.querySelectorAll("[data-settings-tab]"));
const settingsPanes = Array.from(document.querySelectorAll("[data-settings-pane]"));
const localAvoidanceInputs = Array.from(document.querySelectorAll("[data-avoidance-param]"));
const dogModeButtons = Array.from(document.querySelectorAll("[data-dog-mode]"));
const dogMotionButtons = Array.from(document.querySelectorAll("[data-motion-speed][data-motion-yaw]"));
const cameraPreviewImages = [
  document.getElementById("camera-preview-1"),
  document.getElementById("camera-preview-2"),
  document.getElementById("camera-preview-3"),
];
const cameraPreviewStatuses = [
  document.getElementById("camera-preview-status-1"),
  document.getElementById("camera-preview-status-2"),
  document.getElementById("camera-preview-status-3"),
];
const cameraPreviewPaths = [
  "/camera/preview_1.mjpg",
  "/camera/preview_2.mjpg",
  "/camera/preview_3.mjpg",
];

function defaultRosbridgeUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname || "localhost"}:9090`;
}

wsInput.value = defaultRosbridgeUrl();

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(canvas.clientWidth || window.innerWidth, canvas.clientHeight || window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x091116);

const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 500);
camera.up.set(0, 0, 1);
camera.position.set(8, -10, 7);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 0, 0.8);

function makeAxisArrow(direction, color, length, shaftRadius, headLength, headRadius) {
  const group = new THREE.Group();
  const shaftLength = Math.max(0.01, length - headLength);
  const shaftMaterial = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.35,
    metalness: 0.08,
  });
  const headMaterial = new THREE.MeshStandardMaterial({
    color,
    emissive: color,
    emissiveIntensity: 0.18,
    roughness: 0.25,
    metalness: 0.1,
  });

  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 16),
    shaftMaterial,
  );
  shaft.position.y = shaftLength * 0.5;

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(headRadius, headLength, 20),
    headMaterial,
  );
  head.position.y = shaftLength + headLength * 0.5;

  group.add(shaft);
  group.add(head);

  const up = new THREE.Vector3(0, 1, 0);
  group.quaternion.setFromUnitVectors(up, direction.clone().normalize());
  return group;
}

function makeAxes(length = 2.0) {
  const group = new THREE.Group();
  const shaftRadius = 0.035;
  const headLength = 0.22;
  const headRadius = 0.09;

  group.add(makeAxisArrow(new THREE.Vector3(1, 0, 0), 0xff5f5f, length, shaftRadius, headLength, headRadius));
  group.add(makeAxisArrow(new THREE.Vector3(0, 1, 0), 0x58ef74, length, shaftRadius, headLength, headRadius));
  group.add(makeAxisArrow(new THREE.Vector3(0, 0, 1), 0x53b7d8, length, shaftRadius, headLength, headRadius));
  return group;
}

function makeBox(sizeX, sizeY, sizeZ, color) {
  const geometry = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
  const material = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.42,
    metalness: 0.05,
  });
  return new THREE.Mesh(geometry, material);
}

function buildSimpleDogModel() {
  const group = new THREE.Group();

  const body = makeBox(0.50, 0.22, 0.16, 0xf2f2f2);
  body.position.set(0, 0, 0.33);
  group.add(body);

  const head = makeBox(0.12, 0.12, 0.12, 0x151515);
  head.position.set(0.31, 0, 0.35);
  group.add(head);

  const hipColor = 0xeb3131;
  const legColor = 0xf2f2f2;
  const hips = [
    [0.18, 0.13, 0.30],
    [0.18, -0.13, 0.30],
    [-0.18, 0.13, 0.30],
    [-0.18, -0.13, 0.30],
  ];

  hips.forEach(([x, y, z]) => {
    const hip = makeBox(0.06, 0.06, 0.06, hipColor);
    hip.position.set(x, y, z);
    group.add(hip);

    const upper = makeBox(0.04, 0.04, 0.22, legColor);
    upper.position.set(x, y, z - 0.14);
    group.add(upper);
  });

  group.visible = false;
  return group;
}

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.1);
dirLight.position.set(10, -6, 14);
scene.add(dirLight);

const groundGrid = new THREE.GridHelper(30, 30, 0x34515c, 0x24353d);
groundGrid.rotation.x = Math.PI * 0.5;
scene.add(groundGrid);
scene.add(makeAxes(1.8));

let retainedMapCloudObject = null;
let localMapCloudObject = null;
let pathObject = null;
let manualPredictionObject = null;
let trackingPointObject = null;
let startArrow = null;
let startCube = null;
let goalArrow = null;
let goalCube = null;
let robotObject = buildSimpleDogModel();
let voxelSize = 0.2;
let ros = null;
let startTopic = null;
let goalTopic = null;
let goalPoseTopic = null;
let startNavigationTopic = null;
let stopNavigationTopic = null;
let goalToleranceTopic = null;
let navigationControlModeTopic = null;
let localAvoidanceConfigTopic = null;
let cmdVelTopic = null;
let dogModeCommandTopic = null;
let unitreeStatusTopic = null;
let odomTopic = null;
let reconnectTimer = null;
let activePointerId = null;
let navigationDrag = null;
let placementMode = null;
let pendingNavigationGoal = null;
let pendingNavigationPath = null;
let navigationControlMode = "auto";
let goalToleranceMeters = 0.08;
let navigationSettingsApi = "/api/navigation-settings";
let localAvoidanceSettingsApi = "/api/local-avoidance-settings";
let localAvoidanceSettings = null;
let localAvoidancePublishTimer = null;
let navigationConfirmTimer = null;
let joystickActivePointerId = null;
let joystickLastPublishMs = 0;
let joystickRepeatTimer = null;
let joystickCurrentLinearX = 0;
let joystickCurrentLinearY = 0;
let joystickCurrentAngularZ = 0;
let latestMapBounds = null;
let latestGlobalPointCount = 0;
let latestLocalPointCount = 0;
let globalMapHasBeenAutoFramed = false;
let latestOdomPose = null;
let navigationMapFrameId = "camera_init";
const joystickMaxLinearX = 0.42;
const joystickMaxAngularZ = 0.45;
const joystickDeadband = 0.12;
const joystickMinCommandSpeed = 0.06;
const joystickPublishIntervalMs = 80;
const robotDisplayOffset = { x: 0.0, y: 0.0, z: -0.3 };
const defaultLocalAvoidanceSettings = {
  local_planner_enabled: true,
  stop_without_cloud: true,
  front_only_obstacles: true,
  preview_time: 1.2,
  stop_distance: 0.55,
  warning_distance: 1.35,
  rotate_stop_radius: 0.55,
  rotate_warning_radius: 0.95,
  trajectory_horizon: 1.8,
  trajectory_dt: 0.12,
  trajectory_collision_margin: 0.08,
  lateral_margin: 0.15,
  front_min_x: 0.0,
  clearance_weight: 4.0,
  heading_weight: 2.2,
  speed_weight: 1.4,
  smoothness_weight: 1.2,
  rotation_escape_weight: 0.6,
  point_sample_step: 4,
  max_points: 5000,
  candidate_linear_scales: [1.0, 0.75, 0.5, 0.25, 0.0],
  candidate_angular_offsets: [-0.70, -0.45, -0.25, 0.0, 0.25, 0.45, 0.70],
};

scene.add(robotObject);

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
function setConnectionStatus(text) {
  connStatus.textContent = text;
}

function setSelectionStatus(text) {
  selectionStatus.textContent = text;
}

function setRobotTfStatus(text) {
  if (robotTfStatus) {
    robotTfStatus.textContent = text;
  }
}

function setRelocalizationStatus(localized) {
  if (!relocalizationStatus) {
    return;
  }
  relocalizationStatus.textContent = localized ? "里程计已接入" : "未收到里程计";
  relocalizationStatus.classList.toggle("ok", localized);
  relocalizationStatus.classList.toggle("fail", !localized);
}

function setMapStatus(text) {
  mapStatus.textContent = text;
}

function setCameraPreviewStatus(index, text) {
  const status = cameraPreviewStatuses[index];
  if (status) {
    status.textContent = text;
  }
}

function startCameraPreviews() {
  cameraPreviewImages.forEach((image, index) => {
    const path = cameraPreviewPaths[index];
    if (!image || !path) {
      return;
    }
    image.width = 640;
    image.height = 480;
    image.src = `${path}?t=${Date.now()}`;
    image.onload = () => setCameraPreviewStatus(index, `HTTP 640×480 预览 ${index + 1}`);
    image.onerror = () => setCameraPreviewStatus(index, `未收到图像流：${path}`);
    setCameraPreviewStatus(index, `加载 ${path}`);
  });
}

function stopCameraPreviews(text = "等待图像") {
  cameraPreviewImages.forEach((image) => {
    if (image) {
      image.removeAttribute("src");
      image.onload = null;
      image.onerror = null;
    }
  });
  cameraPreviewStatuses.forEach((status) => {
    if (status) {
      status.textContent = text;
    }
  });
}

function setGoalToleranceUi(value, saved = false) {
  goalToleranceMeters = Number(value);
  const text = `${goalToleranceMeters.toFixed(2)}m`;
  if (goalToleranceInput) {
    goalToleranceInput.value = goalToleranceMeters.toFixed(2);
  }
  if (goalToleranceStatus) {
    goalToleranceStatus.textContent = `${saved ? "已保存" : "当前"} ${text}`;
  }
  if (goalToleranceSummary) {
    goalToleranceSummary.textContent = text;
  }
}

function parseGoalToleranceInput() {
  const value = Number(goalToleranceInput?.value || goalToleranceMeters);
  if (!Number.isFinite(value) || value <= 0) {
    throw new Error("到达目标容差必须大于 0");
  }
  return Math.max(0.01, Math.min(value, 2.0));
}

function publishGoalTolerance(value = goalToleranceMeters) {
  if (!goalToleranceTopic) {
    return;
  }
  goalToleranceTopic.publish(new ROSLIB.Message({ data: value }));
}

async function loadNavigationSettings() {
  try {
    const response = await fetch(navigationSettingsApi, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const settings = await response.json();
    setGoalToleranceUi(Number(settings.goalTolerance || 0.08), true);
    publishGoalTolerance(goalToleranceMeters);
  } catch (error) {
    setGoalToleranceUi(0.08, false);
    if (goalToleranceStatus) {
      goalToleranceStatus.textContent = "使用默认 0.08m，设置接口暂不可用";
    }
  }
}

async function saveNavigationSettings() {
  let value = 0.08;
  try {
    value = parseGoalToleranceInput();
    const response = await fetch(navigationSettingsApi, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ goalTolerance: value }),
    });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const settings = await response.json();
    setGoalToleranceUi(Number(settings.goalTolerance || value), true);
    publishGoalTolerance(goalToleranceMeters);
    setSelectionStatus(`到达目标容差已保存为 ${goalToleranceMeters.toFixed(2)}m。`);
  } catch (error) {
    if (goalToleranceStatus) {
      goalToleranceStatus.textContent = `保存失败：${error.message}`;
    }
    setSelectionStatus(`到达目标容差保存失败：${error.message}`);
  }
}

function normalizeLocalAvoidanceSettings(settings = {}) {
  return { ...defaultLocalAvoidanceSettings, ...settings };
}

function parseNumberList(text, fallback) {
  const values = String(text || "")
    .split(",")
    .map((item) => Number(item.trim()))
    .filter((value) => Number.isFinite(value));
  return values.length > 0 ? values : fallback;
}

function setLocalAvoidanceUi(settings, saved = false) {
  localAvoidanceSettings = normalizeLocalAvoidanceSettings(settings);
  localAvoidanceInputs.forEach((input) => {
    const key = input.dataset.avoidanceParam;
    const value = localAvoidanceSettings[key];
    if (input.type === "checkbox") {
      input.checked = Boolean(value);
    } else if (Array.isArray(value)) {
      input.value = value.join(",");
    } else if (value !== undefined) {
      input.value = String(value);
    }
  });
  if (localAvoidanceStatus) {
    localAvoidanceStatus.textContent = saved ? "局部避障参数已保存并下发。" : "局部避障参数已实时下发。";
  }
}

function collectLocalAvoidanceSettings() {
  const settings = normalizeLocalAvoidanceSettings(localAvoidanceSettings || {});
  localAvoidanceInputs.forEach((input) => {
    const key = input.dataset.avoidanceParam;
    if (!key) {
      return;
    }
    if (input.type === "checkbox") {
      settings[key] = input.checked;
      return;
    }
    if (key === "candidate_linear_scales" || key === "candidate_angular_offsets") {
      settings[key] = parseNumberList(input.value, defaultLocalAvoidanceSettings[key]);
      return;
    }
    const value = Number(input.value);
    if (Number.isFinite(value)) {
      settings[key] = value;
    }
  });
  localAvoidanceSettings = settings;
  return settings;
}

function publishLocalAvoidanceSettings(settings = collectLocalAvoidanceSettings()) {
  localAvoidanceSettings = normalizeLocalAvoidanceSettings(settings);
  if (localAvoidanceConfigTopic) {
    localAvoidanceConfigTopic.publish(new ROSLIB.Message({ data: JSON.stringify(localAvoidanceSettings) }));
  }
  if (localAvoidanceStatus) {
    localAvoidanceStatus.textContent = "局部避障参数已实时下发。";
  }
}

function scheduleLocalAvoidancePublish() {
  if (localAvoidancePublishTimer) {
    window.clearTimeout(localAvoidancePublishTimer);
  }
  localAvoidancePublishTimer = window.setTimeout(() => {
    localAvoidancePublishTimer = null;
    publishLocalAvoidanceSettings();
  }, 180);
}

async function loadLocalAvoidanceSettings() {
  try {
    const response = await fetch(localAvoidanceSettingsApi, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const settings = await response.json();
    setLocalAvoidanceUi(settings, true);
    publishLocalAvoidanceSettings(localAvoidanceSettings);
  } catch (error) {
    setLocalAvoidanceUi(defaultLocalAvoidanceSettings, false);
    if (localAvoidanceStatus) {
      localAvoidanceStatus.textContent = "使用默认局部避障参数，设置接口暂不可用。";
    }
  }
}

async function saveLocalAvoidanceSettings() {
  const settings = collectLocalAvoidanceSettings();
  try {
    const response = await fetch(localAvoidanceSettingsApi, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(settings),
    });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const saved = await response.json();
    setLocalAvoidanceUi(saved, true);
    publishLocalAvoidanceSettings(localAvoidanceSettings);
    setSelectionStatus("局部避障参数已保存，并实时下发到避障节点。");
  } catch (error) {
    if (localAvoidanceStatus) {
      localAvoidanceStatus.textContent = `保存失败：${error.message}`;
    }
    setSelectionStatus(`局部避障参数保存失败：${error.message}`);
  }
}

function setSettingsTab(tabName) {
  settingsTabButtons.forEach((button) => button.classList.toggle("active", button.dataset.settingsTab === tabName));
  settingsPanes.forEach((pane) => pane.classList.toggle("active", pane.dataset.settingsPane === tabName));
}

function publishNavigationControlMode(mode = navigationControlMode) {
  if (!navigationControlModeTopic) {
    return;
  }
  navigationControlModeTopic.publish(new ROSLIB.Message({ data: mode }));
}

function setNavigationControlMode(mode, options = {}) {
  navigationControlMode = mode === "manual" ? "manual" : "auto";
  const isAuto = navigationControlMode === "auto";
  if (navModeStatus) {
    navModeStatus.textContent = isAuto ? "自动模式" : "手动模式";
    navModeStatus.classList.toggle("auto", isAuto);
    navModeStatus.classList.toggle("manual", !isAuto);
  }
  if (navModeSummary) {
    navModeSummary.textContent = isAuto ? "自动" : "手动";
  }
  if (navModeToggleBtn) {
    navModeToggleBtn.textContent = isAuto ? "切到手动" : "切到自动";
    navModeToggleBtn.classList.toggle("auto", isAuto);
    navModeToggleBtn.classList.toggle("manual", !isAuto);
  }
  publishNavigationControlMode(navigationControlMode);
  if (!isAuto) {
    stopPathTrackingForManualMotion();
    publishZeroVelocity();
    clearPlannedPathVisual();
    setPlacementMode(null);
    if (!options.quiet) {
      setSelectionStatus("已切换手动模式：机器狗已请求立刻停止，导航状态已重置。下次需切回自动并重新设置导航目标。");
    }
  } else if (!options.quiet) {
    pendingNavigationGoal = null;
    pendingNavigationPath = null;
    clearPlannedPathVisual();
    setSelectionStatus("已切换自动模式：请重新设置导航目标后开始新的自动导航。");
  }
}

function toggleNavigationControlMode() {
  setNavigationControlMode(navigationControlMode === "auto" ? "manual" : "auto");
}

function clearPlannedPathVisual() {
  if (pathObject) {
    scene.remove(pathObject);
    disposeObject(pathObject);
    pathObject = null;
  }
  clearTrackingPointObjects();
}

function updateRendererSize() {
  const width = canvas.clientWidth || window.innerWidth;
  const height = canvas.clientHeight || Math.max(window.innerHeight - 260, 320);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function disposeObject(object) {
  if (!object) {
    return;
  }
  if (object.children && object.children.length > 0) {
    object.children.forEach((child) => disposeObject(child));
  }
  if (object.geometry) {
    object.geometry.dispose();
  }
  if (Array.isArray(object.material)) {
    object.material.forEach((material) => material.dispose());
  } else if (object.material) {
    object.material.dispose();
  }
}

function computePointBounds(points) {
  if (!points || points.length === 0) {
    return null;
  }
  const min = new THREE.Vector3(Infinity, Infinity, Infinity);
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  for (const point of points) {
    min.x = Math.min(min.x, point.x);
    min.y = Math.min(min.y, point.y);
    min.z = Math.min(min.z, point.z);
    max.x = Math.max(max.x, point.x);
    max.y = Math.max(max.y, point.y);
    max.z = Math.max(max.z, point.z);
  }
  if (!Number.isFinite(min.x) || !Number.isFinite(max.x)) {
    return null;
  }
  return { min, max };
}

function frameCameraToBounds(bounds) {
  if (!bounds) {
    return;
  }
  const center = bounds.min.clone().add(bounds.max).multiplyScalar(0.5);
  const size = bounds.max.clone().sub(bounds.min);
  const maxDim = Math.max(size.x, size.y, size.z, voxelSize * 8.0, 1.0);
  const fovRad = THREE.MathUtils.degToRad(camera.fov);
  const distance = Math.max(maxDim * 1.35, (maxDim * 0.5) / Math.tan(fovRad * 0.5));
  const direction = new THREE.Vector3(1.0, -1.25, 0.8).normalize();
  const position = center.clone().add(direction.multiplyScalar(distance));

  camera.position.copy(position);
  controls.target.copy(center);
  controls.minDistance = Math.max(voxelSize * 2.0, distance * 0.02);
  controls.maxDistance = Math.max(distance * 8.0, maxDim * 8.0);
  camera.near = Math.max(0.01, distance / 10000.0);
  camera.far = Math.max(1000.0, distance * 10.0, maxDim * 20.0);
  camera.updateProjectionMatrix();
  controls.update();
}

function resetMapView() {
  const bounds = latestMapBounds;
  if (!bounds) {
    setMapStatus("尚未收到可用于重置视角的全局点云地图。");
    return;
  }
  frameCameraToBounds(bounds);
}

function frameGlobalMapOnce(bounds) {
  if (!bounds || globalMapHasBeenAutoFramed) {
    return;
  }
  frameCameraToBounds(bounds);
  globalMapHasBeenAutoFramed = true;
}

function updateNavigationMapFrame(header) {
  const frameId = normalizeFrameId(header && header.frame_id);
  if (frameId) {
    navigationMapFrameId = frameId;
  }
}

function parsePointCloud2(msg) {
  if (!msg || !msg.data || !msg.fields) {
    return [];
  }

  const byteArray = Array.isArray(msg.data)
    ? new Uint8Array(msg.data)
    : Uint8Array.from(atob(msg.data), (ch) => ch.charCodeAt(0));
  const view = new DataView(byteArray.buffer, byteArray.byteOffset, byteArray.byteLength);
  const fieldMap = new Map(msg.fields.map((field) => [field.name, field]));
  const xField = fieldMap.get("x");
  const yField = fieldMap.get("y");
  const zField = fieldMap.get("z");
  const intensityField = fieldMap.get("intensity");
  if (!xField || !yField || !zField) {
    return [];
  }

  const littleEndian = !msg.is_bigendian;
  const points = [];
  const pointStep = msg.point_step;
  const total = pointStep > 0 ? Math.floor(byteArray.byteLength / pointStep) : 0;
  for (let i = 0; i < total; i += 1) {
    const base = i * pointStep;
    const x = view.getFloat32(base + xField.offset, littleEndian);
    const y = view.getFloat32(base + yField.offset, littleEndian);
    const z = view.getFloat32(base + zField.offset, littleEndian);
    const intensity = intensityField
      ? view.getFloat32(base + intensityField.offset, littleEndian)
      : 0.0;
    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
      points.push({ x, y, z, intensity });
    }
  }
  return points;
}

function makePointCloudObject(points, mode) {
  const positions = new Float32Array(points.length * 3);
  const colors = new Float32Array(points.length * 3);
  const bounds = computePointBounds(points);
  const minZ = bounds ? bounds.min.z : 0.0;
  const maxZ = bounds ? bounds.max.z : 1.0;
  const zRange = Math.max(0.05, maxZ - minZ);
  const color = new THREE.Color();

  points.forEach((point, index) => {
    const base = index * 3;
    positions[base] = point.x;
    positions[base + 1] = point.y;
    positions[base + 2] = point.z;

    if (mode === "local") {
      color.setRGB(1.0, 0.68, 0.10);
    } else {
      const t = Math.max(0.0, Math.min(1.0, (point.z - minZ) / zRange));
      color.setHSL(0.58 - t * 0.36, 0.82, 0.56);
    }
    colors[base] = color.r;
    colors[base + 1] = color.g;
    colors[base + 2] = color.b;
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.computeBoundingSphere();

  const material = new THREE.PointsMaterial({
    size: mode === "local" ? Math.max(voxelSize * 0.58, 0.075) : Math.max(voxelSize * 0.30, 0.04),
    vertexColors: true,
    sizeAttenuation: true,
    transparent: mode === "local",
    opacity: mode === "local" ? 0.96 : 1.0,
    depthWrite: mode !== "local",
  });
  const object = new THREE.Points(geometry, material);
  object.renderOrder = mode === "local" ? 20 : 10;
  return { object, bounds };
}

function updateMapStatus() {
  if (!latestGlobalPointCount && !latestLocalPointCount) {
    setMapStatus("尚未加载全局点云地图和 3m×3m×3m 局部避障点云。");
    return;
  }
  if (!latestGlobalPointCount && latestLocalPointCount) {
    setMapStatus(`仅收到 3m×3m×3m 局部避障 ${latestLocalPointCount} 点；尚未收到全局点云底图，目标点暂不能基于全局地图选择。`);
    return;
  }
  setMapStatus(`全局点云 ${latestGlobalPointCount} 点；3m×3m×3m 局部避障 ${latestLocalPointCount} 点。导航=全局路径+局部避障`);
}

function setRetainedMapCloud(msg) {
  updateNavigationMapFrame(msg.header);
  const points = parsePointCloud2(msg);
  if (points.length === 0) {
    if (!retainedMapCloudObject) {
      latestGlobalPointCount = 0;
      latestMapBounds = null;
      globalMapHasBeenAutoFramed = false;
    }
    updateMapStatus();
    return;
  }

  latestGlobalPointCount = points.length;
  if (retainedMapCloudObject) {
    scene.remove(retainedMapCloudObject);
    disposeObject(retainedMapCloudObject);
    retainedMapCloudObject = null;
  }

  const cloud = makePointCloudObject(points, "global");
  retainedMapCloudObject = cloud.object;
  retainedMapCloudObject.visible = true;
  scene.add(retainedMapCloudObject);

  latestMapBounds = cloud.bounds;
  frameGlobalMapOnce(cloud.bounds);
  updateMapStatus();
}

function setLocalMapCloud(msg) {
  if (localMapCloudObject) {
    scene.remove(localMapCloudObject);
    disposeObject(localMapCloudObject);
    localMapCloudObject = null;
  }

  const points = parsePointCloud2(msg);
  latestLocalPointCount = points.length;
  if (points.length === 0) {
    updateMapStatus();
    return;
  }

  const cloud = makePointCloudObject(points, "local");
  localMapCloudObject = cloud.object;
  localMapCloudObject.visible = true;
  scene.add(localMapCloudObject);

  updateMapStatus();
}

function clearObject(object) {
  if (!object) {
    return;
  }
  scene.remove(object);
  if (object.geometry) {
    object.geometry.dispose();
  }
  if (object.material) {
    object.material.dispose();
  }
}

function makeCube(center, size, color) {
  const geometry = new THREE.BoxGeometry(size, size, size);
  const material = new THREE.MeshStandardMaterial({ color });
  const cube = new THREE.Mesh(geometry, material);
  cube.position.set(center.x, center.y, center.z);
  return cube;
}

function makeArrow(marker, color) {
  if (!marker.points || marker.points.length < 2) {
    return null;
  }
  const start = new THREE.Vector3(marker.points[0].x, marker.points[0].y, marker.points[0].z);
  const end = new THREE.Vector3(marker.points[1].x, marker.points[1].y, marker.points[1].z);
  const dir = end.clone().sub(start);
  const length = dir.length();
  if (length < 1e-6) {
    return null;
  }
  dir.normalize();
  return new THREE.ArrowHelper(dir, start, length, color, Math.max(voxelSize * 1.5, 0.25), Math.max(voxelSize, 0.18), Math.max(voxelSize * 0.65, 0.12));
}

function setSelectionMarkers(markerArray) {
  clearObject(startArrow);
  clearObject(startCube);
  clearObject(goalArrow);
  clearObject(goalCube);
  startArrow = null;
  startCube = null;
  goalArrow = null;
  goalCube = null;

  for (const marker of markerArray.markers) {
    if (marker.type === 0 && marker.id === 0) {
      startArrow = makeArrow(marker, 0x58ef74);
      if (startArrow) {
        scene.add(startArrow);
      }
    } else if (marker.type === 1 && marker.id === 2) {
      startCube = makeCube(marker.pose.position, marker.scale.x, 0x58ef74);
      scene.add(startCube);
    } else if (marker.type === 0 && marker.id === 1) {
      goalArrow = makeArrow(marker, 0xff6767);
      if (goalArrow) {
        scene.add(goalArrow);
      }
    } else if (marker.type === 1 && marker.id === 3) {
      goalCube = makeCube(marker.pose.position, marker.scale.x, 0xff6767);
      scene.add(goalCube);
    }
  }
}

function setPath(pathMsg) {
  if (pathObject) {
    scene.remove(pathObject);
    pathObject.geometry.dispose();
    if (pathObject.material) {
      pathObject.material.dispose();
    }
    pathObject = null;
  }

  if (navigationControlMode !== "auto") {
    pendingNavigationGoal = null;
    pendingNavigationPath = null;
    clearTrackingPointObjects();
    setSelectionStatus("当前为手动模式：已忽略规划路径，需切回自动并重新设置导航目标。");
    return;
  }

  if (!pathMsg.poses || pathMsg.poses.length < 2) {
    return;
  }

  const points = pathMsg.poses.map(
    (pose) =>
      new THREE.Vector3(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
      ),
  );
  const curve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.TubeGeometry(
    curve,
    Math.max(16, points.length * 3),
    Math.max(voxelSize * 0.22, 0.06),
    12,
    false,
  );
  const material = new THREE.MeshStandardMaterial({
    color: 0x58ef74,
    emissive: 0x266b2e,
    roughness: 0.28,
    metalness: 0.08,
  });
  pathObject = new THREE.Mesh(geometry, material);
  scene.add(pathObject);
  scheduleNavigationConfirmation(pathMsg);
}

function setManualPredictionPath(pathMsg) {
  if (manualPredictionObject) {
    scene.remove(manualPredictionObject);
    disposeObject(manualPredictionObject);
    manualPredictionObject = null;
  }

  if (!pathMsg.poses || pathMsg.poses.length < 2) {
    return;
  }

  const points = pathMsg.poses.map(
    (pose) =>
      new THREE.Vector3(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z + Math.max(voxelSize * 0.3, 0.04),
      ),
  );
  const curve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.TubeGeometry(
    curve,
    Math.max(12, points.length * 2),
    Math.max(voxelSize * 0.14, 0.04),
    10,
    false,
  );
  const material = new THREE.MeshStandardMaterial({
    color: 0xffa040,
    emissive: 0x7a4000,
    roughness: 0.24,
    metalness: 0.05,
  });
  manualPredictionObject = new THREE.Mesh(geometry, material);
  scene.add(manualPredictionObject);

  const end = points[points.length - 1];
  const endpoint = new THREE.Mesh(
    new THREE.SphereGeometry(Math.max(voxelSize * 0.45, 0.10), 20, 12),
    new THREE.MeshStandardMaterial({
      color: 0xffcc60,
      emissive: 0x6a4800,
      roughness: 0.22,
      metalness: 0.04,
    }),
  );
  endpoint.position.copy(end);
  endpoint.name = "manual-prediction-endpoint";
  manualPredictionObject.add(endpoint);
}

function setTrackingPoint(marker) {
  clearTrackingPointObjects();

  if (marker.action === 2) {
    return;
  }

  const scale = Math.max(
    0.08,
    Number(marker.scale?.x || marker.scale?.y || marker.scale?.z || voxelSize * 1.5),
  );
  const geometry = new THREE.SphereGeometry(scale * 0.5, 24, 16);
  const material = new THREE.MeshStandardMaterial({
    color: 0x24a6ff,
    emissive: 0x0b4e86,
    roughness: 0.22,
    metalness: 0.08,
  });
  trackingPointObject = new THREE.Mesh(geometry, material);
  trackingPointObject.name = "current-tracking-point";
  trackingPointObject.position.set(
    marker.pose.position.x,
    marker.pose.position.y,
    marker.pose.position.z,
  );
  scene.add(trackingPointObject);
}

function clearTrackingPointObjects() {
  const staleObjects = [];
  scene.traverse((object) => {
    if (object.name === "current-tracking-point") {
      staleObjects.push(object);
    }
  });

  for (const object of staleObjects) {
    if (object.parent) {
      object.parent.remove(object);
    } else {
      scene.remove(object);
    }
    disposeObject(object);
  }
  trackingPointObject = null;
}

function scheduleNavigationConfirmation(pathMsg) {
  if (!pendingNavigationGoal || !pathMsg.poses || pathMsg.poses.length < 2) {
    return;
  }
  if (navigationConfirmTimer) {
    window.clearTimeout(navigationConfirmTimer);
    navigationConfirmTimer = null;
  }

  pendingNavigationPath = pathMsg;
  resolveNavigationConfirmation(true);
  setSelectionStatus(`路径规划完成：${pathMsg.poses.length} 个路径点，已自动开始导航。`);
}

function publishStartNavigation(shouldStart) {
  if (!startNavigationTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送导航执行确认。");
    return;
  }
  startNavigationTopic.publish(new ROSLIB.Message({ data: shouldStart }));
}

function makeTwist(linearX = 0, linearY = 0, angularZ = 0) {
  return new ROSLIB.Message({
    linear: { x: linearX, y: linearY, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  });
}

function updateManualVelocityDisplay() {
  if (!manualVelocityDisplay) {
    return;
  }
  manualVelocityDisplay.textContent =
    `speed=${joystickCurrentLinearX.toFixed(3)} yaw=${joystickCurrentAngularZ.toFixed(3)}`;
}

function publishManualVelocity(force = false) {
  if (!cmdVelTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送手动速度。");
    updateManualVelocityDisplay();
    return;
  }
  const now = Date.now();
  if (!force && now - joystickLastPublishMs < joystickPublishIntervalMs) {
    return;
  }
  joystickLastPublishMs = now;
  cmdVelTopic.publish(makeTwist(joystickCurrentLinearX, joystickCurrentLinearY, joystickCurrentAngularZ));
  updateManualVelocityDisplay();
}

function publishZeroVelocity() {
  joystickCurrentLinearX = 0;
  joystickCurrentLinearY = 0;
  joystickCurrentAngularZ = 0;
  publishManualVelocity(true);
  stopManualRepeatTimerIfIdle();
}

function setManualVelocity(linearX, angularZ, force = true) {
  joystickCurrentLinearX = linearX;
  joystickCurrentLinearY = 0;
  joystickCurrentAngularZ = angularZ;
  publishManualVelocity(force);
  if (manualVelocityIsActive()) {
    ensureManualRepeatTimer();
  } else {
    stopManualRepeatTimerIfIdle();
  }
}

function setDogModeStatus(text) {
  if (dogModeStatus) {
    dogModeStatus.textContent = text;
  }
  if (unitreeStatus) {
    unitreeStatus.textContent = text;
  }
}

function publishDogModeCommand(mode) {
  if (!dogModeCommandTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送宇树模式命令。");
    return;
  }
  dogModeCommandTopic.publish(new ROSLIB.Message({ data: mode }));
  setDogModeStatus(`已发送 ${mode}`);
  setSelectionStatus(`已发送宇树高层模式命令：${mode}`);
}

function updateUnitreeStatus(msg) {
  const text = msg && msg.data ? msg.data : "等待控制桥状态";
  setDogModeStatus(text);
}

function applyJoystickSpeedCurve(normalizedValue, maxSpeed) {
  const sign = Math.sign(normalizedValue);
  const magnitude = Math.abs(normalizedValue);
  if (magnitude < joystickDeadband) {
    return 0;
  }

  const scaled = (magnitude - joystickDeadband) / (1 - joystickDeadband) * maxSpeed;
  return sign * Math.max(joystickMinCommandSpeed, scaled);
}

function manualVelocityIsActive() {
  return (
    Math.abs(joystickCurrentLinearX) > 1e-6 ||
    Math.abs(joystickCurrentLinearY) > 1e-6 ||
    Math.abs(joystickCurrentAngularZ) > 1e-6
  );
}

function ensureManualRepeatTimer() {
  if (joystickRepeatTimer) {
    return;
  }
  joystickRepeatTimer = window.setInterval(() => {
    if (manualVelocityIsActive()) {
      publishManualVelocity(true);
    }
  }, 100);
}

function stopManualRepeatTimerIfIdle() {
  if (!joystickRepeatTimer || manualVelocityIsActive()) {
    return;
  }
  window.clearInterval(joystickRepeatTimer);
  joystickRepeatTimer = null;
}

function stopNavigation() {
  if (!stopNavigationTopic) {
    setSelectionStatus("ROSBridge 未连接，无法发送停止导航命令。");
    return;
  }
  stopPathTrackingForManualMotion();
  publishZeroVelocity();
  setSelectionStatus("已发送停止导航命令：路径跟踪已中止，并请求底盘速度归零。");
}

function stopPathTrackingForManualMotion() {
  pendingNavigationGoal = null;
  pendingNavigationPath = null;
  hideNavigationConfirmModal();
  if (navigationConfirmTimer) {
    window.clearTimeout(navigationConfirmTimer);
    navigationConfirmTimer = null;
  }
  if (!stopNavigationTopic) {
    return;
  }
  publishStartNavigation(false);
  stopNavigationTopic.publish(new ROSLIB.Message({ data: true }));
}

function resetJoystickKnob() {
  joystickKnob.style.transform = "translate(-50%, -50%)";
  joystickKnob.classList.remove("active");
}

function updateJoystickFromEvent(event, forcePublish = false) {
  const rect = joystickPad.getBoundingClientRect();
  const radius = rect.width * 0.5;
  const knobRadius = joystickKnob.getBoundingClientRect().width * 0.5;
  const maxOffset = Math.max(1, radius - knobRadius - 6);
  const centerX = rect.left + radius;
  const centerY = rect.top + radius;
  let dx = event.clientX - centerX;
  let dy = event.clientY - centerY;
  const distance = Math.hypot(dx, dy);
  if (distance > maxOffset) {
    dx = dx / distance * maxOffset;
    dy = dy / distance * maxOffset;
  }

  joystickKnob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

  const normalizedY = dy / maxOffset;
  const normalizedX = dx / maxOffset;
  joystickCurrentLinearX = applyJoystickSpeedCurve(-normalizedY, joystickMaxLinearX);
  joystickCurrentLinearY = 0;
  joystickCurrentAngularZ = applyJoystickSpeedCurve(-normalizedX, joystickMaxAngularZ);
  publishManualVelocity(forcePublish);
}

function startJoystickControl(event) {
  if (event.button !== undefined && event.button !== 0) {
    return;
  }
  event.preventDefault();
  joystickActivePointerId = event.pointerId;
  joystickKnob.classList.add("active");
  joystickKnob.setPointerCapture(event.pointerId);
  updateJoystickFromEvent(event, true);
  ensureManualRepeatTimer();
  setSelectionStatus("手动运动控制中：松开小圆将回中并发送零速度。");
}

function moveJoystickControl(event) {
  if (joystickActivePointerId !== event.pointerId) {
    return;
  }
  event.preventDefault();
  updateJoystickFromEvent(event);
}

function endJoystickControl(event) {
  if (joystickActivePointerId !== event.pointerId) {
    return;
  }
  event.preventDefault();
  joystickActivePointerId = null;
  joystickCurrentLinearX = 0;
  joystickCurrentLinearY = 0;
  joystickCurrentAngularZ = 0;
  resetJoystickKnob();
  publishManualVelocity(true);
  stopManualRepeatTimerIfIdle();
  setSelectionStatus("手动运动已停止，已发送零速度。");
  if (joystickKnob.hasPointerCapture(event.pointerId)) {
    joystickKnob.releasePointerCapture(event.pointerId);
  }
}


function hideNavigationConfirmModal() {
  navigationConfirmModal.hidden = true;
}

function resolveNavigationConfirmation(shouldStart) {
  if (!pendingNavigationGoal && !pendingNavigationPath) {
    hideNavigationConfirmModal();
    return;
  }
  pendingNavigationGoal = null;
  pendingNavigationPath = null;
  hideNavigationConfirmModal();
  publishStartNavigation(shouldStart);
  setSelectionStatus(
    shouldStart
      ? "已确认开始导航，路径跟踪已启动。"
      : "已取消导航执行，仅显示规划路线，不进行路径跟踪。",
  );
}

function makePointStamped(x, y, z) {
  const now = Date.now();
  return new ROSLIB.Message({
    header: {
      frame_id: navigationMapFrameId,
      stamp: {
        secs: Math.floor(now / 1000),
        nsecs: (now % 1000) * 1000000,
      },
    },
    point: { x, y, z },
  });
}

function makePoseStamped(x, y, z, yaw) {
  const now = Date.now();
  const halfYaw = yaw * 0.5;
  return new ROSLIB.Message({
    header: {
      frame_id: navigationMapFrameId,
      stamp: {
        secs: Math.floor(now / 1000),
        nsecs: (now % 1000) * 1000000,
      },
    },
    pose: {
      position: { x, y, z },
      orientation: {
        x: 0,
        y: 0,
        z: Math.sin(halfYaw),
        w: Math.cos(halfYaw),
      },
    },
  });
}

function setPointVisual(kind, center, yaw) {
  const isStart = kind === "start";
  const color = isStart ? 0x58ef74 : 0xff6767;
  const arrowRef = isStart ? startArrow : goalArrow;
  const cubeRef = isStart ? startCube : goalCube;

  clearObject(arrowRef);
  clearObject(cubeRef);
  if (isStart) {
    startArrow = null;
    startCube = null;
  } else {
    goalArrow = null;
    goalCube = null;
  }

  const cube = makeCube(center, Math.max(voxelSize, 0.18), color);
  scene.add(cube);
  const direction = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
  const arrowLength = Math.max(voxelSize * 2.5, 0.45);
  const arrow = new THREE.ArrowHelper(
    direction.normalize(),
    new THREE.Vector3(center.x, center.y, center.z),
    arrowLength,
    color,
    Math.max(voxelSize * 1.5, 0.25),
    Math.max(voxelSize, 0.18),
  );
  scene.add(arrow);

  if (isStart) {
    startCube = cube;
    startArrow = arrow;
  } else {
    goalCube = cube;
    goalArrow = arrow;
  }
}

function yawFromQuaternion(qx, qy, qz, qw) {
  const sinyCosp = 2.0 * (qw * qz + qx * qy);
  const cosyCosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return Math.atan2(sinyCosp, cosyCosp);
}

function normalizeFrameId(frameId) {
  if (!frameId) {
    return "";
  }
  return String(frameId).replace(/^\/+/, "");
}

function setOdomPose(msg) {
  if (!msg || !msg.pose || !msg.pose.pose) {
    return;
  }
  const pose = msg.pose.pose;
  latestOdomPose = {
    x: pose.position.x,
    y: pose.position.y,
    z: pose.position.z,
    yaw: yawFromQuaternion(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w,
    ),
    frame: normalizeFrameId(msg.header && msg.header.frame_id) || navigationMapFrameId,
  };
  setRobotTfStatus(`使用 /pointlio/odom，frame=${latestOdomPose.frame}`);
  setRelocalizationStatus(true);
}

function updateRelocalizationStatus() {
  setRelocalizationStatus(Boolean(latestOdomPose));
}

function resolveRobotPose() {
  if (!latestOdomPose) {
    setRobotTfStatus("未收到 /pointlio/odom");
    return null;
  }
  return latestOdomPose;
}

function updateRobotVisual() {
  if (!robotObject) {
    return;
  }
  const robotPose = resolveRobotPose();
  if (!robotPose) {
    robotObject.visible = false;
    return;
  }

  robotObject.visible = true;
  robotObject.position.set(
    robotPose.x + robotDisplayOffset.x,
    robotPose.y + robotDisplayOffset.y,
    robotPose.z + robotDisplayOffset.z,
  );
  robotObject.rotation.set(0, 0, robotPose.yaw);
}

function publishStartPose(intersectionPoint, yaw) {
  if (!startTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  startTopic.publish(makePointStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z));
  setPointVisual("start", intersectionPoint, yaw);
  setSelectionStatus(
    `起始点已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°。`,
  );
}

function publishGoalPose(intersectionPoint, yaw) {
  if (!goalTopic || !goalPoseTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  goalTopic.publish(makePointStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z));
  goalPoseTopic.publish(makePoseStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, yaw));
  setPointVisual("goal", intersectionPoint, yaw);
  setSelectionStatus(
    `目标点已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°，正在规划。`,
  );
}

function publishNavigationGoal(intersectionPoint, yaw) {
  if (navigationControlMode !== "auto") {
    pendingNavigationGoal = null;
    pendingNavigationPath = null;
    setSelectionStatus("当前为手动模式：请先切换到自动模式，再重新设置导航目标。");
    return;
  }
  if (!startTopic || !goalTopic || !goalPoseTopic || !startNavigationTopic) {
    setSelectionStatus("ROSBridge 未连接。");
    return;
  }
  const robotPose = resolveRobotPose();
  if (!robotPose) {
    setSelectionStatus("未收到 /pointlio/odom，无法设置导航目标。");
    return;
  }
  if (stopNavigationTopic) {
    stopNavigationTopic.publish(new ROSLIB.Message({ data: true }));
  }
  hideNavigationConfirmModal();
  pendingNavigationGoal = {
    goal: { x: intersectionPoint.x, y: intersectionPoint.y, z: intersectionPoint.z },
    yaw,
  };
  startTopic.publish(makePointStamped(robotPose.x, robotPose.y, robotPose.z));
  goalTopic.publish(makePointStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z));
  goalPoseTopic.publish(
    makePoseStamped(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, yaw),
  );
  setPointVisual("start", { x: robotPose.x, y: robotPose.y, z: robotPose.z }, robotPose.yaw);
  setPointVisual("goal", intersectionPoint, yaw);
  setSelectionStatus(
    `导航目标已设置：[${intersectionPoint.x.toFixed(2)}, ${intersectionPoint.y.toFixed(2)}, ${intersectionPoint.z.toFixed(2)}]，朝向 ${(yaw * 180 / Math.PI).toFixed(1)}°。正在规划路径，路径生成后自动开始导航。`,
  );
}

function updatePointerFromEvent(event) {
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
}

function pickPointOnGlobalMap(event) {
  if (!retainedMapCloudObject) {
    return null;
  }
  updatePointerFromEvent(event);
  raycaster.params.Points = raycaster.params.Points || {};
  raycaster.params.Points.threshold = Math.max(voxelSize * 1.8, 0.35);
  const hits = raycaster.intersectObject(retainedMapCloudObject, false);
  return hits.length > 0 ? hits[0].point.clone() : null;
}

function getNavigationPickBounds() {
  return latestMapBounds;
}

function getNavigationPlaneZ() {
  if (latestOdomPose && Number.isFinite(latestOdomPose.z)) {
    return latestOdomPose.z;
  }
  const bounds = getNavigationPickBounds();
  return bounds ? Math.min(0, bounds.min.z) : 0;
}

function pointInNavigationBounds(point) {
  const bounds = getNavigationPickBounds();
  if (!bounds) {
    return false;
  }
  const margin = Math.max(voxelSize * 4.0, 0.5);
  return (
    point.x >= bounds.min.x - margin &&
    point.x <= bounds.max.x + margin &&
    point.y >= bounds.min.y - margin &&
    point.y <= bounds.max.y + margin
  );
}

function pickNavigationPoint(event) {
  const globalPoint = pickPointOnGlobalMap(event);
  if (globalPoint && pointInNavigationBounds(globalPoint)) {
    return globalPoint;
  }

  const point = pickPointOnHeightPlane(event, getNavigationPlaneZ());
  if (!point || !pointInNavigationBounds(point)) {
    return null;
  }
  return point;
}

function pickPointOnHeightPlane(event, planeZ) {
  updatePointerFromEvent(event);
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ);
  const hit = new THREE.Vector3();
  const ok = raycaster.ray.intersectPlane(plane, hit);
  return ok ? hit : null;
}

function computeYawFromPoints(start, end, fallbackYaw = 0) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
    return fallbackYaw;
  }
  return Math.atan2(dy, dx);
}

function scheduleReconnect() {
  if (reconnectTimer) {
    return;
  }
  reconnectTimer = window.setTimeout(() => {
    reconnectTimer = null;
    connectRosbridge(wsInput.value.trim(), false);
  }, 1500);
}

function connectRosbridge(url = wsInput.value.trim(), manual = false) {
  wsInput.value = url;
  if (ros) {
    ros.close();
  }

  setConnectionStatus("Connecting");
  setSelectionStatus(`正在连接 ${url} ...`);

  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    setConnectionStatus("已连接");
    setSelectionStatus("已连接。点击“导航目标”“起始点”或“目标点”，再在点云地图范围按下、拖动、松开设置姿态。");
    advancedSettings.open = false;

    startTopic = new ROSLIB.Topic({
      ros,
      name: "/start_point",
      messageType: "geometry_msgs/PointStamped",
    });
    goalTopic = new ROSLIB.Topic({
      ros,
      name: "/goal_point",
      messageType: "geometry_msgs/PointStamped",
    });
    goalPoseTopic = new ROSLIB.Topic({
      ros,
      name: "/goal_pose",
      messageType: "geometry_msgs/PoseStamped",
    });
    startNavigationTopic = new ROSLIB.Topic({
      ros,
      name: "/start_navigation",
      messageType: "std_msgs/Bool",
    });
    stopNavigationTopic = new ROSLIB.Topic({
      ros,
      name: "/stop_navigation",
      messageType: "std_msgs/Bool",
    });
    goalToleranceTopic = new ROSLIB.Topic({
      ros,
      name: "/navigation/goal_tolerance",
      messageType: "std_msgs/Float64",
    });
    navigationControlModeTopic = new ROSLIB.Topic({
      ros,
      name: "/navigation/control_mode",
      messageType: "std_msgs/String",
    });
    publishGoalTolerance(goalToleranceMeters);
    publishNavigationControlMode(navigationControlMode);
    publishLocalAvoidanceSettings(localAvoidanceSettings || defaultLocalAvoidanceSettings);
    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: "/web_cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    dogModeCommandTopic = new ROSLIB.Topic({
      ros,
      name: "/unitree_highlevel/mode_cmd",
      messageType: "std_msgs/String",
    });
    unitreeStatusTopic = new ROSLIB.Topic({
      ros,
      name: "/unitree_highlevel/status",
      messageType: "std_msgs/String",
    });
    unitreeStatusTopic.subscribe(updateUnitreeStatus);
    odomTopic = new ROSLIB.Topic({
      ros,
      name: "/pointlio/odom",
      messageType: "nav_msgs/Odometry",
    });
    odomTopic.subscribe(setOdomPose);
    startCameraPreviews();

    const mapCloudTopic = new ROSLIB.Topic({
      ros,
      name: "/navigation_map_cloud",
      messageType: "sensor_msgs/PointCloud2",
    });
    mapCloudTopic.subscribe(setRetainedMapCloud);

    const localMapCloudTopic = new ROSLIB.Topic({
      ros,
      name: "/local_navigation_map_cloud",
      messageType: "sensor_msgs/PointCloud2",
    });
    localMapCloudTopic.subscribe(setLocalMapCloud);

    const selectionTopic = new ROSLIB.Topic({
      ros,
      name: "/selection_markers",
      messageType: "visualization_msgs/MarkerArray",
    });
    selectionTopic.subscribe(setSelectionMarkers);

    const globalPathTopic = new ROSLIB.Topic({
      ros,
      name: "/global_plan",
      messageType: "nav_msgs/Path",
    });
    globalPathTopic.subscribe(setPath);

    const localPathTopic = new ROSLIB.Topic({
      ros,
      name: "/local_plan",
      messageType: "nav_msgs/Path",
    });
    localPathTopic.subscribe(setManualPredictionPath);

    const trackingPointTopic = new ROSLIB.Topic({
      ros,
      name: "/tracking_point_marker",
      messageType: "visualization_msgs/Marker",
    });
    trackingPointTopic.subscribe(setTrackingPoint);

    const statusTopic = new ROSLIB.Topic({
      ros,
      name: "/web_selection_status",
      messageType: "std_msgs/String",
    });
    statusTopic.subscribe((msg) => setSelectionStatus(msg.data));

    updateRelocalizationStatus();
  });

  ros.on("error", (error) => {
    setConnectionStatus("错误");
    setSelectionStatus(`ROSBridge 错误：${error}`);
    advancedSettings.open = true;
  });

  ros.on("close", () => {
    setConnectionStatus("未连接");
    startTopic = null;
    goalTopic = null;
    goalPoseTopic = null;
    startNavigationTopic = null;
    stopNavigationTopic = null;
    goalToleranceTopic = null;
    navigationControlModeTopic = null;
    localAvoidanceConfigTopic = null;
    cmdVelTopic = null;
    dogModeCommandTopic = null;
    if (unitreeStatusTopic) {
      unitreeStatusTopic.unsubscribe();
    }
    unitreeStatusTopic = null;
    setDogModeStatus("未连接");
    odomTopic = null;
    latestOdomPose = null;
    if (manualPredictionObject) {
      scene.remove(manualPredictionObject);
      disposeObject(manualPredictionObject);
      manualPredictionObject = null;
    }
    updateRelocalizationStatus();
    pendingNavigationGoal = null;
    pendingNavigationPath = null;
    hideNavigationConfirmModal();
    if (navigationConfirmTimer) {
      window.clearTimeout(navigationConfirmTimer);
      navigationConfirmTimer = null;
    }
    advancedSettings.open = true;
    if (!manual) {
      scheduleReconnect();
    }
  });
}

function setPlacementMode(mode) {
  if (mode === "navigate" && navigationControlMode !== "auto") {
    placementMode = null;
    setNavigateBtn.classList.remove("active");
    setStartBtn.classList.remove("active");
    setGoalBtn.classList.remove("active");
    setSelectionStatus("当前为手动模式：请先切换到自动模式，再设置导航目标。");
    return;
  }
  placementMode = mode;
  setNavigateBtn.classList.toggle("active", mode === "navigate");
  setStartBtn.classList.toggle("active", mode === "start");
  setGoalBtn.classList.toggle("active", mode === "goal");
  if (mode === "navigate") {
    setSelectionStatus("导航目标模式：松开后规划路线，路径生成后自动开始导航。");
  } else if (mode === "start") {
    setSelectionStatus("起始点模式：在点云地图范围按下，拖动调整朝向，松开确认。");
  } else if (mode === "goal") {
    setSelectionStatus("目标点模式：在点云地图范围按下，拖动调整朝向，松开确认并开始规划。");
  } else {
    setSelectionStatus("已连接。点击“导航目标”“起始点”或“目标点”，再在点云地图范围按下、拖动、松开设置姿态。");
  }
}

setNavigateBtn.addEventListener("click", () => setPlacementMode("navigate"));
setStartBtn.addEventListener("click", () => setPlacementMode("start"));
setGoalBtn.addEventListener("click", () => setPlacementMode("goal"));
stopNavigationBtn.addEventListener("click", stopNavigation);
if (navModeToggleBtn) {
  navModeToggleBtn.addEventListener("click", toggleNavigationControlMode);
}
if (saveGoalToleranceBtn) {
  saveGoalToleranceBtn.addEventListener("click", saveNavigationSettings);
}
settingsTabButtons.forEach((button) => {
  button.addEventListener("click", () => setSettingsTab(button.dataset.settingsTab));
});
localAvoidanceInputs.forEach((input) => {
  input.addEventListener("input", scheduleLocalAvoidancePublish);
  input.addEventListener("change", scheduleLocalAvoidancePublish);
});
if (applyLocalAvoidanceBtn) {
  applyLocalAvoidanceBtn.addEventListener("click", () => publishLocalAvoidanceSettings());
}
if (saveLocalAvoidanceBtn) {
  saveLocalAvoidanceBtn.addEventListener("click", saveLocalAvoidanceSettings);
}
dogModeButtons.forEach((button) => {
  button.addEventListener("click", () => publishDogModeCommand(button.dataset.dogMode));
});
dogMotionButtons.forEach((button) => {
  const startMotion = (event) => {
    if (event.button !== undefined && event.button !== 0) {
      return;
    }
    event.preventDefault();
    stopPathTrackingForManualMotion();
    const speed = Number(button.dataset.motionSpeed || 0);
    const yaw = Number(button.dataset.motionYaw || 0);
    setManualVelocity(speed, yaw, true);
    if (button.setPointerCapture && event.pointerId !== undefined) {
      button.setPointerCapture(event.pointerId);
    }
    setSelectionStatus(`手动速度/yaw：speed=${speed.toFixed(3)} yaw=${yaw.toFixed(3)}，松开后归零。`);
  };
  const stopMotion = (event) => {
    if (event) {
      event.preventDefault();
    }
    publishZeroVelocity();
    if (event && button.hasPointerCapture && event.pointerId !== undefined && button.hasPointerCapture(event.pointerId)) {
      button.releasePointerCapture(event.pointerId);
    }
    setSelectionStatus("手动速度/yaw 已归零。");
  };
  button.addEventListener("pointerdown", startMotion);
  button.addEventListener("pointerup", stopMotion);
  button.addEventListener("pointercancel", stopMotion);
  button.addEventListener("lostpointercapture", stopMotion);
});
joystickKnob.addEventListener("pointerdown", startJoystickControl);
joystickKnob.addEventListener("pointermove", moveJoystickControl);
joystickKnob.addEventListener("pointerup", endJoystickControl);
joystickKnob.addEventListener("pointercancel", endJoystickControl);

navigationConfirmStartBtn.addEventListener("click", () => resolveNavigationConfirmation(true));
navigationConfirmCancelBtn.addEventListener("click", () => resolveNavigationConfirmation(false));
resetViewBtn.addEventListener("click", resetMapView);

canvas.addEventListener("pointerdown", (event) => {
  if (!placementMode) {
    setSelectionStatus("请先点击“导航目标”“起始点”或“目标点”按钮。");
    return;
  }
  if (event.button !== 0) {
    return;
  }
  const point = pickNavigationPoint(event);
  if (!point) {
    setSelectionStatus("没有点中全局点云地图范围，请在全局地图底图上选择可导航位置设置导航目标。");
    return;
  }
  activePointerId = event.pointerId;
  navigationDrag = {
    start: point,
    yaw: 0,
    mode: placementMode,
  };
  controls.enabled = false;
  canvas.setPointerCapture(event.pointerId);
  setPointVisual(
    placementMode === "start" ? "start" : "goal",
    point,
    0,
  );
  setSelectionStatus(
    placementMode === "navigate"
      ? "导航目标位置已设置。保持按下并拖动以调整红色箭头朝向。"
      : placementMode === "start"
        ? "起始点位置已设置。保持按下并拖动以调整绿色箭头朝向。"
        : "目标点位置已设置。保持按下并拖动以调整红色箭头朝向。"
  );
});

canvas.addEventListener("pointermove", (event) => {
  if (activePointerId !== event.pointerId || !navigationDrag) {
    return;
  }
  const point = pickPointOnHeightPlane(event, navigationDrag.start.z);
  if (!point) {
    return;
  }
  navigationDrag.yaw = computeYawFromPoints(navigationDrag.start, point, navigationDrag.yaw);
  setPointVisual(
    navigationDrag.mode === "start" ? "start" : "goal",
    navigationDrag.start,
    navigationDrag.yaw,
  );
});

canvas.addEventListener("pointerup", (event) => {
  if (activePointerId !== event.pointerId || !navigationDrag) {
    return;
  }
  event.preventDefault();
  const point = pickPointOnHeightPlane(event, navigationDrag.start.z);
  if (point) {
    navigationDrag.yaw = computeYawFromPoints(navigationDrag.start, point, navigationDrag.yaw);
  }
  if (navigationDrag.mode === "start") {
    publishStartPose(navigationDrag.start, navigationDrag.yaw);
  } else if (navigationDrag.mode === "goal") {
    publishGoalPose(navigationDrag.start, navigationDrag.yaw);
  } else {
    publishNavigationGoal(navigationDrag.start, navigationDrag.yaw);
  }
  navigationDrag = null;
  activePointerId = null;
  controls.enabled = true;
  setPlacementMode(null);
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
});

canvas.addEventListener("pointercancel", (event) => {
  if (activePointerId !== event.pointerId) {
    return;
  }
  navigationDrag = null;
  activePointerId = null;
  controls.enabled = true;
  setPlacementMode(null);
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
});

connectBtn.addEventListener("click", () => connectRosbridge(wsInput.value.trim(), true));
reconnectBtn.addEventListener("click", () => connectRosbridge(defaultRosbridgeUrl(), true));
window.addEventListener("resize", updateRendererSize);
setGoalToleranceUi(goalToleranceMeters, false);
setNavigationControlMode("auto", { quiet: true });
setLocalAvoidanceUi(defaultLocalAvoidanceSettings, false);
loadNavigationSettings();
loadLocalAvoidanceSettings();
updateRendererSize();
startCameraPreviews();
connectRosbridge(defaultRosbridgeUrl(), false);

function animate() {
  controls.update();
  updateRobotVisual();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

window.setInterval(updateRelocalizationStatus, 1000);

animate();
