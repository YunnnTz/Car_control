// ── Camera buttons ──────────────────────────────────────────
const CAMERA_REPEAT_MS = 120; // 按住時每 120 ms 重送一次，讓雲台持續轉動

let cameraRepeatTimer = null;

function sendCamera(direction) {
  const message = { type: 'camera', direction };

  if (motorSocket && motorSocket.readyState === WebSocket.OPEN) {
    motorSocket.send(JSON.stringify(message));
  }
}

function camPress(direction) {
  // 先清掉舊的計時器：手機上 touchstart 和 mousedown 會各觸發一次，
  // 而且可能有另一顆按鈕還按著沒放開
  camRelease();
  sendCamera(direction);
  cameraRepeatTimer = setInterval(() => sendCamera(direction), CAMERA_REPEAT_MS);
}

function camRelease() {
  clearInterval(cameraRepeatTimer);
  cameraRepeatTimer = null;
}

// ── Mode switch & face detection ────────────────────────────
const modeManualBtn = document.getElementById('btn-mode-manual');
const modeAutoBtn = document.getElementById('btn-mode-auto');
const camGrid = document.getElementById('cam-grid');
const faceTrackBtn = document.getElementById('btn-face-track');
const trackLabel = document.getElementById('track-label');
const targetStatus = document.getElementById('target-status');
const targetText = document.getElementById('target-text');
const alertPatrol = document.getElementById('alert-patrol');
const angleReadout = document.getElementById('angle-readout');

let mode = 'manual';   // 開啟網頁時預設手動
let detectOn = false;  // 手動模式下使用者自己開的偵測

function send(message) {
  if (motorSocket && motorSocket.readyState === WebSocket.OPEN) {
    motorSocket.send(JSON.stringify(message));
  }
}

function applyMode() {
  const auto = mode === 'auto';
  modeAutoBtn.classList.toggle('active', auto);
  modeManualBtn.classList.toggle('active', !auto);

  // 自動模式由巡邏和追蹤接管，方向鍵鎖住
  camGrid.classList.toggle('locked', auto);
  camRelease();

  // 自動模式的偵測強制開啟且不可關；手動模式才交還給使用者
  faceTrackBtn.classList.toggle('on', auto || detectOn);
  faceTrackBtn.disabled = auto;
  trackLabel.textContent = auto ? '人臉追蹤（自動）' : '人臉偵測';
}

function setMode(next) {
  mode = next;
  applyMode();
  send({ type: 'mode', mode });
  if (next === 'manual') send({ type: 'detect', enabled: detectOn });
}

modeManualBtn.addEventListener('click', () => setMode('manual'));
modeAutoBtn.addEventListener('click', () => setMode('auto'));

faceTrackBtn.addEventListener('click', () => {
  if (mode === 'auto') return;   // 自動模式不讓關
  detectOn = !detectOn;
  faceTrackBtn.classList.toggle('on', detectOn);
  send({ type: 'detect', enabled: detectOn });
});

function syncState() {
  send({ type: 'mode', mode });
  send({ type: 'detect', enabled: detectOn });
}

// 後端每 0.25 秒推一次狀態，用來更新提示與角度
function handleStatus(data) {
  // 目標狀態列一直都在，只換顏色和文字，不會整個消失
  targetStatus.classList.toggle('found', data.face || data.locked);
  if (data.locked) {
    // 自動模式抓到第一張臉就鎖定，偵測已停止，要跟「還在找」明確區分開
    targetText.textContent = '已鎖定目標（偵測停止）';
  } else {
    targetText.textContent = data.face ? '偵測到目標' : '未偵測到目標';
  }

  alertPatrol.classList.toggle('show', data.patrol);
  angleReadout.textContent = `${data.pan}° / ${data.tilt}°`;
}

// ── 車身姿態（MPU6050）──────────────────────────────────────
const attitude = document.getElementById('attitude');
const horizon = document.getElementById('horizon');
const imuRoll = document.getElementById('imu-roll');
const imuPitch = document.getElementById('imu-pitch');
const imuYaw = document.getElementById('imu-yaw');
const imuAccel = document.getElementById('imu-accel');

const HORIZON_PX_PER_DEG = 1.1;  // 俯仰角換算成天地線要上下平移幾 px
const HORIZON_MAX_DEG = 30;      // 超過就不再往外移，免得天地線整個滑出圓框

// 後端每 0.2 秒推一次（server.py 的 IMU_PERIOD_S）
function handleImu(data) {
  attitude.classList.toggle('offline', !data.ok);

  if (!data.ok) {
    imuRoll.textContent = '--';
    imuPitch.textContent = '--';
    imuYaw.textContent = '--';
    imuAccel.textContent = '--';
    return;
  }

  imuRoll.textContent = `${data.roll.toFixed(1)}°`;
  imuPitch.textContent = `${data.pitch.toFixed(1)}°`;
  imuYaw.textContent = `${data.yaw.toFixed(1)}°`;
  imuAccel.textContent = `${data.accel.toFixed(2)} g`;

  // 天地線往車身的反方向轉，外框不動 —— 跟飛機的人工地平儀一樣。
  // 先轉再平移，平移才會沿著車身自己的上下方向走。
  // 裝好之後如果轉的方向相反，把這兩個負號改成正的。
  const shift = clamp(data.pitch, -HORIZON_MAX_DEG, HORIZON_MAX_DEG);
  horizon.style.transform =
    `rotate(${-data.roll}deg) translateY(${shift * HORIZON_PX_PER_DEG}px)`;
}

// 按著按鈕把手指／滑鼠滑出去再放開時，按鈕本身收不到 release，
// 所以在 document 上補一層，避免雲台停不下來
document.addEventListener('mouseup', camRelease);
document.addEventListener('touchend', camRelease);
document.addEventListener('touchcancel', camRelease);
window.addEventListener('blur', camRelease);

// ── Joystick ─────────────────────────────────────────────────
const ring = document.getElementById('joystick-ring');
const knob = document.getElementById('joystick-knob');

let joystickActive = false;
let currentLeft = 0;
let currentRight = 0;
let signalInterval = null;
let motorSocket = null;
let reconnectTimer = null;
let pageUnloading = false;

const SIGNAL_RATE_MS = 100; // emit signal every 100 ms while held
const RECONNECT_DELAY_MS = 1000;

const connectionDot = document.getElementById('connection-dot');
const connectionStatus = document.getElementById('connection-status');

function setConnectionStatus(connected, label) {
  connectionDot.classList.toggle('dot-offline', !connected);
  connectionStatus.classList.toggle('status-offline', !connected);
  connectionStatus.textContent = label;
}

function motorWebSocketUrl() {
  const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${window.location.hostname}:8080/ws`;
}

function sendMotor(left, right) {
  const message = { type: 'motor', left, right };

  if (motorSocket && motorSocket.readyState === WebSocket.OPEN) {
    motorSocket.send(JSON.stringify(message));
  }

  console.log('motor', message);
}

function connectMotorWebSocket() {
  if (pageUnloading ||
      (motorSocket && (motorSocket.readyState === WebSocket.OPEN ||
                       motorSocket.readyState === WebSocket.CONNECTING))) {
    return;
  }

  setConnectionStatus(false, '連線中');
  motorSocket = new WebSocket(motorWebSocketUrl());

  motorSocket.addEventListener('open', () => {
    setConnectionStatus(true, '已連接');
    sendMotor(0, 0);
    syncState();   // 重連後把模式與偵測狀態同步回伺服器
  });

  motorSocket.addEventListener('message', (event) => {
    let data;
    try {
      data = JSON.parse(event.data);
    } catch (err) {
      return;
    }
    if (data.type === 'status') handleStatus(data);
    else if (data.type === 'imu') handleImu(data);
  });

  motorSocket.addEventListener('close', () => {
    setConnectionStatus(false, '未連接');
    motorSocket = null;
    if (!pageUnloading && !reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectMotorWebSocket();
      }, RECONNECT_DELAY_MS);
    }
  });

  motorSocket.addEventListener('error', () => {
    setConnectionStatus(false, '連線錯誤');
  });
}

// ── Camera stream ────────────────────────────────────────────
const videoStream = document.getElementById('video-stream');
const panelCenter = document.getElementById('panel-center');
const streamRes = document.getElementById('stream-res');

const STREAM_RETRY_MS = 2000;
let streamRetryTimer = null;
let streamAlive = false;

function setStreamAlive(alive) {
  streamAlive = alive;
  panelCenter.classList.toggle('stream-ok', alive);
}

function reloadStream() {
  streamRetryTimer = null;
  videoStream.src = `/stream?t=${Date.now()}`; // 加時間戳，避免拿到快取的舊串流
}

videoStream.addEventListener('error', () => {
  setStreamAlive(false);
  if (!pageUnloading && !streamRetryTimer) {
    streamRetryTimer = setTimeout(reloadStream, STREAM_RETRY_MS);
  }
});

// MJPEG 是持續不斷的 multipart 回應，串流結束前 load 事件不會觸發，
// 所以改用 naturalWidth 判斷第一張影格有沒有真的進來。
setInterval(() => {
  if (streamAlive || videoStream.naturalWidth === 0) return;
  setStreamAlive(true);
  streamRes.textContent = `${videoStream.naturalWidth}×${videoStream.naturalHeight}`;
}, 500);

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function xyToWheelSpeed(x, y) {
  // x: -100 (left) to 100 (right)
  // y: -100 (backward) to 100 (forward)
  
  // Convert joystick position to differential drive
  // left wheel = forward/backward + turn adjustment
  // right wheel = forward/backward - turn adjustment
  
  let left = y + x;
  let right = y - x;
  
  // Clamp to -100 to 100 range
  left = clamp(left, -100, 100);
  right = clamp(right, -100, 100);
  
  return { left: Math.round(left), right: Math.round(right) };
}

function quantizeJoystick(dx, dy, maxR) {
  // Calculate normalized distance (-1 to 1 range)
  const normalizedX = dx / maxR;
  const normalizedY = dy / maxR;
  
  // Scale to -100 to 100 range
  let x = Math.round(normalizedX * 100);
  let y = Math.round(-normalizedY * 100); // Invert Y so up is positive
  
  // Clamp values
  x = clamp(x, -100, 100);
  y = clamp(y, -100, 100);
  
  // Dead zone threshold (about 12%)
  const threshold = 12;
  if (Math.abs(x) < threshold && Math.abs(y) < threshold) {
    return { x: 0, y: 0 };
  }
  
  return { x, y };
}

function startSignal(left, right) {
  if (left === currentLeft && right === currentRight) return;
  
  currentLeft = left;
  currentRight = right;
  
  sendMotor(left, right);
  
  clearInterval(signalInterval);
  if (left !== 0 || right !== 0) {
    signalInterval = setInterval(() => {
      sendMotor(left, right);
    }, SIGNAL_RATE_MS);
  }
  
  // Update visual feedback based on direction
  updateDirectionArrows(left, right);
}

function updateDirectionArrows(left, right) {
  ring.classList.remove('dir-up', 'dir-down', 'dir-left', 'dir-right');
  
  const threshold = 20;
  const avg = (left + right) / 2;
  const diff = left - right;
  
  if (Math.abs(avg) > Math.abs(diff)) {
    // Primarily forward/backward
    if (avg > threshold) ring.classList.add('dir-up');
    else if (avg < -threshold) ring.classList.add('dir-down');
  } else {
    // Primarily turning
    if (diff > threshold) ring.classList.add('dir-left');
    else if (diff < -threshold) ring.classList.add('dir-right');
  }
}

function stopSignal() {
  clearInterval(signalInterval);
  signalInterval = null;
  currentLeft = 0;
  currentRight = 0;
  sendMotor(0, 0);
  ring.classList.remove('dir-up', 'dir-down', 'dir-left', 'dir-right');
  knob.classList.remove('active');
  knob.style.transform = '';
}

function handleMove(clientX, clientY) {
  const rect = ring.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top  + rect.height / 2;
  const dx = clientX - cx;
  const dy = clientY - cy;

  // clamp knob inside ring
  const maxR = rect.width / 2 - knob.offsetWidth / 2 - 4;
  const dist = Math.sqrt(dx * dx + dy * dy);
  const clampedDist = Math.min(dist, maxR);
  const angle = Math.atan2(dy, dx);
  const kx = Math.cos(angle) * clampedDist;
  const ky = Math.sin(angle) * clampedDist;
  knob.style.transform = `translate(${kx}px, ${ky}px)`;

  const coords = quantizeJoystick(kx, ky, maxR);
  const wheels = xyToWheelSpeed(coords.x, coords.y);
  startSignal(wheels.left, wheels.right);
}

// Touch events
knob.addEventListener('touchstart', (e) => {
  e.preventDefault();
  joystickActive = true;
  knob.classList.add('active');
}, { passive: false });

document.addEventListener('touchmove', (e) => {
  if (!joystickActive) return;
  e.preventDefault();
  const t = e.touches[0];
  handleMove(t.clientX, t.clientY);
}, { passive: false });

document.addEventListener('touchend', (e) => {
  if (!joystickActive) return;
  joystickActive = false;
  stopSignal();
});

document.addEventListener('touchcancel', () => {
  if (!joystickActive) return;
  joystickActive = false;
  stopSignal();
});

// Mouse events (for desktop testing)
knob.addEventListener('mousedown', (e) => {
  joystickActive = true;
  knob.classList.add('active');
});

document.addEventListener('mousemove', (e) => {
  if (!joystickActive) return;
  handleMove(e.clientX, e.clientY);
});

document.addEventListener('mouseup', (e) => {
  if (!joystickActive) return;
  joystickActive = false;
  stopSignal();
});

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    stopSignal();
    camRelease();
  }
});

window.addEventListener('pagehide', () => {
  pageUnloading = true;
  stopSignal();
  if (motorSocket && motorSocket.readyState === WebSocket.OPEN)
    motorSocket.close();
});

connectMotorWebSocket();

// ── Force landscape orientation ──────────────────────────────
if (screen.orientation && screen.orientation.lock) {
  screen.orientation.lock('landscape').catch((err) => {
    console.warn('Screen orientation lock not supported:', err);
  });
}

// ── Air Quality Control ──────────────────────────────────────
// Example function to update air quality
// Call updateAirQuality('good'), updateAirQuality('moderate'), or updateAirQuality('poor')
function updateAirQuality(level) {
  const elem = document.getElementById('air-quality');
  elem.classList.remove('good', 'moderate', 'poor');
  
  switch(level) {
    case 'good':
      elem.classList.add('good');
      elem.textContent = '優良';
      break;
    case 'moderate':
      elem.classList.add('moderate');
      elem.textContent = '普通';
      break;
    case 'poor':
      elem.classList.add('poor');
      elem.textContent = '不佳';
      break;
  }
}

// ── Light Level Control ──────────────────────────────────────
// Example function to update light level
// Call updateLightLevel(500) with lux value
function updateLightLevel(lux) {
  const elem = document.getElementById('light');
  elem.textContent = `${lux} lux`;
}
