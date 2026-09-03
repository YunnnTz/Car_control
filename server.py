#!/usr/bin/env python3
"""
智慧車後端 — Raspberry Pi 5

一支程式同時提供三件事，全部跑在 8080 埠：

    GET  /            前端靜態檔（car_control/）
    GET  /stream      MJPEG 影像串流
    WS   /ws          馬達控制指令 + 感測器資料推播

系統分工：
    Pi 5     網頁伺服器、相機串流、驅動輪馬達、車身姿態（MPU6050）
    Pico W   雲台伺服馬達（GP10 / GP11），韌體在 pico/main.py
             兩者用 UART 直連（交叉接，TX 對 RX）：
                 Pi GPIO14 = TXD（pin 8）  ──→ Pico GP5 = RX（pin 7）
                 Pi GPIO15 = RXD（pin 10） ←── Pico GP4 = TX（pin 6）
                 Pi GND    （pin 6）       ─── Pico GND   （pin 3）
             Pico 用的是 UART1 的 GP4/GP5，不是預設的 GP0/GP1 ——
             這片 Pico 的 GP0/GP1 實測收發都不通，詳見 pico/main.py 的說明。

MPU6050 走 I2C 接在 Pi 上（GPIO2/3 = pin 3/5），驅動和接線說明在 mpu6050.py。

── 啟用 Pi 5 的 UART（只需做一次）─────────────────────────
    1. sudo raspi-config
         Interface Options → Serial Port
           "login shell over serial?"   → No     ← 一定要選 No，否則會被 getty 佔用
           "serial port hardware?"      → Yes
    2. 確認 /boot/firmware/config.txt 裡有 dtparam=uart0=on
       （Bookworm 的路徑是 /boot/firmware/，不是舊的 /boot/）
    3. sudo reboot

    注意裝置要用 /dev/ttyAMA0，不是 /dev/serial0 —— 見下方 PICO_PORT 的說明。

── 啟用 Pi 5 的 I2C（給 MPU6050，只需做一次）──────────────
    1. sudo raspi-config → Interface Options → I2C → Yes
    2. sudo reboot
    3. i2cdetect -y 1   表格裡要出現 68（AD0 沒接就是這個位址）

為什麼統一用 8080：前端 app.js:36 寫死連 ws://<host>:8080/ws，
把整包都掛在 8080 底下就完全不用改前端。開 http://<pi-ip>:8080/ 即可。

── 安裝（Raspberry Pi OS Bookworm / Pi 5）──────────────────
    sudo apt install -y python3-aiohttp python3-pil python3-opencv \
                        python3-lgpio python3-gpiozero python3-serial \
                        python3-smbus2 i2c-tools
    python server.py

    apt 找不到 python3-smbus2 的話（版本比較舊的系統會這樣）改用：
        pip install --break-system-packages smbus2
    Bookworm 起系統 Python 是 externally-managed，不加那個旗標會被擋。

    相機預設當成 USB 視訊鏡頭處理（走 V4L2/OpenCV）。
    如果改用 CSI 排線相機，加裝 python3-picamera2 並用 --camera csi 啟動。

    用 apt 而不是 pip + venv，是因為這些套件 Debian 都有現成的包，
    裝完系統全域可用，不必每次登入都重新啟用虛擬環境。

── 沒有硬體時先在筆電上測 ─────────────────────────────────
    pip install aiohttp pillow
    python server.py --mock
"""

import argparse
import asyncio
import glob
import io
import json
import logging
import random
import re
import sys
import threading
import time
from pathlib import Path

from aiohttp import web, WSMsgType

from mpu6050 import MPU6050

# ── 硬體接線設定（依你們實際接的腳位修改）──────────────────
# 這裡假設用 L298N 這類雙 H 橋驅動板，每顆馬達三條線：正轉 / 反轉 / 致能(PWM)
LEFT_FORWARD, LEFT_BACKWARD, LEFT_ENABLE = 17, 27, 12
RIGHT_FORWARD, RIGHT_BACKWARD, RIGHT_ENABLE = 22, 23, 13

# 雲台：伺服馬達接在 Pico W 的 GP10 / GP11，Pi 5 透過 UART（GPIO14/15）跟它溝通。
#
# 這裡一定要寫 /dev/ttyAMA0，不能用 /dev/serial0：
# Pi 5 上 serial0 指向 ttyAMA10，那是板子上那個獨立的三針除錯接頭，
# 不是排針的 GPIO14/15。用 serial0 的話埠打得開、寫入也不會報錯，
# 但資料全部送到除錯接頭去，Pico 一個字都收不到。
# （Pi 4 以前 serial0 才是指向排針的那組，舊教學不能照抄。）
PICO_PORT = "/dev/ttyAMA0"   # 可用 --pico 覆蓋
PICO_BAUD = 115200

PORT = 8080
STATIC_DIR = Path(__file__).parent / "car_control"

MOTOR_TIMEOUT_S = 0.3   # 看門狗：超過這麼久沒收到指令就強制停車
SENSOR_PERIOD_S = 1.0   # 感測器推播間隔
STREAM_FPS = 15
FRAME_SIZE = (640, 480)
JPEG_QUALITY = 70
CAMERA_REPORT_S = 5.0   # 每隔幾秒回報一次抓圖狀況

# 相機類型："usb" = USB 視訊鏡頭（走 V4L2/OpenCV）、"csi" = 排線相機（走 picamera2）
CAMERA_BACKEND = "usb"
USB_DEVICE = None   # None = 自動偵測；也可寫死成 0、1 之類的編號

# ── 人臉追蹤 ────────────────────────────────────────────────
FACE_DETECT_FPS = 5           # 每秒偵測幾次（Haar 很吃 CPU，不必每張都做）
FACE_DETECT_SCALE = 0.5       # 偵測前先把影格縮小，加快速度
FACE_LOST_S = 1.5             # 多久沒偵測到臉就把畫面上的框拿掉
FACE_MANUAL_SUSPEND_S = 2.0   # 手動按方向鍵之後，暫停自動追蹤幾秒

# 自動模式偵測到第一張臉之後就鎖定：停止偵測、停止巡邏、雲台停在原地。
# 用意是「掃到人就停下來」，而不是一直跟著人跑。
# 要重新開始，切回手動再切回自動即可（set_mode 會解除鎖定）。
# 改成 False 就回到原本的行為：持續追蹤，跟丟了再繼續巡邏。
AUTO_STOP_ON_FIRST_FACE = True

# 追蹤採比例控制：每次只修正誤差的一部分，會平順收斂。
# 固定步進（偏多少就轉固定幾度）在接近中心時很容易過衝、來回震盪。
# 死區是「精度」和「穩定」的取捨：太大會停在偏離中心的位置，太小則會被
# 偵測雜訊帶著一直微調。0.12 約等於殘餘誤差 3.6°，仍高於平滑後的雜訊量。
FACE_DEADZONE = 0.12       # 誤差小於畫面的這個比例就完全不動
FACE_GAIN = 0.35           # 每次修正掉多少比例的誤差；調大追得快但容易震盪
FACE_MAX_STEP_DEG = 6.0    # 單次修正的角度上限，避免偵測跳掉時甩過去
FACE_SMOOTHING = 0.6       # 偵測位置的平滑係數，0 = 完全信任新值，越大越穩
FACE_FOV_DEG = (60.0, 45.0)   # 鏡頭大約的水平 / 垂直視野角度

# 如果追蹤方向相反（鏡頭往臉的反方向跑），把對應的這個改成 True。
# 會不會反取決於伺服馬達的安裝方向，接好之後試一次就知道。
FACE_INVERT_PAN = False
FACE_INVERT_TILT = False

# 伺服馬達的安裝方向。按「右」卻往左轉的話，把對應的改成 True。
# 這是在送給 Pico 之前把角度反號，所以手動控制和人臉追蹤會一起修正 ——
# 程式內部一律用「正值 = 右 / 上」來記帳，只有輸出到硬體時才轉換。
PAN_REVERSED = True
TILT_REVERSED = True    # 實機確認：按「上」會往下轉，所以要反號

# 伺服是否持續保持扭力。
# Pico 韌體在停止動作 GIMBAL_IDLE_S 之後會切斷 PWM（原本是為了消除待機時的
# 嗡嗡聲），但雲台裝上相機有重量之後，放鬆會導致下垂 —— 追蹤就會變成
# 「下垂 → 偵測到偏移 → 彈回來」的週期性抖動。
# 開啟這個之後，Pi 會定期重送同一個角度，讓 Pico 那邊的計時器不會逾時。
GIMBAL_HOLD = True
GIMBAL_HOLD_PERIOD_S = 0.3

# ── 雲台穩定（用 MPU6050 的車身姿態反向補償）──────────────────
# 只補 tilt 這一軸。pan 要靠 yaw，而 MPU6050 沒有磁力計，yaw 純靠陀螺儀
# 積分會慢慢漂 —— 補久了畫面會自己轉離原本的方向，比不補還糟。
# roll 沒有對應的軸，兩軸雲台補不掉。
STABILIZE_TILT = True
STABILIZE_GAIN = 1.0          # 補多少比例的車身傾角；1.0 = 完全抵銷
STABILIZE_MAX_DEG = 25.0      # 補償量上限，避免讀數異常時把伺服甩到底
# 死區：IMU 靜止時仍有零點幾度的殘餘雜訊，不設死區的話伺服會一直微抖，
# 既吵又耗電。設成 1° 表示車身晃動超過 1° 才動作。
STABILIZE_DEADZONE_DEG = 1.0
# 補償要重送角度才會生效，所以送得比 GIMBAL_HOLD_PERIOD_S 密。
# 0.1 秒對上 SG90 大約 0.1 秒轉 60° 的速度，再快伺服也跟不上。
STABILIZE_PERIOD_S = 0.1

GIMBAL_STEP_DEG = 3.0    # 每收到一次按鍵指令轉幾度
GIMBAL_IDLE_S = 0.5      # 停止動作幾秒後放開訊號（消除抖動與嗡嗡聲）
PAN_RANGE = (-90.0, 90.0)    # 水平可轉範圍：180° 全幅
TILT_RANGE = (-45.0, 45.0)   # 垂直可轉範圍：90° 全幅

# ── 自動模式的巡邏掃描 ──────────────────────────────────────
# 沒偵測到人臉時，雲台依序走訪這些點來回掃描；偵測到人臉就立刻暫停。
# 刻意不走到範圍極限，留一點餘裕給追蹤時的修正。
PATROL_WAYPOINTS = (
    (-70.0, 20.0), (0.0, 20.0), (70.0, 20.0),
    (70.0, -20.0), (0.0, -20.0), (-70.0, -20.0),
)
PATROL_SPEED_DEG_S = 30.0   # 掃描移動速度
PATROL_PAUSE_S = 1.2        # 每個點停留多久，讓偵測有足夠時間
PATROL_RATE_HZ = 20.0       # 巡邏更新頻率
STATUS_PERIOD_S = 0.25      # 狀態推播給前端的頻率

# 車身姿態（MPU6050）推播頻率。刻意比 SENSOR_PERIOD_S 密：溫溼度一秒更新
# 一次看不出差別，但傾角一秒才動一次的話畫面上會一格一格跳。
IMU_PERIOD_S = 0.2

log = logging.getLogger("car")


def clamp(value, low, high):
    return max(low, min(high, value))


# ── 馬達 ────────────────────────────────────────────────────
class Motors:
    """把 -100~100 的輪速轉成 GPIO 輸出，並記錄最後一次收到指令的時間。"""

    def __init__(self, mock):
        self.mock = mock
        self.last_command = time.monotonic()
        self.left = 0
        self.right = 0

        if mock:
            log.info("馬達：模擬模式（不會真的動）")
            return

        # Pi 5 一定要用 gpiozero + lgpio，RPi.GPIO 在 Pi 5 上不能用
        from gpiozero import Motor
        self._l = Motor(forward=LEFT_FORWARD, backward=LEFT_BACKWARD,
                        enable=LEFT_ENABLE, pwm=True)
        self._r = Motor(forward=RIGHT_FORWARD, backward=RIGHT_BACKWARD,
                        enable=RIGHT_ENABLE, pwm=True)
        log.info("馬達：GPIO 已就緒")

    @property
    def moving(self):
        return self.left != 0 or self.right != 0

    def drive(self, left, right):
        left = max(-100, min(100, int(left)))
        right = max(-100, min(100, int(right)))
        self.last_command = time.monotonic()

        if (left, right) == (self.left, self.right):
            return
        self.left, self.right = left, right

        if self.mock:
            log.debug("馬達 L=%4d R=%4d", left, right)
            return
        self._l.value = left / 100.0    # gpiozero 的 Motor.value 收 -1.0 ~ 1.0
        self._r.value = right / 100.0

    def stop(self):
        self.drive(0, 0)

    def close(self):
        self.stop()
        if not self.mock:
            self._l.close()
            self._r.close()


async def watchdog(app):
    """前端斷線、Wi-Fi 掉了、或指令中斷超過 MOTOR_TIMEOUT_S 就自動停車。

    這是整支程式最重要的安全機制：前端送不出 stop 的時候，車子靠這個停下來。
    """
    motors = app["motors"]
    while True:
        await asyncio.sleep(0.05)
        if motors.moving and time.monotonic() - motors.last_command > MOTOR_TIMEOUT_S:
            log.warning("看門狗觸發：超過 %.1fs 沒收到指令，強制停車", MOTOR_TIMEOUT_S)
            motors.stop()


# ── 雲台（相機轉向）────────────────────────────────────────
class Gimbal:
    """兩顆伺服馬達：pan 負責左右、tilt 負責上下。

    前端按住方向鍵時每 120ms 送一次指令，每次轉 GIMBAL_STEP_DEG 度，
    所以按住不放大約是每秒 25 度，放開就停在當下角度。
    """

    def __init__(self, mock, port=PICO_PORT, imu=None):
        self.mock = mock
        self.port = port
        self.imu = imu          # 給雲台穩定用；None = 不補償
        self.pan = 0.0
        self.tilt = 0.0
        self.last_move = 0.0
        self._released = True
        self._serial = None
        self._next_retry = 0.0
        self._rx = b""
        self._last_sent = 0.0   # 最後一次送出的時間，保持扭力用
        self._commands = 0      # 這一輪動作收到幾筆指令，停下時一併回報
        self._seen = []         # 這一輪收到過哪些方向

        if self.stabilizing:
            log.info("雲台穩定：已啟用（MPU6050 的 pitch 反向補 tilt，"
                     "死區 %.1f°，上限 %.0f°）",
                     STABILIZE_DEADZONE_DEG, STABILIZE_MAX_DEG)
        else:
            log.info("雲台穩定：未啟用")

        if mock:
            log.info("雲台：模擬模式（不會真的動）")
            return
        self._connect()

    def _connect(self):
        try:
            import serial
        except ImportError:
            log.error("雲台：找不到 pyserial，請執行 pip install pyserial")
            self._next_retry = time.monotonic() + 60
            return

        try:
            self._serial = serial.Serial(self.port, PICO_BAUD, timeout=0)
        except Exception as exc:
            # Pico 沒插上不該讓整台車掛掉，相機和驅動輪要還能用
            self._serial = None
            self._next_retry = time.monotonic() + 3
            log.warning("雲台：連不上 Pico W（%s）：%s", self.port, exc)
            return

        log.info("雲台：已連上 Pico W（%s）", self.port)
        self._apply()

    def step(self, direction):
        if direction == "left":
            self.pan = clamp(self.pan - GIMBAL_STEP_DEG, *PAN_RANGE)
        elif direction == "right":
            self.pan = clamp(self.pan + GIMBAL_STEP_DEG, *PAN_RANGE)
        elif direction == "up":
            self.tilt = clamp(self.tilt + GIMBAL_STEP_DEG, *TILT_RANGE)
        elif direction == "down":
            self.tilt = clamp(self.tilt - GIMBAL_STEP_DEG, *TILT_RANGE)
        else:
            log.warning("未知的雲台方向：%r", direction)
            return

        self._commands += 1
        if direction not in self._seen:
            self._seen.append(direction)
        self._apply()

    def nudge(self, delta_pan, delta_tilt):
        """依角度差微調（人臉追蹤用），一次只送一筆指令。

        step() 是給按鍵用的固定步進；追蹤需要連續的角度量，而且要避免
        一次連送好幾筆造成 PWM 頻繁跳動。
        """
        if abs(delta_pan) < 0.1 and abs(delta_tilt) < 0.1:
            return
        self.pan = clamp(self.pan + delta_pan, *PAN_RANGE)
        self.tilt = clamp(self.tilt + delta_tilt, *TILT_RANGE)
        self._commands += 1
        if "追蹤" not in self._seen:
            self._seen.append("追蹤")
        self._apply()

    def aim(self, pan, tilt):
        """直接設定絕對角度（巡邏掃描用）。"""
        pan = clamp(pan, *PAN_RANGE)
        tilt = clamp(tilt, *TILT_RANGE)
        if (round(pan, 1), round(tilt, 1)) == (round(self.pan, 1), round(self.tilt, 1)):
            return
        self.pan, self.tilt = pan, tilt
        self._apply()

    def _apply(self):
        """角度變了：更新狀態並送出。"""
        self.last_move = time.monotonic()
        self._released = False
        self._send()

    @property
    def stabilizing(self):
        return STABILIZE_TILT and self.imu is not None

    def _stabilize_offset(self):
        """車身晃動要補多少度（只補 tilt）。IMU 沒就緒時回 0。"""
        if not self.stabilizing:
            return 0.0

        _pan_fix, tilt_fix = self.imu.stabilize()   # 沒資料時回 (0, 0)
        if abs(tilt_fix) < STABILIZE_DEADZONE_DEG:
            return 0.0
        return clamp(tilt_fix * STABILIZE_GAIN,
                     -STABILIZE_MAX_DEG, STABILIZE_MAX_DEG)

    def _send(self):
        """把目前角度送給 Pico W（不改動狀態，保持扭力時也會呼叫）。"""
        if self.mock or self._serial is None:
            return
        self._last_sent = time.monotonic()

        # 補償只加在輸出上，self.tilt 維持「使用者要求的角度」——
        # 這樣介面上顯示的角度才不會跟著車身晃動一直跳。
        tilt_target = clamp(self.tilt + self._stabilize_offset(), *TILT_RANGE)

        # 只在這裡依安裝方向反號，內部記帳維持「正值 = 右 / 上」
        pan = -self.pan if PAN_REVERSED else self.pan
        tilt = -tilt_target if TILT_REVERSED else tilt_target
        payload = json.dumps({"pan": round(pan, 1), "tilt": round(tilt, 1)})
        try:
            self._serial.write((payload + "\n").encode())
        except Exception as exc:
            log.warning("雲台：送指令失敗，稍後重新連線：%s", exc)
            self._close_serial()
            self._next_retry = time.monotonic() + 3

    def tick(self):
        """定期呼叫：記錄停止位置、讀 Pico 的回訊、斷線時重試連接。

        伺服待機時的抖動與嗡嗡聲由 Pico 韌體自己處理（停 0.5 秒後切斷 PWM），
        這邊不需要再送放開指令。
        """
        if not self._released and time.monotonic() - self.last_move > GIMBAL_IDLE_S:
            self._released = True
            log.info("雲台停在 pan=%.0f° tilt=%.0f°（本輪收到 %d 筆指令：%s）",
                     self.pan, self.tilt, self._commands,
                     " ".join(self._seen) or "無（不是按鍵造成的）")
            self._commands = 0
            self._seen = []

        self._drain()

        # 定期重送角度：一來避免 Pico 因逾時切斷 PWM 讓雲台下垂，
        # 二來補償量是在 _send() 裡即時算的，不重送就等於沒有穩定效果。
        # 開了穩定就要送得更密，否則補償跟不上車身晃動。
        period = STABILIZE_PERIOD_S if self.stabilizing else GIMBAL_HOLD_PERIOD_S
        if (GIMBAL_HOLD and not self.mock and self._serial is not None
                and time.monotonic() - self._last_sent >= period):
            self._send()

        if not self.mock and self._serial is None and time.monotonic() >= self._next_retry:
            self._connect()

    def _drain(self):
        """把 Pico 送回來的訊息印出來。

        開機時 Pico 會送一行 "gimbal ready"，Pi 這邊收得到就代表 RX 那條線
        （Pico GP4 → Pi GPIO15）也接對了，是很方便的接線檢查。
        """
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if not waiting:
                return
            self._rx += self._serial.read(waiting)
        except Exception:
            self._close_serial()
            self._next_retry = time.monotonic() + 3
            return

        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            text = line.decode("utf-8", "replace").strip()
            if text:
                log.info("Pico：%s", text)

        if len(self._rx) > 256:
            self._rx = b""

    def _close_serial(self):
        if self._serial is None:
            return
        try:
            self._serial.close()
        except Exception:
            pass
        self._serial = None

    def close(self):
        self._close_serial()


async def gimbal_idle(app):
    gimbal = app["gimbal"]
    while True:
        # 跑得比 STABILIZE_PERIOD_S 快一倍，補償才會穩定地每 0.1 秒送出去；
        # 兩者一樣快的話會被排程抖動吃掉，變成兩次才送一次
        await asyncio.sleep(0.05)
        gimbal.tick()


# ── 人臉追蹤 ────────────────────────────────────────────────
def find_cascade(cv2, filename):
    """找出 Haar 模型檔的位置。

    pip 裝的 opencv-python 有 cv2.data 可以直接問，但 apt 裝的 python3-opencv
    沒有這個屬性，模型是由 opencv-data 套件放到 /usr/share/opencv4/ 底下。
    """
    bases = []
    data = getattr(cv2, "data", None)
    if data is not None:
        bases.append(data.haarcascades)
    bases += [
        "/usr/share/opencv4/haarcascades",        # Debian / Raspberry Pi OS
        "/usr/share/opencv/haarcascades",         # 較舊的 Debian
        "/usr/local/share/opencv4/haarcascades",  # 自行編譯安裝
    ]

    for base in bases:
        candidate = Path(base) / filename
        if candidate.exists():
            return candidate
    return None


class FaceTracker:
    """在相機影格上找人臉，驅動雲台把臉維持在畫面中央。

    掛在 Camera 的抓圖迴圈上當回呼，不另外開相機 —— 同一個 V4L2 裝置
    沒辦法被兩個程序同時開啟。
    """

    def __init__(self, gimbal, cv2):
        self.gimbal = gimbal
        self.enabled = False    # 是否執行偵測並在畫面上標註
        self.tracking = False   # 是否連帶驅動雲台（手動模式只標註不追蹤）
        self.locked = False     # 自動模式已鎖定目標，偵測停止
        self.suspend_until = 0.0

        self._cv2 = cv2
        self._last_detect = 0.0
        self._last_seen = 0.0
        self._box = None
        self._smoothed = None   # 平滑後的臉部中心，用來壓掉偵測雜訊

        path = find_cascade(cv2, "haarcascade_frontalface_default.xml")
        if path is None:
            raise RuntimeError("找不到 Haar 人臉模型，請執行 sudo apt install opencv-data")

        self._cascade = cv2.CascadeClassifier(str(path))
        if self._cascade.empty():
            raise RuntimeError(f"載入不了人臉模型：{path}")
        log.info("人臉追蹤：使用模型 %s", path)

    @property
    def has_face(self):
        """目前畫面上是否有臉（巡邏要據此決定暫不暫停）。"""
        return (self._box is not None
                and time.monotonic() - self._last_seen <= FACE_LOST_S)

    def __call__(self, frame):
        """Camera 每抓到一張影格就會呼叫這裡。"""
        if self.locked:
            # 已鎖定：不再跑偵測（Haar 完全不執行，CPU 也省下來），但把最後
            # 那個框留在畫面上 —— 直接讓框消失的話，使用者分不出是
            # 「鎖定了」還是「跟丟了」。
            self._draw(frame)
            return frame

        if not self.enabled:
            self._box = None
            self._smoothed = None
            return frame

        now = time.monotonic()
        if now - self._last_detect >= 1 / FACE_DETECT_FPS:
            self._last_detect = now
            self._detect(frame, now)

        self._draw(frame)
        return frame

    def _detect(self, frame, now):
        cv2 = self._cv2
        height, width = frame.shape[:2]

        small = cv2.resize(frame, None,
                           fx=FACE_DETECT_SCALE, fy=FACE_DETECT_SCALE)
        gray = cv2.equalizeHist(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY))
        faces = self._cascade.detectMultiScale(gray, 1.2, 5, minSize=(30, 30))

        if len(faces) == 0:
            if now - self._last_seen > FACE_LOST_S:
                self._box = None
                self._smoothed = None
            return

        # 取最大的那張臉，通常就是離鏡頭最近的人
        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        scale = 1 / FACE_DETECT_SCALE
        self._box = (int(x * scale), int(y * scale),
                     int(w * scale), int(h * scale))
        self._last_seen = now

        # 自動模式鎖定第一張臉。放在驅動雲台之前 —— 一旦鎖定就不該再轉，
        # 畫面要停在「發現目標的那一刻」，所以連這一次的修正都不做。
        # self.tracking 只有自動模式才是 True，手動模式的偵測不受影響。
        if AUTO_STOP_ON_FIRST_FACE and self.tracking:
            self.locked = True
            log.info("人臉追蹤：鎖定目標，停止偵測與巡邏"
                     "（切回手動再切回自動可重新開始）")
            return

        # 平滑：Haar 的偵測框每張影格都在跳，直接拿來驅動馬達就是在放大雜訊
        bx, by, bw, bh = self._box
        cx, cy = bx + bw / 2, by + bh / 2
        if self._smoothed is None:
            self._smoothed = (cx, cy)
        else:
            a = FACE_SMOOTHING
            self._smoothed = (a * self._smoothed[0] + (1 - a) * cx,
                              a * self._smoothed[1] + (1 - a) * cy)

        # 手動模式只標註不轉動；自動模式才驅動雲台
        if not self.tracking or now < self.suspend_until:
            return

        sx, sy = self._smoothed
        dpan, dtilt = self._correction((sx / width - 0.5) * 2,
                                       (sy / height - 0.5) * 2)
        self.gimbal.nudge(dpan, dtilt)

    @staticmethod
    def _correction(dx, dy):
        """畫面上的偏移量（-1~1）→ 雲台該轉幾度。

        先把偏移量換算成實際的角度誤差（用鏡頭視野角），再乘上增益只修正
        其中一部分。這樣每次都往正確方向收斂一點，不會衝過頭。
        """
        def axis(offset, fov, invert):
            if abs(offset) < FACE_DEADZONE:
                return 0.0
            degrees = offset * (fov / 2) * FACE_GAIN
            degrees = clamp(degrees, -FACE_MAX_STEP_DEG, FACE_MAX_STEP_DEG)
            return -degrees if invert else degrees

        return (axis(dx, FACE_FOV_DEG[0], FACE_INVERT_PAN),
                axis(-dy, FACE_FOV_DEG[1], FACE_INVERT_TILT))  # y 軸往下遞增

    def _draw(self, frame):
        if self._box is None:
            return
        x, y, w, h = self._box
        self._cv2.rectangle(frame, (x, y), (x + w, y + h), (80, 220, 120), 2)


# ── 巡邏掃描 ────────────────────────────────────────────────
class Patrol:
    """自動模式下的循環掃描：依序走訪各個定點，偵測到人臉就暫停。

    走訪之間是平滑移動而不是瞬間跳過去 —— 一方面畫面比較好看，
    另一方面移動中的影像太模糊，人臉偵測會失效。
    """

    def __init__(self, gimbal):
        self.gimbal = gimbal
        self.active = False
        self._index = 0
        self._resume_at = 0.0
        self._last_tick = time.monotonic()

    def reset(self):
        self.active = False
        self._index = 0
        self._resume_at = 0.0

    def tick(self, mode, has_face):
        """由巡邏工作定期呼叫；回傳目前是否正在掃描。"""
        now = time.monotonic()
        elapsed = now - self._last_tick
        self._last_tick = now

        # 只有自動模式、而且畫面上沒有人臉時才掃描
        if mode != "auto" or has_face:
            if self.active:
                log.info("巡邏：暫停（%s）",
                         "偵測到人臉" if has_face else "離開自動模式")
            self.active = False
            return False

        if not self.active:
            self.active = True
            self._resume_at = 0.0
            log.info("巡邏：開始掃描")

        if now < self._resume_at:
            return True   # 停在定點上讓偵測有時間

        target_pan, target_tilt = PATROL_WAYPOINTS[self._index]
        step = PATROL_SPEED_DEG_S * min(elapsed, 0.2)

        pan, arrived_pan = self._approach(self.gimbal.pan, target_pan, step)
        tilt, arrived_tilt = self._approach(self.gimbal.tilt, target_tilt, step)
        self.gimbal.aim(pan, tilt)

        if arrived_pan and arrived_tilt:
            self._index = (self._index + 1) % len(PATROL_WAYPOINTS)
            self._resume_at = now + PATROL_PAUSE_S
        return True

    @staticmethod
    def _approach(current, target, step):
        """朝目標移動一步，回傳 (新位置, 是否已抵達)。"""
        gap = target - current
        if abs(gap) <= step:
            return target, True
        return current + (step if gap > 0 else -step), False


async def patrol_loop(app):
    patrol, tracker = app["patrol"], app["tracker"]
    while True:
        await asyncio.sleep(1 / PATROL_RATE_HZ)
        # locked 也要算進去：鎖定之後偵測就停了，has_face 會在 FACE_LOST_S
        # 之後自己變回 False，只看它的話巡邏會誤以為跟丟了而重新開始掃描。
        hold = tracker is not None and (tracker.has_face or tracker.locked)
        patrol.tick(app["mode"], hold)


# ── 感測器 ──────────────────────────────────────────────────
def read_sensors(mock):
    """回傳要推給前端的感測器資料。

    接上真感測器後把 mock 分支換掉即可，常見組合：
      溫濕度 DHT22  → pip install adafruit-circuitpython-dht
      光照   BH1750 → pip install smbus2（走 I2C）
      空品   MQ-135 → 類比訊號，Pi 沒有 ADC，要多接一顆 ADS1115
    """
    if mock:
        return {
            "type": "sensor",
            "temp": round(random.uniform(22, 28), 1),
            "humi": round(random.uniform(45, 70)),
            "lux": round(random.uniform(200, 1200)),
            "air": random.choice(["good", "good", "moderate", "poor"]),
        }

    raise NotImplementedError("接上感測器後在這裡實作")


def set_mode(app, mode):
    """切換手動 / 自動模式，並連帶調整偵測與追蹤的開關。"""
    if mode not in ("manual", "auto"):
        log.warning("未知的模式：%r", mode)
        return

    app["mode"] = mode
    tracker = app["tracker"]

    if tracker is not None:
        # 每次切換模式都解除鎖定，這是唯一的重新武裝方式：
        # 自動 → 手動 → 自動，就會重新開始巡邏找人
        tracker.locked = False

        if mode == "auto":
            # 自動模式一律開啟偵測並追蹤
            tracker.enabled = True
            tracker.tracking = True
        else:
            # 手動模式只保留使用者自己開的偵測，且不驅動雲台
            tracker.tracking = False
            tracker.enabled = app["detect_manual"]

    if mode == "manual":
        app["patrol"].reset()

    log.info("模式：%s", "自動（巡邏 + 追蹤）" if mode == "auto" else "手動")


async def status_push(app):
    """定期把模式、偵測狀態、雲台角度推給前端。"""
    tracker = app["tracker"]
    while True:
        await asyncio.sleep(STATUS_PERIOD_S)
        if not app["clients"]:
            continue
        gimbal = app["gimbal"]
        payload = json.dumps({
            "type": "status",
            "mode": app["mode"],
            # 鎖定之後 Haar 不再執行，所以「偵測中」要跟著變成 false
            "detecting": bool(tracker and tracker.enabled and not tracker.locked),
            "face": bool(tracker and tracker.has_face),
            "locked": bool(tracker and tracker.locked),
            "patrol": app["patrol"].active,
            "pan": round(gimbal.pan),
            "tilt": round(gimbal.tilt),
        })
        for ws in list(app["clients"]):
            try:
                await ws.send_str(payload)
            except (ConnectionResetError, RuntimeError):
                app["clients"].discard(ws)


async def sensor_push(app):
    """定期把感測器資料廣播給所有連線中的前端。"""
    while True:
        await asyncio.sleep(SENSOR_PERIOD_S)
        if not app["clients"]:
            continue
        try:
            payload = json.dumps(read_sensors(app["mock"]))
        except NotImplementedError:
            continue
        for ws in list(app["clients"]):
            try:
                await ws.send_str(payload)
            except (ConnectionResetError, RuntimeError):
                app["clients"].discard(ws)


async def imu_push(app):
    """定期把車身姿態（MPU6050）廣播給所有連線中的前端。

    走自己的訊息型別而不是併進 sensor_push：一來姿態要更新得比環境感測器
    快，二來 read_sensors() 是別人負責的區塊，不去動它比較乾淨。

    感測器沒接好時照樣推 ok=False，前端才知道要把讀數變灰，而不是停在
    最後一個值讓人以為車子還在那個角度。
    """
    imu = app["imu"]
    while True:
        await asyncio.sleep(IMU_PERIOD_S)
        if not app["clients"]:
            continue

        data = imu.read()
        if data is None:
            payload = json.dumps({"type": "imu", "ok": False})
        else:
            payload = json.dumps({
                "type": "imu",
                "ok": True,
                "roll": round(data["roll"], 1),
                "pitch": round(data["pitch"], 1),
                "yaw": round(data["yaw"], 1),
                "accel": round(data["accel_g"], 2),
            })

        for ws in list(app["clients"]):
            try:
                await ws.send_str(payload)
            except (ConnectionResetError, RuntimeError):
                app["clients"].discard(ws)


# ── 攝影機 ──────────────────────────────────────────────────
class Camera:
    """背景執行緒持續抓圖，把最新一張 JPEG 存起來給所有觀看者共用。"""

    def __init__(self, mock, backend=CAMERA_BACKEND, device=USB_DEVICE):
        self.mock = mock
        self.backend = backend
        self.device = device
        self.latest = None
        self.on_frame = None   # 由外部掛上人臉追蹤，收到未壓縮影格
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._cam = None
        self._cv2 = None

        if not mock:
            import cv2
            self._cv2 = cv2   # CSI 模式也要用它編 JPEG
            if backend == "usb":
                self._open_usb()
            else:
                self._open_csi()

        threading.Thread(target=self._loop, daemon=True).start()

    def _open_usb(self):
        """USB 視訊鏡頭走 V4L2（OpenCV），不要用 picamera2。

        picamera2 是給 CSI 排線相機用的。USB 鏡頭原生輸出通常是 YUYV，
        picamera2 內建的 JPEG 編碼器不認得這個格式，會直接噴 KeyError。
        """
        import cv2

        self._cv2 = cv2

        if self.device is None:
            candidates = self._candidate_devices()
            log.info("攝影機：自動偵測，候選裝置 %s",
                     ", ".join(f"/dev/video{d}" for d in candidates) or "（無）")
        else:
            candidates = [self.device]

        for device in candidates:
            # 先試 MJPG（鏡頭自己壓縮，USB 頻寬省很多），不行再退回預設格式。
            # 有些便宜的 USB 鏡頭設了 MJPG 之後就再也讀不出東西，所以要有退路。
            for mjpg in (True, False):
                cap = self._try_open(cv2, device, mjpg)
                if cap is not None:
                    self._cam = cap
                    self.device = device
                    return

        raise RuntimeError(
            "找不到能抓到影格的 USB 鏡頭。用 v4l2-ctl --list-devices 看一下，"
            "再用 --video N 指定節點編號"
        )

    @staticmethod
    def _candidate_devices():
        """列出可能是相機的 /dev/videoN。

        Pi 5 上 pispbe（影像處理器）和 rpi-hevc-dec（解碼器）也會各佔掉一堆
        video 節點，要先用 sysfs 的裝置名稱排掉，不然光是逐一嘗試就要等很久。
        """
        skip = ("pispbe", "rpi-hevc", "rpi-h264", "bcm2835-codec")
        found = []

        for path in glob.glob("/dev/video*"):
            digits = re.sub(r"\D", "", path)
            if not digits:
                continue
            index = int(digits)
            try:
                with open(f"/sys/class/video4linux/video{index}/name") as handle:
                    name = handle.read().strip()
            except OSError:
                name = ""
            if any(name.startswith(prefix) for prefix in skip):
                continue
            found.append(index)

        return sorted(found)

    def _try_open(self, cv2, device, mjpg):
        """開啟指定裝置並實際抓一張驗證；失敗回傳 None。"""
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None

        if mjpg:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 只留最新一張，避免延遲越積越久

        # 剛開啟時前幾張常常會失敗，多試幾次再判定
        for _ in range(6):
            ok, _frame = cap.read()
            if ok:
                code = int(cap.get(cv2.CAP_PROP_FOURCC))
                fourcc = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
                log.info("攝影機：USB 鏡頭已啟動（/dev/video%s，%dx%d，格式 %s）",
                         device,
                         int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                         int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                         fourcc.strip())
                return cap
            time.sleep(0.05)

        cap.release()
        return None

    def _open_csi(self):
        """CSI 排線相機走 picamera2（Pi 5 上舊的 raspistill/raspivid 已被移除）。"""
        from picamera2 import Picamera2

        self._cam = Picamera2()
        # 一定要指定 format：picamera2 的 "RGB888" 在記憶體裡其實是 BGR 排列，
        # 這樣它內建的 JPEG 編碼器才認得
        self._cam.configure(
            self._cam.create_video_configuration(
                main={"size": FRAME_SIZE, "format": "RGB888"}
            )
        )
        self._cam.start()
        log.info("攝影機：CSI 相機已啟動（picamera2）")

    def _loop(self):
        interval = 1 / STREAM_FPS
        grabbed = missed = 0
        last_report = time.monotonic()

        while not self._stop.is_set():
            try:
                frame = self._mock_frame() if self.mock else self._real_frame()
                if frame:
                    grabbed += 1
                    with self._lock:
                        self.latest = frame
                else:
                    missed += 1
            except Exception:
                missed += 1
                log.exception("抓圖失敗")
                time.sleep(1)

            # 定期回報，不然抓不到圖的時候整支程式會安靜到看不出問題在哪
            now = time.monotonic()
            if now - last_report >= CAMERA_REPORT_S:
                if grabbed:
                    log.info("攝影機：過去 %.0f 秒抓到 %d 張（失敗 %d）",
                             CAMERA_REPORT_S, grabbed, missed)
                else:
                    log.error("攝影機：過去 %.0f 秒一張都沒抓到（失敗 %d 次）"
                              "— 畫面不會有影像", CAMERA_REPORT_S, missed)
                grabbed = missed = 0
                last_report = now

            time.sleep(interval)

    def _real_frame(self):
        raw = self._grab_raw()
        if raw is None:
            return None

        # 人臉追蹤掛在這裡：拿到的是未壓縮影格，偵測完可以順便把框畫上去
        if self.on_frame is not None:
            raw = self.on_frame(raw)

        ok, jpeg = self._cv2.imencode(
            ".jpg", raw, [self._cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        )
        return jpeg.tobytes() if ok else None

    def _grab_raw(self):
        """抓一張未壓縮影格（BGR 排列的 numpy 陣列）。"""
        if self.backend == "usb":
            ok, frame = self._cam.read()
            return frame if ok else None
        return self._cam.capture_array()

    def _mock_frame(self):
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            return None
        img = Image.new("RGB", FRAME_SIZE, (14, 32, 16))
        d = ImageDraw.Draw(img)
        d.text((20, 20), "MOCK CAMERA", fill=(120, 220, 140))
        d.text((20, 40), time.strftime("%H:%M:%S"), fill=(120, 220, 140))
        # 畫一條會動的直線，方便一眼看出畫面確實有在更新
        x = int(time.time() * 120) % FRAME_SIZE[0]
        d.line([(x, 0), (x, FRAME_SIZE[1])], fill=(60, 120, 80), width=3)
        buf = io.BytesIO()
        img.save(buf, format="jpeg", quality=70)
        return buf.getvalue()

    def frame(self):
        with self._lock:
            return self.latest

    def close(self):
        self._stop.set()
        if self._cam is None:
            return
        if self.backend == "usb":
            self._cam.release()   # OpenCV 是 release()，不是 stop()
        else:
            self._cam.stop()


# ── HTTP / WebSocket ────────────────────────────────────────
@web.middleware
async def no_cache(request, handler):
    """要求瀏覽器每次都重新驗證前端檔案。

    改了 index.html / app.js / style.css 之後，瀏覽器常常還是拿舊的快取，
    看起來就像「程式沒生效」。這是區網 demo，省下的頻寬遠不如少踩這個坑值得。
    串流和 WebSocket 的標頭已經送出去了，不能再改，所以要跳過。
    """
    response = await handler(request)
    if not response.prepared:
        response.headers.setdefault("Cache-Control", "no-cache, must-revalidate")
    return response


async def index(request):
    return web.FileResponse(STATIC_DIR / "index.html")


async def stream(request):
    camera = request.app["camera"]
    resp = web.StreamResponse(headers={
        "Content-Type": "multipart/x-mixed-replace; boundary=frame",
        "Cache-Control": "no-cache, private",
    })
    await resp.prepare(request)
    log.info("串流開始 %s", request.remote)

    sent = 0
    warned = False
    started = time.monotonic()
    try:
        while True:
            frame = camera.frame()
            if frame:
                await resp.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                    + frame + b"\r\n"
                )
                sent += 1
            elif not warned and time.monotonic() - started > 2:
                warned = True
                log.warning("串流：瀏覽器連上了，但相機還沒有任何影格可以送")
            await asyncio.sleep(1 / STREAM_FPS)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        log.info("串流結束 %s（共送出 %d 張）", request.remote, sent)
    return resp


async def websocket(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    motors = request.app["motors"]
    gimbal = request.app["gimbal"]
    tracker = request.app["tracker"]
    request.app["clients"].add(ws)
    log.info("前端已連線 %s（目前 %d 個）", request.remote, len(request.app["clients"]))

    try:
        async for msg in ws:
            if msg.type is not WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                log.warning("收到非 JSON：%r", msg.data[:80])
                continue
            kind = data.get("type")
            if kind == "motor":
                motors.drive(data.get("left", 0), data.get("right", 0))
            elif kind == "camera":
                # 自動模式由巡邏和追蹤接管，方向鍵不生效
                if request.app["mode"] == "auto":
                    continue
                if tracker is not None:
                    # 手動操作時讓自動追蹤先退開，不然兩邊會互相拉扯
                    tracker.suspend_until = time.monotonic() + FACE_MANUAL_SUSPEND_S
                gimbal.step(data.get("direction"))
            elif kind == "mode":
                set_mode(request.app, data.get("mode"))
            elif kind == "detect":
                # 手動模式下由使用者決定要不要偵測；自動模式一律開著
                request.app["detect_manual"] = bool(data.get("enabled"))
                if tracker is None:
                    log.warning("人臉偵測：無法啟用（模擬模式或 OpenCV 沒裝）")
                elif request.app["mode"] == "manual":
                    tracker.enabled = request.app["detect_manual"]
                    log.info("人臉偵測：%s", "開啟" if tracker.enabled else "關閉")
    finally:
        request.app["clients"].discard(ws)
        motors.stop()   # 任何一個前端斷線就先停車，安全優先
        log.info("前端已斷線 %s，已停車", request.remote)
    return ws


async def on_startup(app):
    app["tasks"] = [
        asyncio.create_task(watchdog(app)),
        asyncio.create_task(sensor_push(app)),
        asyncio.create_task(gimbal_idle(app)),
        asyncio.create_task(patrol_loop(app)),
        asyncio.create_task(status_push(app)),
        asyncio.create_task(imu_push(app)),
    ]


async def on_cleanup(app):
    for task in app["tasks"]:
        task.cancel()
    app["motors"].close()
    app["gimbal"].close()
    app["camera"].close()
    app["imu"].close()
    log.info("已關閉，馬達停止")


def build_tracker(app, mock):
    """建立人臉追蹤器並掛到相機的抓圖迴圈上。

    OpenCV 沒裝或模型載入失敗都只記錄警告，其餘功能照常運作。
    """
    try:
        import cv2
    except ImportError:
        log.warning("人臉追蹤：沒裝 OpenCV，此功能停用")
        return None

    try:
        tracker = FaceTracker(app["gimbal"], cv2)
    except Exception as exc:
        # 追蹤是加分功能，出什麼事都不該讓相機和馬達跟著停擺
        log.warning("人臉追蹤：初始化失敗，此功能停用（%s）", exc)
        return None

    if not mock:
        app["camera"].on_frame = tracker
        log.info("人臉追蹤：已就緒（預設關閉，從介面開啟）")
    return tracker


def main():
    parser = argparse.ArgumentParser(description="智慧車後端")
    parser.add_argument("--mock", action="store_true",
                        help="不使用真實硬體，用假資料跑（可在筆電上測前端）")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--pico", default=PICO_PORT,
                        help=f"Pico W 的 UART 裝置（預設 {PICO_PORT}，也可指定 /dev/ttyAMA0）")
    parser.add_argument("--camera", choices=("usb", "csi"), default=CAMERA_BACKEND,
                        help=f"相機類型（預設 {CAMERA_BACKEND}）")
    parser.add_argument("--video", type=int, default=USB_DEVICE,
                        help=f"USB 鏡頭的裝置編號，對應 /dev/videoN（預設 {USB_DEVICE}）")
    args = parser.parse_args()

    # Windows 主控台預設是 cp950，中文 log 會變亂碼
    for console in (sys.stdout, sys.stderr):
        if hasattr(console, "reconfigure"):
            console.reconfigure(encoding="utf-8", errors="replace")

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    app = web.Application(middlewares=[no_cache])
    app["mock"] = args.mock
    app["clients"] = set()
    app["motors"] = Motors(args.mock)
    # IMU 要先於雲台建立：雲台穩定需要拿它的姿態
    app["imu"] = MPU6050(args.mock)   # 開機會花約一秒校正陀螺儀零點，車要靜止
    app["gimbal"] = Gimbal(args.mock, args.pico, app["imu"])
    app["camera"] = Camera(args.mock, args.camera, args.video)
    app["tracker"] = build_tracker(app, args.mock)
    app["patrol"] = Patrol(app["gimbal"])
    app["mode"] = "manual"        # 開啟網頁時預設為手動模式
    app["detect_manual"] = False  # 手動模式下使用者是否開了人臉偵測

    app.router.add_get("/", index)
    app.router.add_get("/stream", stream)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/", STATIC_DIR)   # 這行必須放最後

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    log.info("啟動：http://0.0.0.0:%d/%s", args.port,
             "  (模擬模式)" if args.mock else "")
    web.run_app(app, host="0.0.0.0", port=args.port, print=None)


if __name__ == "__main__":
    main()
