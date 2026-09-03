#!/usr/bin/env python3
"""
MPU6050 六軸慣性感測器驅動 — Raspberry Pi 5

提供兩件事：
    1. 原始 / 物理量資料：加速度（g）、角速度（°/s）、晶片溫度（°C）
    2. 互補濾波算出來的 roll / pitch 傾角，給雲台穩定用

── 接線（Pi 5 的 40 pin 排針）─────────────────────────────
    MPU6050 (GY-521)         Pi 5
    VCC  ───────────────→  pin 1   3.3V         ← 不要接 5V，見下面說明
    GND  ───────────────→  pin 9   GND
    SDA  ───────────────→  pin 3   GPIO2 (SDA1)
    SCL  ───────────────→  pin 5   GPIO3 (SCL1)
    AD0                    不接
    INT                    不接（這支驅動用輪詢，沒有用到中斷腳）
    XCL / XDA              不接（那是給 MPU6050 自己再外掛磁力計用的）

    GND 挑 pin 9 只是為了跟 Pico 佔用的 pin 6 錯開，接哪一個 GND 都一樣。

    AD0 沒接為什麼位址是 0x68：
    AD0 就是 I2C 位址的最低位元，GY-521 模組上有下拉電阻，浮接等於接地
    → 0x68。哪天要掛第二顆，把那顆的 AD0 拉到 3.3V，它就會變成 0x69。

    為什麼一定要 3.3V 不能接 5V：
    模組上 SDA/SCL 的上拉電阻是接在 VCC 那一側的。餵 5V 的話這兩條訊號線
    會被拉到 5V，超出 Pi GPIO 的耐壓。模組上那顆 LDO 只穩壓給晶片本身，
    救不了 I2C 這兩條線。

── 啟用 Pi 5 的 I2C（只需做一次）──────────────────────────
    1. sudo raspi-config → Interface Options → I2C → Yes
       （等同在 /boot/firmware/config.txt 加 dtparam=i2c_arm=on）
    2. sudo reboot
    3. sudo apt install -y i2c-tools python3-smbus2
    4. i2cdetect -y 1
       表格裡要出現 68。整片都是 -- 就是接線或供電有問題，先查那邊。

    跟 UART 不一樣，I2C 這組腳位在 Pi 5 上沒有 ttyAMA0/serial0 那種陷阱，
    i2c-1 從 Pi 3 到 Pi 5 都是 GPIO2/GPIO3。

── 單獨測試（不用啟動整台伺服器）──────────────────────────
    python mpu6050.py              # 在 Pi 上，接著真感測器
    python mpu6050.py --mock       # 在筆電上，看輸出格式跟濾波器行為

── 接進 server.py ─────────────────────────────────────────
    from mpu6050 import MPU6050

    imu = MPU6050(mock)        # 開機時建立一次，會自己起背景執行緒
    data = imu.read()          # 隨時取最新一筆，不會阻塞；沒就緒時回 None

    推給前端（read_sensors() 裡）：
        imu = app["imu"].read()
        if imu:
            payload["roll"] = round(imu["roll"], 1)
            payload["pitch"] = round(imu["pitch"], 1)

    注意 temp 是「晶片自己的溫度」，會比室溫高好幾度，不能拿來當環境溫度
    推給前端 —— 環境溫度要另外接 DHT22 之類的。

    雲台穩定（在送角度給 Pico 之前補上去）：
        pan_fix, tilt_fix = imu.stabilize()
        gimbal.aim(target_pan + pan_fix, target_tilt + tilt_fix)
"""

import argparse
import logging
import math
import os
import random
import sys
import threading
import time

log = logging.getLogger("car.imu")

# ── I2C ─────────────────────────────────────────────────────
I2C_BUS = 1          # Pi 的 GPIO2/GPIO3（pin 3/5）是 i2c-1
I2C_ADDRESS = 0x68   # AD0 浮接（模組上有下拉電阻）→ 0x68；拉到 3.3V 才是 0x69

# ── 暫存器 ──────────────────────────────────────────────────
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A   # 內建低通濾波器
REG_GYRO_CONFIG  = 0x1B   # 陀螺儀量程
REG_ACCEL_CONFIG = 0x1C   # 加速度計量程
REG_ACCEL_XOUT_H = 0x3B   # 資料起點，往後 14 bytes = 加速度 6 + 溫度 2 + 陀螺 6
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

BURST_LENGTH = 14
WHO_AM_I_EXPECTED = 0x68

# 量程 → (寫進暫存器的值, 每 1 g / 每 1 °/s 對應多少個 LSB)
ACCEL_RANGES = {2: (0x00, 16384.0), 4: (0x08, 8192.0),
                8: (0x10, 4096.0), 16: (0x18, 2048.0)}
GYRO_RANGES = {250: (0x00, 131.0), 500: (0x08, 65.5),
               1000: (0x10, 32.8), 2000: (0x18, 16.4)}

# 預設量程。車子的動作沒那麼激烈，用最小的量程解析度最好：
# ±2g 一個單位是 0.06 mg，±250°/s 一個單位是 0.008°/s。
# 之後如果讀數常常貼在 ±2.0 / ±250 不動（表示滿檔被截掉了），再往上調。
DEFAULT_ACCEL_RANGE = 2
DEFAULT_GYRO_RANGE = 250

# 晶片內建的低通濾波（DLPF_CFG，0~6）。車子在跑的時候底盤震動很兇，
# 不濾的話加速度計整片都是噪訊。3 ≈ 44Hz，對輪型車是個不錯的起點；
# 數字越大截止頻率越低（越穩但反應越鈍），越小則相反。
DLPF_CFG = 3

# 背景取樣頻率。雲台穩定要跟得上車身晃動，1Hz 的感測器推播頻率遠遠不夠；
# 而且互補濾波是靠固定的時間間隔積分陀螺儀，取樣間隔忽長忽短算出來會歪。
SAMPLE_RATE_HZ = 100

# ── 互補濾波 ────────────────────────────────────────────────
# 加速度計長期準（有重力當絕對基準）但短期被震動和車子自身加速度干擾；
# 陀螺儀短期準但積分會慢慢漂。互補濾波就是低頻信加速度計、高頻信陀螺儀。
#
# TAU 是兩者的分界時間常數：比 TAU 短的變化聽陀螺儀的，比 TAU 長的交給
# 加速度計拉回來。0.5 秒代表陀螺儀的漂移大約半秒內就會被修掉。
FILTER_TAU_S = 0.5

# 車子加速、煞車、壓到坑洞的時候，加速度計量到的不只有重力，這時候用它
# 算出來的傾角是錯的。用「合加速度離 1g 差多遠」判斷這一筆能不能信：
# 差越多越不採信，差超過這個值就完全不採信，該輪只積分陀螺儀。
ACCEL_TRUST_BAND_G = 0.15

# 靜止時陀螺儀 Z 軸仍有殘餘雜訊，直接積分的話 yaw 會一直往同一邊偏。
# 角速度小於這個值就當作沒在轉。
YAW_DEADBAND_DPS = 0.8

# 開機校正陀螺儀零點要取樣多久。這段時間車子一定要靜止不動。
CALIBRATE_S = 1.0

# 兩次取樣之間隔太久（Pi 忙著壓 JPEG、系統卡了一下）就不要把整段時間都
# 拿去積分陀螺儀，不然角度會一次跳一大格。超過就當成只過了這麼久。
MAX_DT_S = 0.2

# ── 雲台穩定 ────────────────────────────────────────────────
# 裝好之後試一次：把車頭抬起來，鏡頭應該要往下補。如果反而跟著往上跑，
# 就把對應的這個改成 True。跟 server.py 的 FACE_INVERT_* 是同樣的意思，
# 會不會反取決於 MPU6050 貼在車上的方向。
STABILIZE_INVERT_PAN = False
STABILIZE_INVERT_TILT = False


def _s16(data, index):
    """把兩個 byte 併成有號 16 位元整數（MPU6050 是高位元組在前）。"""
    value = (data[index] << 8) | data[index + 1]
    return value - 65536 if value & 0x8000 else value


class MPU6050:
    """MPU6050 驅動。背景執行緒固定頻率取樣，隨時可以拿到最新一筆。

    跟 Camera 一樣的作法：讀取跑在自己的執行緒裡，read() 只是拿快照，
    不會卡住呼叫的人。互補濾波也非得這樣做不可 —— 它要靠穩定的取樣間隔
    積分陀螺儀，如果交給一秒推播一次的 sensor_push 去呼叫，陀螺儀那一半
    等於沒作用，剩下的就只是一個很吵的加速度計。
    """

    def __init__(self, mock=False, bus=I2C_BUS, address=I2C_ADDRESS,
                 accel_range=DEFAULT_ACCEL_RANGE, gyro_range=DEFAULT_GYRO_RANGE,
                 sample_rate_hz=SAMPLE_RATE_HZ, calibrate_s=CALIBRATE_S):
        if accel_range not in ACCEL_RANGES:
            raise ValueError(f"加速度量程只能是 {sorted(ACCEL_RANGES)} 其中之一")
        if gyro_range not in GYRO_RANGES:
            raise ValueError(f"陀螺儀量程只能是 {sorted(GYRO_RANGES)} 其中之一")

        self.mock = mock
        self.bus = bus
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.sample_rate_hz = sample_rate_hz
        self.calibrate_s = calibrate_s
        self.ok = False

        self._accel_scale = ACCEL_RANGES[accel_range][1]
        self._gyro_scale = GYRO_RANGES[gyro_range][1]
        self._bias = (0.0, 0.0, 0.0)

        self._roll = None      # None = 還沒有第一筆，濾波器尚未起始
        self._pitch = 0.0
        self._yaw = 0.0

        self._smbus = None
        self._state = None
        self._state_at = 0.0
        self._next_retry = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._open()
        threading.Thread(target=self._loop, daemon=True).start()

    # ── 連線與設定 ──────────────────────────────────────────
    def _open(self):
        """開啟 I2C 並設定晶片。失敗不丟例外，只標記成離線讓外面繼續跑。"""
        if self.mock:
            self.ok = True
            log.info("IMU：模擬模式（不會讀真的感測器）")
            return

        try:
            from smbus2 import SMBus
        except ImportError:
            log.error("IMU：找不到 smbus2，請執行 "
                      "sudo apt install -y python3-smbus2")
            self._next_retry = time.monotonic() + 60
            return

        try:
            self._smbus = SMBus(self.bus)
            self._configure()
        except OSError as exc:
            # 感測器沒接好不該讓整台車掛掉，相機、驅動輪、雲台都要還能用
            self._close_bus()
            self._next_retry = time.monotonic() + 3
            log.warning("IMU：連不上 MPU6050（i2c-%d 位址 0x%02X）：%s",
                        self.bus, self.address, exc)
            return

        self.ok = True
        log.info("IMU：已就緒（i2c-%d 位址 0x%02X，±%dg / ±%d°/s，%dHz）",
                 self.bus, self.address, self.accel_range, self.gyro_range,
                 self.sample_rate_hz)

        if self.calibrate_s > 0:
            try:
                self.calibrate(self.calibrate_s)
            except OSError as exc:
                self._fail(f"校正時讀取失敗：{exc}")

    def _configure(self):
        """叫醒晶片並寫入量程、濾波、取樣率。"""
        # MPU6050 一上電是睡著的（PWR_MGMT_1 的 SLEEP 位元預設為 1），
        # 不叫醒的話讀出來全是 0。先送一次 reset，是為了不管上一支程式把它
        # 設成什麼樣，都從乾淨的預設狀態開始。
        self._write(REG_PWR_MGMT_1, 0x80)   # DEVICE_RESET
        time.sleep(0.1)

        # 0x01 = 解除睡眠 + 時脈源改用陀螺儀 X 軸的 PLL。
        # 資料手冊建議不要留在預設的內部 8MHz 震盪器，它的溫飄比較大。
        self._write(REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        who = self._read(REG_WHO_AM_I)
        if who != WHO_AM_I_EXPECTED:
            # 副廠料和 MPU6500 會回別的值，但暫存器配置一樣，照樣能用。
            # 只提醒不擋，免得買到相容晶片還在那邊查接線。
            log.warning("IMU：WHO_AM_I 回 0x%02X 而不是 0x%02X，"
                        "可能是相容晶片，先照常使用", who, WHO_AM_I_EXPECTED)

        self._write(REG_CONFIG, DLPF_CFG)

        # 晶片內部的取樣率設成我們輪詢頻率的兩倍，確保每次讀到的都是新資料。
        # 基準頻率：開了 DLPF（1~6）是 1kHz，關掉（0 或 7）才是 8kHz。
        base = 1000 if 1 <= DLPF_CFG <= 6 else 8000
        divider = round(base / max(1, self.sample_rate_hz * 2)) - 1
        self._write(REG_SMPLRT_DIV, max(0, min(255, divider)))

        self._write(REG_ACCEL_CONFIG, ACCEL_RANGES[self.accel_range][0])
        self._write(REG_GYRO_CONFIG, GYRO_RANGES[self.gyro_range][0])

    def _write(self, register, value):
        self._smbus.write_byte_data(self.address, register, value)

    def _read(self, register):
        return self._smbus.read_byte_data(self.address, register)

    def _close_bus(self):
        if self._smbus is not None:
            try:
                self._smbus.close()
            except OSError:
                pass
            self._smbus = None

    def _fail(self, message):
        """讀取出錯：標記成離線、關掉匯流排，過一下再重連。"""
        if self.ok:
            log.warning("IMU：%s", message)
        self.ok = False
        self._close_bus()
        self._next_retry = time.monotonic() + 3

    # ── 取樣 ────────────────────────────────────────────────
    def _sample(self):
        """讀一次資料並換算成物理量，回傳 (加速度 g, 角速度 °/s, 溫度 °C)。"""
        if self.mock:
            return self._mock_sample()

        # 一次把 14 個 byte 連續讀完，不要分成七次讀。分次讀的話中間感測器
        # 會更新，六個軸就不是同一個瞬間的值，算出來的姿態會歪。
        raw = self._smbus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H,
                                              BURST_LENGTH)

        accel = (_s16(raw, 0) / self._accel_scale,
                 _s16(raw, 2) / self._accel_scale,
                 _s16(raw, 4) / self._accel_scale)

        # 溫度換算公式來自資料手冊。這是晶片自己的溫度，不是室溫。
        temp = _s16(raw, 6) / 340.0 + 36.53

        gyro = (_s16(raw, 8) / self._gyro_scale - self._bias[0],
                _s16(raw, 10) / self._gyro_scale - self._bias[1],
                _s16(raw, 12) / self._gyro_scale - self._bias[2])

        return accel, gyro, temp

    def _mock_sample(self):
        """假資料：車身慢慢左右晃 + 前後點頭，同時緩緩左右轉。

        加速度和角速度都是從同一組角度推回去算的，彼此自洽，所以互補濾波
        拿這組資料跑會收斂到正確的角度 —— 沒有硬體時也能驗證濾波器寫對了。

        Z 軸給的是明顯超過 YAW_DEADBAND_DPS 的轉速，這樣前端的 yaw 讀數在
        模擬模式下才會動；只給雜訊的話會被死區濾掉，看起來像壞掉。
        """
        now = time.monotonic()
        roll = math.radians(6.0 * math.sin(now * 0.7))
        pitch = math.radians(4.0 * math.sin(now * 0.45 + 1.0))

        accel = (-math.sin(pitch) + random.gauss(0, 0.01),
                 math.sin(roll) * math.cos(pitch) + random.gauss(0, 0.01),
                 math.cos(roll) * math.cos(pitch) + random.gauss(0, 0.01))
        gyro = (6.0 * 0.7 * math.cos(now * 0.7) + random.gauss(0, 0.2),
                4.0 * 0.45 * math.cos(now * 0.45 + 1.0) + random.gauss(0, 0.2),
                8.0 * math.sin(now * 0.25) + random.gauss(0, 0.2))

        return accel, gyro, 34.0 + random.gauss(0, 0.05)

    def calibrate(self, seconds=CALIBRATE_S):
        """量陀螺儀的零點偏移。這段時間車子必須完全靜止。

        陀螺儀就算沒在轉也會輸出一個固定的偏差值，不扣掉的話積分出來的
        角度會一直往同一個方向漂。每次開機都要重新量，因為這個偏差會隨
        溫度變 —— 存起來下次直接用是不準的。

        加速度計不需要這樣校正：它有重力當絕對基準，濾波時會自己把陀螺儀
        的殘餘漂移拉回來。只有 yaw 沒有基準可拉，所以最怕零點沒校好。
        """
        if self.mock:
            return

        self._bias = (0.0, 0.0, 0.0)   # 先歸零，不然變成拿舊值再扣一次

        samples = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            _accel, gyro, _temp = self._sample()
            samples.append(gyro)
            time.sleep(1.0 / self.sample_rate_hz)

        if not samples:
            return

        bias = tuple(sum(axis) / len(samples) for axis in zip(*samples))
        spread = max(max(abs(s[i] - bias[i]) for s in samples) for i in range(3))
        self._bias = bias

        log.info("IMU：陀螺儀零點 %+.2f / %+.2f / %+.2f °/s（取樣 %d 筆）",
                 bias[0], bias[1], bias[2], len(samples))
        if spread > 2.0:
            log.warning("IMU：校正期間感測器好像在動（跳動 %.1f °/s），"
                        "零點可能不準；靜止之後重跑一次會比較好", spread)

    # ── 姿態 ────────────────────────────────────────────────
    def _update(self, accel, gyro, temp, dt, now):
        """互補濾波：更新 roll / pitch / yaw，並存下這一筆快照。"""
        ax, ay, az = accel
        gx, gy, gz = gyro

        # roll 繞 X 軸（左右翻）、pitch 繞 Y 軸（前後仰），都照右手定則。
        # 平放時加速度計讀到 (0, 0, +1g)，兩個角度都是 0。
        #
        # pitch 用 hypot(ay, az) 當分母而不是直接除 az：這樣鏡頭朝上或朝下
        # 都不會有正負號跳動，範圍完整涵蓋 -90°~+90°。
        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        magnitude = math.sqrt(ax * ax + ay * ay + az * az)

        if self._roll is None:
            # 第一筆直接拿加速度計算出來的角度當起點，省掉開機後從 0 慢慢
            # 收斂那幾秒 —— 那段期間的讀數是錯的，雲台會先歪一下才回正。
            self._roll, self._pitch = accel_roll, accel_pitch
        else:
            # 合加速度離 1g 越遠，代表這一筆混進越多車子自身的加速度，
            # 越不能拿來當傾角的基準。trust 為 0 時這一輪純靠陀螺儀撐著。
            trust = max(0.0, 1.0 - abs(magnitude - 1.0) / ACCEL_TRUST_BAND_G)
            alpha = FILTER_TAU_S / (FILTER_TAU_S + dt)
            weight = (1.0 - alpha) * trust     # 加速度計這一輪佔的比重

            self._roll = ((self._roll + gx * dt) * (1.0 - weight)
                          + accel_roll * weight)
            self._pitch = ((self._pitch + gy * dt) * (1.0 - weight)
                           + accel_pitch * weight)

        # yaw 只能積分陀螺儀。MPU6050 沒有磁力計，沒有任何絕對基準可以把它
        # 拉回來，所以一定會漂 —— 短時間（幾十秒內）可用，長時間要靠
        # reset_yaw() 重新歸零，或是改用有磁力計的 MPU9250。
        if abs(gz) > YAW_DEADBAND_DPS:
            self._yaw = (self._yaw + gz * dt + 180.0) % 360.0 - 180.0

        state = {
            "ok": True,
            "ax": ax, "ay": ay, "az": az,          # g
            "gx": gx, "gy": gy, "gz": gz,          # °/s，已扣掉零點偏移
            "accel_g": magnitude,                  # 合加速度，靜止時 ≈ 1.0
            "temp": temp,                          # °C，晶片溫度不是室溫
            "roll": self._roll,                    # 度，濾波後
            "pitch": self._pitch,
            "yaw": self._yaw,                      # 度，會漂
        }
        with self._lock:
            self._state = state
            self._state_at = now

    def _loop(self):
        """背景執行緒：固定頻率取樣、跑濾波、斷線時自動重連。"""
        period = 1.0 / self.sample_rate_hz
        next_tick = time.monotonic()
        last = None

        while not self._stop.is_set():
            if not self.ok:
                if time.monotonic() >= self._next_retry:
                    self._open()
                    last = None
                self._stop.wait(1.0)
                next_tick = time.monotonic()
                continue

            try:
                accel, gyro, temp = self._sample()
            except OSError as exc:
                self._fail(f"讀取失敗：{exc}")
                with self._lock:
                    self._state = None
                continue

            now = time.monotonic()
            dt = period if last is None else min(now - last, MAX_DT_S)
            last = now
            self._update(accel, gyro, temp, dt, now)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                # 跟不上就重新對時，不要讓落後一直累積成越追越急
                next_tick = time.monotonic()

    # ── 對外 ────────────────────────────────────────────────
    def read(self):
        """拿最新一筆的快照（dict）。感測器還沒就緒的話回傳 None。

        不會阻塞，也不會等下一次取樣 —— 拿到的是背景執行緒最後存進來的
        那一筆，額外多一個 age 欄位代表它是幾秒前量的。
        """
        with self._lock:
            state = self._state
            measured_at = self._state_at
        if state is None:
            return None
        return dict(state, age=time.monotonic() - measured_at)

    def stabilize(self):
        """回傳雲台要補多少角度才能抵銷車身晃動，(pan, tilt)，單位是度。

        座標跟 server.py 的 Gimbal 一致：正值 = 右 / 上。用法是把這個值加到
        使用者（或人臉追蹤）指定的角度上，再送給 Pico。

        兩軸雲台補得掉什麼：
            tilt ← 車身 pitch，補得掉，這項最有感（過坎時畫面不會跟著抬頭）
            pan  ← 車身 yaw，只有短時間內準，會慢慢漂，需要時呼叫 reset_yaw()
            roll   沒有對應的軸，補不掉，畫面還是會歪 —— 這是兩軸雲台的硬
                   限制，要修只能在影像端把畫面轉回來
        """
        data = self.read()
        if data is None:
            return 0.0, 0.0

        pan = -data["yaw"]
        tilt = -data["pitch"]
        if STABILIZE_INVERT_PAN:
            pan = -pan
        if STABILIZE_INVERT_TILT:
            tilt = -tilt
        return pan, tilt

    def reset_yaw(self):
        """把 yaw 歸零，當成「現在這個方向就是正前方」。

        yaw 沒有絕對基準會一直漂，所以雲台回正、切換模式這種時機點呼叫
        一次，比放著讓它累積誤差好。
        """
        with self._lock:
            self._yaw = 0.0
            if self._state is not None:
                self._state = dict(self._state, yaw=0.0)

    def close(self):
        self._stop.set()
        self._close_bus()
        self.ok = False


# ── 單獨測試 ────────────────────────────────────────────────
def check_i2c(bus):
    """I2C 沒啟用的話 /dev/i2c-N 根本不存在，先擋在這裡比較好查。"""
    path = f"/dev/i2c-{bus}"
    if os.path.exists(path):
        return None
    return (f"找不到 {path} —— I2C 還沒啟用。\n"
            f"  sudo raspi-config → Interface Options → I2C → Yes，"
            f"然後 sudo reboot")


def main():
    parser = argparse.ArgumentParser(description="MPU6050 單獨測試")
    parser.add_argument("--mock", action="store_true",
                        help="不接硬體，用假資料跑（可在筆電上看輸出）")
    parser.add_argument("--bus", type=int, default=I2C_BUS,
                        help=f"I2C 匯流排編號（預設 {I2C_BUS}）")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=I2C_ADDRESS,
                        help=f"I2C 位址（預設 0x{I2C_ADDRESS:02X}，AD0 沒接就是這個）")
    parser.add_argument("--once", action="store_true", help="印一筆完整資料就結束")
    args = parser.parse_args()

    # Windows 主控台預設是 cp950，中文會變亂碼
    for console in (sys.stdout, sys.stderr):
        if hasattr(console, "reconfigure"):
            console.reconfigure(encoding="utf-8", errors="replace")

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    if not args.mock:
        problem = check_i2c(args.bus)
        if problem:
            print(problem)
            return 1

    print(f"位址 0x{args.address:02X}（AD0 沒接 → 模組上的下拉電阻把它當成 0）")
    print("校正陀螺儀零點中，請不要動感測器…")

    imu = MPU6050(args.mock, bus=args.bus, address=args.address)

    if not imu.ok:
        print("\n感測器沒有回應。依序檢查：")
        print("  1. i2cdetect -y 1  有沒有看到 68")
        print("  2. VCC 是不是接在 3.3V（pin 1），不是 5V")
        print("  3. SDA→pin 3、SCL→pin 5 有沒有接反")
        return 1

    # 等第一筆資料進來（背景執行緒剛起來，可能還沒跑完第一輪）
    deadline = time.monotonic() + 2.0
    while imu.read() is None and time.monotonic() < deadline:
        time.sleep(0.05)

    data = imu.read()
    if data is None:
        print("讀不到資料，感測器可能在校正之後掉線了")
        return 1

    if args.once:
        print()
        print(f"  加速度   x {data['ax']:+7.3f}  y {data['ay']:+7.3f}  "
              f"z {data['az']:+7.3f}  g")
        print(f"  角速度   x {data['gx']:+7.2f}  y {data['gy']:+7.2f}  "
              f"z {data['gz']:+7.2f}  °/s")
        print(f"  傾角     roll {data['roll']:+7.2f}  pitch {data['pitch']:+7.2f}  "
              f"yaw {data['yaw']:+7.2f}  度")
        print(f"  合加速度 {data['accel_g']:.3f} g（靜止平放時應該接近 1.000）")
        print(f"  晶片溫度 {data['temp']:.1f} °C（不是室溫，會比室溫高幾度）")
        imu.close()
        return 0

    print("\n把板子傾斜看看 roll / pitch 有沒有跟著動。Ctrl-C 結束。\n")
    try:
        while True:
            data = imu.read()
            if data is None:
                print("等待感測器…                                        ",
                      end="\r", flush=True)
            else:
                print(f"roll {data['roll']:+7.2f}°  pitch {data['pitch']:+7.2f}°  "
                      f"yaw {data['yaw']:+8.2f}°  |a| {data['accel_g']:5.3f}g  "
                      f"晶片 {data['temp']:5.1f}°C ", end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        imu.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
