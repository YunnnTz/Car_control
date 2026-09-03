"""
智慧車雲台韌體 — Raspberry Pi Pico W (MicroPython)

── 接線 ────────────────────────────────────────────────────
伺服馬達：
    pan  訊號線（橘）→ GP10（實體 pin 14）
    tilt 訊號線（橘）→ GP11（實體 pin 15）
    電源線（紅）     → 獨立 5V 電源，不要接 Pico 的 VBUS 或 3V3
    接地線（棕/黑）  → 和 Pico 的 GND 共接

UART（接 Pi 5）—— 目前用 UART1，見下方 UART_TX_GPIO 的說明：
    Pico GP4 = TX（實體 pin 6） ──→ Pi 5 GPIO15 = RXD（實體 pin 10）
    Pico GP5 = RX（實體 pin 7） ←── Pi 5 GPIO14 = TXD（實體 pin 8）
    Pico GND     （實體 pin 3） ─── Pi 5 GND     （實體 pin 6）

    TX 要接對方的 RX，是交叉接的。兩邊都是 3.3V 邏輯，不需要準位轉換。

GP10 / GP11 是 RP2040 同一組 PWM slice(5) 的 A、B 通道，會共用頻率；
伺服本來就都跑 50Hz，所以沒有衝突。

註：本專案這片 Pico 的 GP0/GP1（UART0）實測無法收發 —— 韌體有跑、腳位電氣
也連通，但資料進不去也出不來。改用 UART1 的 GP4/GP5 之後正常。

── 通訊 ────────────────────────────────────────────────────
指令從 UART 或 USB 任一邊進來都接受，一行一筆 JSON：

    {"pan": 18.0, "tilt": -6.0}

同時支援兩個來源的用意是：UART 接線很容易出問題，而 USB 線本來就要接著供電，
所以 USB 這條路等於是免費的備援。Pi 端用 --pico /dev/ttyACM0 就能改走 USB。

送的是「絕對角度」而不是增量，中間掉幾筆也不會累積誤差。
開機時會往 UART 送一行 "gimbal ready"，Pi 收得到就代表 RX 那條線也接對了。

── 安裝 ────────────────────────────────────────────────────
用 Thonny 把這個檔案存到 Pico 上、檔名必須是 main.py，插電就會自動執行。
"""

from machine import Pin, PWM, UART
import json
import select
import sys
import time

PAN_GPIO, TILT_GPIO = 10, 11
SERVO_FREQ_HZ = 50
MIN_PULSE_US, MAX_PULSE_US = 500, 2400  # SG90 規格；轉不到底就調這兩個值
IDLE_RELEASE_MS = 500                   # 停止動作多久之後切斷訊號

# RP2040 的 UART 可以映射到多組腳位。如果目前這組不通（訊號進不去也出不來），
# 換一組是排除「該腳位損壞或未正確綁定」最快的方法：
#
#   UART0 → GP0/GP1（pin 1/2）、GP12/GP13（pin 16/17）、GP16/GP17（pin 21/22）
#   UART1 → GP4/GP5（pin 6/7）、GP8/GP9（pin 11/12）
#
# 換的時候記得 Pi 那兩條線也要跟著移到對應的針腳上。
# GP10/GP11 給伺服用，不要選到。
UART_ID = 1
UART_TX_GPIO, UART_RX_GPIO = 4, 5     # 原本是 UART_ID=0 搭配 0, 1
UART_BAUD = 115200
MAX_BUFFER = 256                        # 收到沒有換行的垃圾時的丟棄門檻

HEARTBEAT_MS = 1000                     # 待機時每隔多久閃一下
LED_FLASH_MS = 80                       # 每次閃多久

# 除錯用：把 UART 收到的原始位元組印到 USB（在 Pi 上 cat /dev/ttyACM0 可看）。
# 平時要關著 —— 保持扭力會讓 Pi 每 0.3 秒送一次，開著就會不停 print，
# USB 接著但沒人讀時緩衝區塞滿可能讓韌體卡住。
UART_ECHO_TO_USB = False

pan = PWM(Pin(PAN_GPIO))
tilt = PWM(Pin(TILT_GPIO))
for servo in (pan, tilt):
    servo.freq(SERVO_FREQ_HZ)

uart = UART(UART_ID, baudrate=UART_BAUD,
            tx=Pin(UART_TX_GPIO), rx=Pin(UART_RX_GPIO))

# 板載 LED：Pico W 掛在無線晶片上（用 "LED" 這個別名），一般 Pico 則是 GP25。
# Pico 沒有電源指示燈，這顆燈是唯一能從外觀判斷韌體有沒有在跑的方式。
try:
    led = Pin("LED", Pin.OUT)
except (TypeError, ValueError):
    led = Pin(25, Pin.OUT)


def angle_to_ns(angle):
    """把 -90~90 度換算成脈寬（奈秒）。"""
    if angle < -90:
        angle = -90
    elif angle > 90:
        angle = 90
    us = MIN_PULSE_US + (angle + 90) * (MAX_PULSE_US - MIN_PULSE_US) / 180
    return int(us * 1000)


def apply(pan_deg, tilt_deg):
    pan.duty_ns(angle_to_ns(pan_deg))
    tilt.duty_ns(angle_to_ns(tilt_deg))


def release():
    """切斷 PWM 訊號，消除伺服馬達待機時的抖動與嗡嗡聲。

    放開之後靠齒輪本身的阻力就足以維持角度。
    """
    pan.duty_ns(0)
    tilt.duty_ns(0)


# 兩個來源共用同一個行緩衝區：UART 和 USB 進來的指令格式完全一樣
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)


def read_usb():
    """把 USB 序列上已經到達的位元組讀出來。

    不用 readline()：poll 只保證「有資料」，不保證整行都到了，
    readline() 會把整個迴圈卡住。
    """
    chunk = b""
    for _ in range(128):
        if not poller.poll(0):
            break
        char = sys.stdin.read(1)
        if not char:
            break
        chunk += char.encode()
    return chunk


def announce():
    """往兩個通道都回報一次，對面不管接哪邊都收得到。"""
    uart.write(b"gimbal ready\n")
    print("gimbal ready")


buffer = b""
last_move = time.ticks_ms()
released = True

# 開機時連閃三下，代表韌體已經開始執行
for _ in range(3):
    led.value(1)
    time.sleep_ms(120)
    led.value(0)
    time.sleep_ms(120)

last_blink = time.ticks_ms()
led_on = False

announce()

while True:
    # USB 和 UART 都收，哪邊有資料就用哪邊
    incoming = read_usb()
    if uart.any():
        from_uart = uart.read() or b""
        if from_uart and UART_ECHO_TO_USB:
            print("uart<", from_uart)
        incoming += from_uart

    if incoming:
        buffer += incoming

        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)

            # 讓對面隨時能確認韌體還活著，不必等 Pico 重開機那唯一一次
            if line.strip() == b"ping":
                announce()
                continue

            try:
                cmd = json.loads(line)
                apply(cmd.get("pan", 0), cmd.get("tilt", 0))
                last_move = time.ticks_ms()
                released = False
            except (ValueError, TypeError):
                pass  # 收到壞掉的資料就丟掉，不要讓韌體整個掛掉

        if len(buffer) > MAX_BUFFER:
            buffer = b""

    now = time.ticks_ms()

    if not released and time.ticks_diff(now, last_move) > IDLE_RELEASE_MS:
        release()
        released = True

    # LED 狀態：收指令期間常亮，待機時每秒閃一下
    if not released:
        led.value(1)
        led_on = True
        last_blink = now
    else:
        elapsed = time.ticks_diff(now, last_blink)
        if led_on and elapsed >= LED_FLASH_MS:
            led.value(0)
            led_on = False
            last_blink = now
        elif not led_on and elapsed >= HEARTBEAT_MS:
            led.value(1)
            led_on = True
            last_blink = now

    time.sleep_ms(5)
