#!/usr/bin/env python3
"""單獨測試 Pi 5 ←→ Pico W 的 UART，不需要啟動整台伺服器。

在 Pi 上執行：
    python tools/uart_test.py

會做三件事：
    1. 印出序列埠設定，並檢查有沒有東西在跟你搶這個埠
    2. 靜聽 3 秒，看 Pico 有沒有主動送 "gimbal ready" 過來
    3. 送出一組角度指令，觀察伺服馬達會不會動

Pi 5 注意：/dev/serial0 指向 ttyAMA10（獨立的三針除錯接頭），
不是排針的 GPIO14/15。要用 /dev/ttyAMA0。

想做迴路測試（只測 Pi 這一側，不接 Pico）：
    把 Pi 的 pin 8 (TXD) 和 pin 10 (RXD) 用一條杜邦線直接短接，再跑
    python tools/uart_test.py --loopback
    收得回自己送出去的字串，就代表 Pi 的 UART 本身是好的。
"""

import argparse
import json
import subprocess
import sys
import time

PORT = "/dev/ttyAMA0"   # Pi 5 的 GPIO14/15；serial0 是除錯接頭，不要用
BAUD = 115200


def check_port_owner(port):
    """看看有沒有其他程式佔著這個序列埠（最常見的是序列主控台 getty）。"""
    try:
        result = subprocess.run(["fuser", "-v", port],
                                capture_output=True, text=True, timeout=5)
        output = (result.stdout + result.stderr).strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    return output or None


def check_console():
    """檢查開機參數有沒有把序列主控台掛在這條線上。"""
    for path in ("/boot/firmware/cmdline.txt", "/boot/cmdline.txt"):
        try:
            with open(path) as handle:
                cmdline = handle.read()
        except OSError:
            continue
        if "console=serial0" in cmdline or "console=ttyAMA0" in cmdline:
            return path
        return None
    return None


def check_uart_config():
    """看 config.txt 裡跟 UART 有關的設定。"""
    for path in ("/boot/firmware/config.txt", "/boot/config.txt"):
        try:
            with open(path) as handle:
                lines = [line.strip() for line in handle]
        except OSError:
            continue
        hits = [line for line in lines
                if "uart" in line.lower() and not line.startswith("#")]
        return path, hits
    return None, []


def list_serial_devices():
    """列出系統上的序列裝置，順便解開符號連結指向哪裡。"""
    import glob
    import os

    found = []
    patterns = ("/dev/serial*", "/dev/ttyAMA*", "/dev/ttyS[0-9]")
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            target = os.path.realpath(path)
            found.append(f"{path} → {target}" if target != path else path)
    return found


def drain(serial_port, seconds, label):
    """在指定秒數內把收到的東西全部印出來。"""
    print(f"  {label}（{seconds} 秒）...")
    deadline = time.monotonic() + seconds
    buffer = b""
    lines = []

    while time.monotonic() < deadline:
        waiting = serial_port.in_waiting
        if waiting:
            buffer += serial_port.read(waiting)
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("utf-8", "replace").strip()
                if text:
                    lines.append(text)
                    print(f"    ← 收到：{text!r}")
        time.sleep(0.05)

    if buffer:
        text = buffer.decode("utf-8", "replace").strip()
        lines.append(text)
        print(f"    ← 收到（沒有換行結尾）：{buffer!r}")
    if not lines:
        print("    ← 什麼都沒收到")
    return lines


def main():
    parser = argparse.ArgumentParser(description="UART 連線測試")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--loopback", action="store_true",
                        help="迴路測試：pin 8 和 pin 10 短接，只測 Pi 這一側")
    parser.add_argument("--listen", type=float, metavar="秒",
                        help="只監聽不送任何東西。搭配重新插拔 Pico 電源使用，"
                             "可單獨驗證 Pico→Pi 這個方向（開機時會送 gimbal ready）")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("找不到 pyserial，請執行：sudo apt install python3-serial")

    print("=" * 60)
    print("1. 檢查環境")
    print("=" * 60)

    console = check_console()
    if console:
        print(f"  ⚠ {console} 裡還留著 console=serial0")
        print("    序列主控台佔著這條線，Pico 送來的資料會被它吃掉。")
        print("    解法：sudo raspi-config → Interface Options → Serial Port")
        print("          「login shell over serial?」選 No，然後重開機")
    else:
        print("  ✓ 開機參數沒有把主控台掛在序列埠上")

    config_path, uart_lines = check_uart_config()
    if config_path is None:
        print("  ? 找不到 config.txt（不是在 Raspberry Pi 上跑？）")
    elif uart_lines:
        print(f"  ✓ {config_path} 裡的 UART 設定：")
        for line in uart_lines:
            print(f"      {line}")
    else:
        print(f"  ⚠ {config_path} 裡完全沒有 UART 相關設定")
        print("    UART 硬體沒被啟用。加上這一行然後重開機：")
        print("      echo 'enable_uart=1' | sudo tee -a " + config_path)

    devices = list_serial_devices()
    if devices:
        print("  ✓ 系統上的序列裝置：")
        for device in devices:
            print(f"      {device}")
    else:
        print("  ⚠ 找不到任何序列裝置")

    owner = check_port_owner(args.port)
    if owner:
        print(f"  ⚠ 有程式正在使用 {args.port}：")
        for line in owner.splitlines():
            print(f"      {line}")
        print("    如果是 server.py 自己，先把它停掉再測。")
    else:
        print(f"  ✓ 沒有其他程式佔用 {args.port}")

    print()
    print("=" * 60)
    print("2. 開啟序列埠並靜聽")
    print("=" * 60)

    try:
        port = serial.Serial(args.port, BAUD, timeout=0)
    except Exception as exc:
        sys.exit(f"  ✗ 打不開 {args.port}：{exc}")

    print(f"  ✓ 已開啟 {port.name}，鮑率 {port.baudrate}")
    print()

    if args.listen:
        print("=" * 60)
        print("單向監聽（只收不送）")
        print("=" * 60)
        print("  現在請把 Pico 的電源拔掉再插上。")
        print("  韌體開機時會主動送出 gimbal ready —— 收得到就代表")
        print("  Pico 的 GP4(TX，pin 6) → Pi 的 pin 10 這個方向是通的。\n")
        lines = drain(port, args.listen, "監聽中")
        print()

        # 只有真的看到 gimbal ready 才算數。電源切換時線上會出現 \x00 之類的
        # 假訊框，那是電位跳動不是資料，不能當成通了。
        if any("gimbal ready" in line for line in lines):
            print("  ✓ 收到 gimbal ready —— 回程（含 GND）是好的")
            print("    → 問題單獨落在 Pi pin 8 → Pico pin 7（GP5, RX）這個方向")
        elif lines:
            print("  ⚠ 收到東西，但不是 gimbal ready：")
            for line in lines:
                print(f"      {line!r}")
            print("    單獨的 \\x00 通常是電源切換造成的假訊框，不代表資料通了。")
            print("    如果是一堆亂碼，那就是鮑率不符。")
        else:
            print("  ✗ 完全沒收到")
            print("    → Pico 的 GP4（pin 6）、那條線、或 GND 有問題")
            print("    （若這段期間沒有重新插拔 Pico 電源，請再跑一次）")
        port.close()
        return

    if args.loopback:
        probe = b"LOOPBACK-TEST\n"
        print(f"  送出：{probe!r}")
        port.reset_input_buffer()
        port.write(probe)
        port.flush()
        replies = drain(port, 2, "等待自己送出的資料繞回來")
        print()
        if any("LOOPBACK-TEST" in line for line in replies):
            print("  ✓ 繞回來了，這條路徑完全導通")
        elif replies:
            print("  ✗ 收到東西，但不是我們送出去的內容：")
            for line in replies:
                print(f"      {line!r}")
            print("    單獨的 \\x00 是電位跳動或線路被拉低造成的假訊框。")
            print("    如果路徑上有通電的晶片在驅動同一條線，它會蓋掉訊號 ——")
            print("    橋接 Pico 針腳測試時，要先把 Pico 的電源拔掉。")
        else:
            print("  ✗ 完全收不回來 —— 這條路徑不通")
        port.close()
        return

    # Pico 只在開機時送一次 gimbal ready，如果它已經跑一陣子了就會錯過，
    # 所以主動 ping 一下讓它回應
    print("  → 送出 ping")
    port.write(b"ping\n")
    port.flush()
    replies = drain(port, 3, "等待 Pico 回應")
    got_ready = any("gimbal ready" in line for line in replies)
    if replies and not got_ready:
        print("    ⚠ 收到的不是 gimbal ready —— 單獨的 \\x00 是電位跳動造成的")
        print("      假訊框，不代表資料通了")

    print()
    print("=" * 60)
    print("3. 送出角度指令")
    print("=" * 60)
    print("  接下來會左右各轉一次再回中，注意看伺服馬達有沒有動。")

    moves = (
        (-30, 0, "pan −30°  接 GP10（pin 14）那顆，應該往左轉"),
        (30, 0, "pan +30°  同一顆，應該往右轉"),
        (0, -20, "tilt −20° 接 GP11（pin 15）那顆，應該往下"),
        (0, 20, "tilt +20° 同一顆，應該往上"),
        (0, 0, "兩顆都回到中間"),
    )

    sent = []
    for pan, tilt, label in moves:
        payload = json.dumps({"pan": pan, "tilt": tilt})
        sent.append(payload)
        print(f"  → {label}")
        port.write((payload + "\n").encode())
        port.flush()
        time.sleep(1.2)   # 慢一點，方便肉眼確認是哪一顆在動

    print()
    echoed = drain(port, 1, "看看有沒有回應")
    port.close()

    # Pico 韌體不會回送指令，所以收到跟送出一樣的內容只有一種可能
    if echoed and set(echoed) <= set(sent):
        print()
        print("  ⚠ 收到的內容跟送出的一模一樣 —— 這是迴路，不是 Pico 的回應。")
        print("    pin 8 和 pin 10 之間那條測試用的杜邦線還接著，先把它拆掉。")
        print("    （不過這也證明了 Pi 的 UART 本身完全正常。）")
        return

    print()
    print("=" * 60)
    print("結果判讀")
    print("=" * 60)
    if got_ready:
        print("  ✓ 收得到 gimbal ready —— 兩條線都通了。")
        print("    如果伺服馬達還是不動，問題在伺服本身（供電、接線、腳位）。")
    else:
        print("  ✗ 沒有收到 gimbal ready。依序檢查：")
        print("    a) Pico 有沒有電？（USB 或 VSYS，UART 三條線不供電）")
        print("    b) main.py 是不是存在 Pico 上、檔名正確？用 Thonny 確認")
        print("    c) TX/RX 有沒有交叉？Pi pin 8 → Pico pin 2，Pi pin 10 → Pico pin 1")
        print("    d) GND 有沒有共接？Pi pin 6 ←→ Pico pin 3")
        print("    e) Pi 的 UART 有沒有啟用？跑 --loopback 單獨測 Pi 這一側")


if __name__ == "__main__":
    main()
