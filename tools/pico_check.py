#!/usr/bin/env python3
"""透過 USB 檢查 Pico 上到底有沒有我們的韌體，並確認它跑不跑得起來。

在 Windows（Pico 用 USB 接電腦）執行：
    python tools/pico_check.py --port COM4

在 Pi 上（Pico 用 USB 接 Pi）執行：
    python tools/pico_check.py --port /dev/ttyACM0

會做四件事：
    1. 中斷正在執行的程式，問 MicroPython 版本
    2. 列出 Pico 上的檔案，確認 main.py 在不在
    3. 檢查 main.py 是不是我們這一份
    4. 軟重開機，聽看看有沒有送出 gimbal ready

注意：執行期間 Thonny 必須關閉，否則序列埠會被它佔住。
"""

import argparse
import sys
import time

BAUD = 115200


def read_until(port, terminator, timeout=4):
    deadline = time.monotonic() + timeout
    buffer = b""
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        if waiting:
            buffer += port.read(waiting)
            if terminator in buffer:
                break
        time.sleep(0.02)
    return buffer


def enter_raw_repl(port):
    """中斷執行中的程式並進入 raw REPL。"""
    for _ in range(3):
        port.write(b"\x03")      # Ctrl+C
        time.sleep(0.2)
    port.reset_input_buffer()
    port.write(b"\x01")          # Ctrl+A
    reply = read_until(port, b"raw REPL", 3)
    return b"raw REPL" in reply


def probe(port):
    """進不了 REPL 時，把裝置實際吐出來的位元組秀出來。

    看到 '>>>' 就是 MicroPython；一片空白或其他內容代表 Pico 上跑的是別的韌體。
    """
    print("=" * 58)
    print("診斷：這個裝置到底在說什麼")
    print("=" * 58)

    steps = (
        ("開啟後靜聽", None, 2),
        ("送出 Enter", b"\r\n", 1.5),
        ("送出 Ctrl+C（中斷執行中的程式）", b"\x03", 1.5),
        ("再送一次 Enter", b"\r\n", 1.5),
    )

    saw_anything = False
    for label, payload, wait in steps:
        port.reset_input_buffer()
        if payload:
            port.write(payload)
        time.sleep(wait)
        data = port.read(port.in_waiting) if port.in_waiting else b""
        if data:
            saw_anything = True
            print(f"  {label} → {data!r}")
        else:
            print(f"  {label} → （沒有回應）")

    print()
    if not saw_anything:
        print("  ✗ 這個裝置完全不回話，不是 MicroPython。")
        print()
        print("  最可能的情況：Pico 上根本沒裝 MicroPython。")
        print("  裝法（Thonny 內建，最簡單）：")
        print("    1. 拔掉 Pico 的 USB")
        print("    2. 按住板子上的 BOOTSEL 鈕不放，插回 USB，然後放開")
        print("    3. 開 Thonny → 右下角點直譯器 → Install MicroPython")
        print("    4. variant 選 Raspberry Pi Pico W，按 Install")
        print("    5. 裝完拔插一次，再把 pico/main.py 存成 Pico 上的 main.py")
    else:
        print("  → 有回應但不是 MicroPython 的 REPL，把上面的內容貼出來我看。")


def run(port, code):
    """在 raw REPL 裡執行一段程式，回傳 (輸出, 錯誤)。"""
    port.write(code.encode() + b"\x04")   # Ctrl+D 開始執行
    raw = read_until(port, b"\x04>", 6)

    if raw.startswith(b"OK"):
        raw = raw[2:]
    parts = raw.split(b"\x04")
    output = parts[0].decode("utf-8", "replace").strip() if parts else ""
    error = parts[1].decode("utf-8", "replace").strip() if len(parts) > 1 else ""
    return output, error


def upload(port, local_path, remote_name="main.py"):
    """透過 raw REPL 把檔案寫進 Pico，不需要 mpremote 或 Thonny。

    走 base64 是因為原始碼含中文註解，直接塞進 Python 字面值容易被跳脫規則咬到。
    """
    import base64

    data = open(local_path, "rb").read()
    print(f"  來源：{local_path}（{len(data)} 位元組）")

    setup = ("try:\n"
             "    import ubinascii as _b\n"
             "except ImportError:\n"
             "    import binascii as _b\n"
             f"_f=open({remote_name!r},'wb')")
    _, error = run(port, setup)
    if error:
        print(f"  ✗ 開檔失敗：{error}")
        return False

    encoded = base64.b64encode(data).decode()
    chunk_size = 256
    chunks = [encoded[i:i + chunk_size] for i in range(0, len(encoded), chunk_size)]

    for index, chunk in enumerate(chunks, 1):
        _, error = run(port, f"_f.write(_b.a2b_base64('{chunk}'))")
        if error:
            print(f"  ✗ 第 {index}/{len(chunks)} 段寫入失敗：{error}")
            return False
        print(f"\r  寫入中… {index}/{len(chunks)}", end="", flush=True)

    print()
    run(port, "_f.close()")

    output, error = run(port, f"import os;print(os.stat({remote_name!r})[6])")
    written = output.strip()
    if written == str(len(data)):
        print(f"  ✓ 寫入完成，Pico 上的 {remote_name} 為 {written} 位元組")
        return True
    print(f"  ✗ 大小不符：本機 {len(data)}，Pico {written or error}")
    return False


def main():
    parser = argparse.ArgumentParser(description="檢查 Pico 上的韌體")
    parser.add_argument("--port", required=True,
                        help="Pico 的序列埠，例如 COM4 或 /dev/ttyACM0")
    parser.add_argument("--upload", metavar="檔案",
                        help="把指定檔案寫成 Pico 上的 main.py，然後重新啟動")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("找不到 pyserial，請執行：pip install pyserial")

    try:
        port = serial.Serial(args.port, BAUD, timeout=0)
    except Exception as exc:
        sys.exit(f"✗ 打不開 {args.port}：{exc}\n"
                 f"  Thonny 或其他程式可能正佔用這個埠，先關掉再試。")

    print(f"✓ 已連上 {args.port}\n")

    if not enter_raw_repl(port):
        print("✗ 進不了 MicroPython 的 REPL，改用診斷模式看看原因。\n")
        probe(port)
        port.close()
        return

    if args.upload:
        print("=" * 58)
        print("上傳韌體")
        print("=" * 58)
        if not upload(port, args.upload):
            port.close()
            return
        print()

    print("=" * 58)
    print("1. MicroPython 版本")
    print("=" * 58)
    output, error = run(port, "import sys; print(sys.version); print(sys.implementation)")
    print(f"  {output or error}")

    print()
    print("=" * 58)
    print("2. Pico 上有哪些檔案")
    print("=" * 58)
    output, error = run(port, "import os; print(os.listdir())")
    print(f"  {output or error}")
    has_main = "main.py" in output

    if has_main:
        print("  ✓ main.py 存在")
    else:
        print("  ✗ 找不到 main.py —— 韌體根本沒被存進去")
        print("    用 Thonny 開啟 pico/main.py，選「另存新檔」→ Raspberry Pi Pico，")
        print("    檔名輸入 main.py（一定要這個名字才會開機自動執行）")

    if has_main:
        print()
        print("=" * 58)
        print("3. main.py 是不是我們這一份")
        print("=" * 58)
        output, error = run(
            port,
            "f=open('main.py');t=f.read();f.close();"
            "print(len(t));print('gimbal ready' in t);print('PAN_GPIO' in t)"
        )
        lines = output.splitlines()
        if len(lines) >= 3:
            size, has_ready, has_pan = lines[0], lines[1], lines[2]
            print(f"  檔案大小：{size} 位元組")
            if has_ready == "True" and has_pan == "True":
                print("  ✓ 內容符合，就是我們的雲台韌體")
            else:
                print("  ✗ 內容對不上 —— Pico 上是別的程式，不是雲台韌體")
        else:
            print(f"  讀取失敗：{error or output}")

    print()
    print("=" * 58)
    print("4. 軟重開機，看韌體跑不跑得起來")
    print("=" * 58)
    port.write(b"\x02")          # Ctrl+B 離開 raw REPL
    time.sleep(0.2)
    port.reset_input_buffer()
    port.write(b"\x04")          # Ctrl+D 軟重開機
    reply = read_until(port, b"gimbal ready", 5)
    text = reply.decode("utf-8", "replace")

    if "gimbal ready" in text:
        print("  ✓ 韌體正常啟動，看到 gimbal ready")
        print("  → Pico 這一側沒問題，問題在 UART 接線")
        print("    檢查：Pi pin 8 → Pico pin 2、Pi pin 10 → Pico pin 1、GND 共接")
    else:
        print("  ✗ 沒看到 gimbal ready。Pico 回應的內容：")
        for line in text.splitlines():
            if line.strip():
                print(f"      {line.rstrip()}")
        print("  → 如果上面有 Traceback，就是韌體執行時出錯了，把訊息貼出來")

    port.close()


if __name__ == "__main__":
    main()
