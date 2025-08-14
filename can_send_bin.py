import can, struct, sys, time

# ==================== CONFIG ====================
CHANNEL = "COM3@115200"
BITRATE = 1_000_000
FIRMWARE_BIN = "firmware.bin"

CAN_ID_START = 0x7FF
CAN_ID_DATA  = 0x100
CAN_ID_END   = 0x7FE
ACK_ID       = 0x101      # MCU -> PC: 0xAA = ACK, 0xAC = READY
NACK_ID      = 0x102      # MCU -> PC: [idx_lo, idx_hi]
READY_CODE   = 0xAC

BATCH_SIZE   = 8          # ให้ตรงกับ MCU (ยังหลุดค่อยลดเป็น 4)
ACK_TIMEOUT  = 10.0
START_MAX_MS = 20000

# slow-start เฟรมแรก ๆ
WARMUP_FRAMES  = 32
WARMUP_SLEEP_S = 0.0008   # 0.5–1.5 ms แล้วแต่บัส/อะแดปเตอร์
# =================================================

# === CRC แบบเดียวกับ MCU (init=0xFFFFFFFF, poly=0xEDB88320, no final XOR)
def crc32_stm(data: bytes, seed: int = 0xFFFFFFFF) -> int:
    crc = seed
    for b in data:
        c = (crc ^ b) & 0xFF
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if (c & 1) else 0)
        crc = ((crc >> 8) ^ c) & 0xFFFFFFFF
    return crc

# --- โหลดไฟล์และคำนวณ CRC หลังประกาศฟังก์ชันแล้ว ---
with open(FIRMWARE_BIN, "rb") as f:
    fw_data = f.read()

size = len(fw_data)
crc  = crc32_stm(fw_data)
total_frames = (size + 6) // 6  # ไม่ได้ใช้ก็ลบได้

print(f"Firmware size={size} bytes, CRC=0x{crc:08X}")
print("PC HEAD:", fw_data[:32].hex().upper())
print("PC TAIL:", fw_data[-32:].hex().upper())

bus = can.interface.Bus(interface="robotell", channel=CHANNEL, bitrate=BITRATE)

def show_progress(sent_bytes, total_bytes):
    bar_len = 40
    percent = (sent_bytes / total_bytes) * 100 if total_bytes else 0
    filled = int(bar_len * percent / 100)
    bar = "#" * filled + "-" * (bar_len - filled)
    sys.stdout.write(f"\r[{bar}] {percent:6.2f}%")
    sys.stdout.flush()

def drain_bus():
    while True:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break

def poll_nack_now():
    while True:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            return None
        if msg.arbitration_id == NACK_ID and len(msg.data) >= 2:
            return msg.data[0] | (msg.data[1] << 8)
        # ACK/READY ปล่อยผ่าน รอเช็คตอนจบ batch

def wait_for_ack_or_nack(timeout=ACK_TIMEOUT):
    deadline = time.time() + timeout
    last_nack = None
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break
        if msg.arbitration_id == NACK_ID and len(msg.data) >= 2:
            last_nack = msg.data[0] | (msg.data[1] << 8)
            continue
        if msg.arbitration_id == ACK_ID and len(msg.data) >= 1 and msg.data[0] == 0xAA:
            if last_nack is not None:
                return ("NACK", last_nack)
            return ("ACK", None)
    if last_nack is not None:
        return ("NACK", last_nack)
    raise TimeoutError("Timeout waiting for ACK/NACK")

def start_handshake(total_size, max_ms=START_MAX_MS):
    payload = b"STAR" + struct.pack("<I", total_size)
    t0 = time.time()
    while (time.time() - t0) * 1000 < max_ms:
        bus.send(can.Message(arbitration_id=CAN_ID_START, data=payload, is_extended_id=False))
        deadline = time.time() + 0.12
        while time.time() < deadline:
            msg = bus.recv(timeout=0.01)
            if not msg:
                continue
            if msg.arbitration_id == ACK_ID and len(msg.data) >= 1 and msg.data[0] == 0xAA:
                print("START: ACK")
                return True
    return False

def wait_ready(timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = bus.recv(timeout=0.1)
        if not msg:
            continue
        if msg.arbitration_id == ACK_ID and len(msg.data) >= 1 and msg.data[0] == READY_CODE:
            print("READY: MCU erased & ready")
            return True
    return False

try:
    # ===== START =====
    drain_bus()
    if not start_handshake(size, max_ms=START_MAX_MS):
        print("\nTimeout waiting for START ACK (ลองรีเซ็ตบอร์ด)")
        raise SystemExit(1)

    if not wait_ready(timeout=10.0):
        print("No READY from MCU — fallback delay 300 ms")
        time.sleep(0.3)

    # ===== DATA =====
    batch_counter = 0
    frame_idx = 0
    i = 0  # จำนวน byte ที่ส่งไปแล้ว

    while i < size:
        payload = fw_data[i:i+6]
        frame = struct.pack("<H", frame_idx) + payload
        bus.send(can.Message(arbitration_id=CAN_ID_DATA, data=frame, is_extended_id=False))

        if frame_idx < WARMUP_FRAMES:
            time.sleep(WARMUP_SLEEP_S)

        frame_idx += 1
        i += len(payload)
        batch_counter += 1

        redo_immediate = poll_nack_now()
        if redo_immediate is not None:
            old_idx = frame_idx
            frame_idx = redo_immediate
            i = redo_immediate * 6
            batch_counter = 0
            print(f"\n<rewind@immediate> to idx={redo_immediate} (from {old_idx})")
            show_progress(i, size)
            continue

        if batch_counter >= BATCH_SIZE or i >= size:
            sig, redo = wait_for_ack_or_nack(timeout=ACK_TIMEOUT)
            if sig == "NACK":
                old_idx = frame_idx
                frame_idx = redo
                i = redo * 6
                batch_counter = 0
                print(f"\n<rewind@batch> to idx={redo} (from {old_idx})")
            else:
                batch_counter = 0

        show_progress(i, size)

    print("\nSent DATA")

    # ===== END =====
    bus.send(can.Message(arbitration_id=CAN_ID_END, data=b"END\x00" + struct.pack("<I", crc), is_extended_id=False))
    print("Sent END")

finally:
    try:
        bus.shutdown()
    except Exception:
        pass
