import os
import sys
import time
import struct
import threading
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# ============== ต้องมี python-can และ Robotell interface ==============
# pip install python-can
import can

# ==================== ค่าตั้งต้น (แก้ใน UI ได้) ====================
DEFAULT_CHANNEL = "COM3@115200"     # รูปแบบ: COMx@baud (เช่น COM3@115200) คือพอร์ต/บอดเรตของอะแดปเตอร์ Robotell (ไม่ใช่บิทรต CAN)
DEFAULT_BITRATE = 1_000_000         # บิทรตของบัส CAN (ทุกโหนดต้องตรงกัน)

CAN_ID_START = 0x7FF
CAN_ID_DATA  = 0x100
CAN_ID_END   = 0x7FE
ACK_ID       = 0x101      # MCU -> PC: 0xAA = ACK, 0xAC = READY
NACK_ID      = 0x102      # MCU -> PC: [idx_lo, idx_hi]
READY_CODE   = 0xAC

DEFAULT_BATCH_SIZE   = 8          # จำนวนเฟรมที่ส่งรวดเดียวก่อนรอ ACK/NACK
DEFAULT_ACK_TIMEOUT  = 10.0       # เวลารอ MCU ตอบหนึ่ง batch (วินาที)
DEFAULT_START_MAX_MS = 20000      # เวลารวมที่ยอมให้พยายาม START handshake (มิลลิวินาที)

DEFAULT_WARMUP_FRAMES  = 32       # จำนวนเฟรมแรกที่ส่งแบบค่อย ๆ ไป (slow-start)
DEFAULT_WARMUP_SLEEP_S = 0.0008   # หน่วงต่อเฟรมช่วงวอร์ม (วินาที)
# ====================================================================


# ---------- Tooltip (ไม่มี lib เพิ่ม ใช้ได้เลย) ----------
class Tooltip:
    def __init__(self, widget, text, wrap=380):
        self.widget = widget
        self.text = text
        self.wrap = wrap
        self.tipwindow = None
        widget.bind("<Enter>", self.show)
        widget.bind("<Leave>", self.hide)

    def show(self, _event=None):
        if self.tipwindow or not self.text:
            return
        x = self.widget.winfo_rootx() + 15
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5
        self.tipwindow = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        lbl = tk.Label(
            tw, text=self.text, justify="left",
            background="#2b2b2b", foreground="#ffffff",
            relief="solid", borderwidth=1, padx=8, pady=6,
            wraplength=self.wrap, font=("Segoe UI", 9)
        )
        lbl.pack()

    def hide(self, _event=None):
        if self.tipwindow:
            self.tipwindow.destroy()
            self.tipwindow = None

def add_tip(widget, text):
    Tooltip(widget, text)


def crc32_stm(data: bytes, seed: int = 0xFFFFFFFF) -> int:
    """CRC32 แบบเดียวกับ MCU (init=0xFFFFFFFF, poly=0xEDB88320, no final XOR)"""
    crc = seed
    for b in data:
        c = (crc ^ b) & 0xFF
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if (c & 1) else 0)
        crc = ((crc >> 8) ^ c) & 0xFFFFFFFF
    return crc


class CANFlasher(threading.Thread):
    """
    รันใน background thread:
    - เปิดบัส
    - ทำ handshake
    - ส่งเฟรม DATA พร้อมจัดการ ACK/NACK
    - ส่ง END + CRC
    มี callback เพื่ออัปเดต UI อย่างปลอดภัย
    """
    def __init__(
        self,
        channel: str,
        bitrate: int,
        fw_path: str,
        batch_size: int,
        ack_timeout: float,
        start_max_ms: int,
        warmup_frames: int,
        warmup_sleep_s: float,
        on_log,
        on_progress,
        on_done,
    ):
        super().__init__(daemon=True)
        self.channel = channel
        self.bitrate = bitrate
        self.fw_path = fw_path
        self.batch_size = batch_size
        self.ack_timeout = ack_timeout
        self.start_max_ms = start_max_ms
        self.warmup_frames = warmup_frames
        self.warmup_sleep_s = warmup_sleep_s

        self.on_log = on_log
        self.on_progress = on_progress
        self.on_done = on_done

        self._cancel = threading.Event()
        self.bus = None

    def cancel(self):
        self._cancel.set()

    def log(self, msg: str):
        if self.on_log:
            self.on_log(msg)

    def progress(self, sent, total):
        if self.on_progress:
            self.on_progress(sent, total)

    # ---------- helpers ----------
    def drain_bus(self):
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break

    def poll_nack_now(self):
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return None
        # NACK immediate
            if msg.arbitration_id == NACK_ID and len(msg.data) >= 2:
                return msg.data[0] | (msg.data[1] << 8)

    def wait_for_ack_or_nack(self, timeout):
        deadline = time.time() + timeout
        last_nack = None
        while time.time() < deadline:
            rem = max(0.0, deadline - time.time())
            msg = self.bus.recv(timeout=rem)
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

    def start_handshake(self, total_size, max_ms):
        payload = b"STAR" + struct.pack("<I", total_size)
        t0 = time.time()
        while (time.time() - t0) * 1000 < max_ms:
            if self._cancel.is_set():
                return False
            self.bus.send(can.Message(arbitration_id=CAN_ID_START, data=payload, is_extended_id=False))
            deadline = time.time() + 0.12
            while time.time() < deadline:
                if self._cancel.is_set():
                    return False
                msg = self.bus.recv(timeout=0.01)
                if not msg:
                    continue
                if msg.arbitration_id == ACK_ID and len(msg.data) >= 1 and msg.data[0] == 0xAA:
                    self.log("START: ACK")
                    return True
        return False

    def wait_ready(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._cancel.is_set():
                return False
            msg = self.bus.recv(timeout=0.1)
            if not msg:
                continue
            if msg.arbitration_id == ACK_ID and len(msg.data) >= 1 and msg.data[0] == READY_CODE:
                self.log("READY: MCU erased & ready")
                return True
        return False

    # ---------- main run ----------
    def run(self):
        try:
            with open(self.fw_path, "rb") as f:
                fw_data = f.read()
            size = len(fw_data)
            crc = crc32_stm(fw_data)

            self.log(f"Firmware: {os.path.basename(self.fw_path)} | size={size} bytes | CRC=0x{crc:08X}")
            self.log("HEAD: " + fw_data[:32].hex().upper())
            self.log("TAIL: " + (fw_data[-32:].hex().upper() if size >= 32 else fw_data.hex().upper()))

            self.log(f"Open CAN bus (robotell) @ {self.channel}, bitrate={self.bitrate}")
            self.bus = can.interface.Bus(interface="robotell", channel=self.channel, bitrate=self.bitrate)
            

            # START
            self.drain_bus()
            self.log("Handshaking (START)…")
            if not self.start_handshake(size, self.start_max_ms):
                raise RuntimeError("START ACK timeout (Try to reset the board)")

            # READY
            if not self.wait_ready(timeout=10.0):
                self.log("No READY from MCU — fallback delay 300 ms")
                time.sleep(0.3)

            # DATA
            batch_counter = 0
            frame_idx = 0
            i = 0
            self.progress(0, size)

            while i < size:
                if self._cancel.is_set():
                    raise RuntimeError("Canceled by user")

                payload = fw_data[i:i+6]
                frame = struct.pack("<H", frame_idx) + payload
                self.bus.send(can.Message(arbitration_id=CAN_ID_DATA, data=frame, is_extended_id=False))

                if frame_idx < self.warmup_frames:
                    time.sleep(self.warmup_sleep_s)

                frame_idx += 1
                i += len(payload)
                batch_counter += 1

                redo_immediate = self.poll_nack_now()
                if redo_immediate is not None:
                    old_idx = frame_idx
                    frame_idx = redo_immediate
                    i = redo_immediate * 6
                    batch_counter = 0
                    self.log(f"<rewind@immediate> to idx={redo_immediate} (from {old_idx})")
                    self.progress(i, size)
                    continue

                if batch_counter >= self.batch_size or i >= size:
                    sig, redo = self.wait_for_ack_or_nack(timeout=self.ack_timeout)
                    if sig == "NACK":
                        old_idx = frame_idx
                        frame_idx = redo
                        i = redo * 6
                        batch_counter = 0
                        self.log(f"<rewind@batch> to idx={redo} (from {old_idx})")
                    else:
                        batch_counter = 0

                self.progress(i, size)

            self.log("Sent DATA")

            self.bus.send(can.Message(arbitration_id=CAN_ID_END, data=b"END\x00" + struct.pack("<I", crc), is_extended_id=False))
            self.log("Sent END — DONE :)")
            self.on_done(success=True, message="Upload complete")
        except Exception as e:
            self.on_done(success=False, message=str(e))
        finally:
            try:
                if self.bus:
                    self.bus.shutdown()
            except Exception:
                pass


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 CAN Bootloader GUI (Robotell)")
        self.geometry("790x620")
        self.minsize(760, 560)

        # Styles for hints
        style = ttk.Style(self)
        style.configure("Hint.TLabel", foreground="#666666", font=("Segoe UI", 9))

        # State
        self.worker = None
        self.ui_queue = queue.Queue()

        # Vars
        self.var_channel = tk.StringVar(value=DEFAULT_CHANNEL)
        self.var_bitrate = tk.IntVar(value=DEFAULT_BITRATE)
        self.var_fwpath  = tk.StringVar(value="")
        self.var_batch   = tk.IntVar(value=DEFAULT_BATCH_SIZE)
        self.var_ackto   = tk.DoubleVar(value=DEFAULT_ACK_TIMEOUT)
        self.var_startms = tk.IntVar(value=DEFAULT_START_MAX_MS)
        self.var_warm_n  = tk.IntVar(value=DEFAULT_WARMUP_FRAMES)
        self.var_warm_s  = tk.DoubleVar(value=DEFAULT_WARMUP_SLEEP_S)

        self._build_ui()
        self.after(100, self._poll_ui_queue)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 6}

        frm_top = ttk.LabelFrame(self, text="Connection")
        frm_top.pack(fill="x", **pad)

        lbl_chan = ttk.Label(frm_top, text="Channel (Robotell):")
        lbl_chan.grid(row=0, column=0, sticky="w")
        ent_chan = ttk.Entry(frm_top, textvariable=self.var_channel, width=24)
        ent_chan.grid(row=0, column=1, sticky="w", padx=8)
        add_tip(ent_chan, "Type: COMx@baud (e.g. COM3@115200)\nis speed serial of adapter, not bitrate on CAN bus")

        lbl_bitrate = ttk.Label(frm_top, text="Bitrate (CAN):")
        lbl_bitrate.grid(row=0, column=2, sticky="e")
        ent_bit = ttk.Entry(frm_top, textvariable=self.var_bitrate, width=10)
        ent_bit.grid(row=0, column=3, sticky="w", padx=8)
        add_tip(ent_bit, "Bitrate of CAN bus in bits per second (e.g. 1000000 = 1 Mbps)\nmust match MCU and all devices on the bus")

        frm_fw = ttk.LabelFrame(self, text="Firmware")
        frm_fw.pack(fill="x", **pad)

        ttk.Label(frm_fw, text="Binary file:").grid(row=0, column=0, sticky="w")
        ent_fw = ttk.Entry(frm_fw, textvariable=self.var_fwpath, width=60)
        ent_fw.grid(row=0, column=1, sticky="we", padx=8)
        frm_fw.columnconfigure(1, weight=1)

        def browse():
            path = filedialog.askopenfilename(
                title="Select firmware file (.bin)",
                filetypes=[("Binary firmware", "*.bin"), ("All files", "*.*")]
            )
            if path:
                self.var_fwpath.set(path)

        btn_browse = ttk.Button(frm_fw, text="Browse…", command=browse)
        btn_browse.grid(row=0, column=2, sticky="e")
        add_tip(btn_browse, "Select the .bin firmware file to upload to the MCU")

        # Advanced
        self.frm_adv = ttk.LabelFrame(self, text="Advanced")
        self.frm_adv.pack(fill="x", **pad)

        # Row 1
        ttk.Label(self.frm_adv, text="BATCH_SIZE:").grid(row=0, column=0, sticky="w")
        ent_batch = ttk.Entry(self.frm_adv, textvariable=self.var_batch, width=8)
        ent_batch.grid(row=0, column=1, sticky="w", padx=8)
        add_tip(ent_batch, "Number of frames to send consecutively before waiting for ACK/NACK\nMore = faster, but if bus error occurs, may resend a lot. Start with 8, if it fails, reduce to 4.")

        ttk.Label(self.frm_adv, text="ACK_TIMEOUT (s):").grid(row=0, column=2, sticky="e")
        ent_ack = ttk.Entry(self.frm_adv, textvariable=self.var_ackto, width=8)
        ent_ack.grid(row=0, column=3, sticky="w", padx=8)
        add_tip(ent_ack, "Time to wait for MCU response after batch completion in seconds\nToo short → Timeout, too long → Slow. Start with 10s and adjust as needed.")

        ttk.Label(self.frm_adv, text="START_MAX_MS:").grid(row=0, column=4, sticky="e")
        ent_start = ttk.Entry(self.frm_adv, textvariable=self.var_startms, width=10)
        ent_start.grid(row=0, column=5, sticky="w", padx=8)
        add_tip(ent_start, "Total time (ms) to try sending START for handshake\nIf MCU responds slowly/recently reset, this can be increased")

        # Row 2
        ttk.Label(self.frm_adv, text="WARMUP_FRAMES:").grid(row=1, column=0, sticky="w")
        ent_warmn = ttk.Entry(self.frm_adv, textvariable=self.var_warm_n, width=8)
        ent_warmn.grid(row=1, column=1, sticky="w", padx=8)
        add_tip(ent_warmn, "Number of initial frames to send slowly to allow adapter/bus to settle\nTypically 8–32")

        ttk.Label(self.frm_adv, text="WARMUP_SLEEP_S:").grid(row=1, column=2, sticky="e")
        ent_warms = ttk.Entry(self.frm_adv, textvariable=self.var_warm_s, width=10)
        ent_warms.grid(row=1, column=3, sticky="w", padx=8)
        add_tip(ent_warms, "Sleep time per frame during warmup (seconds)\n0.0005–0.0015 is ideal, if frames drop increase slightly")

        # Hints (บรรทัดเล็ก ๆ ใต้แต่ละพารามิเตอร์)
        ttk.Label(self.frm_adv, text="high=fast but risky; low=secure but slow", style="Hint.TLabel") \
            .grid(row=2, column=0, columnspan=2, sticky="w", padx=2)
        ttk.Label(self.frm_adv, text="if MCU writes flash slowly, increase this value", style="Hint.TLabel") \
            .grid(row=2, column=2, columnspan=2, sticky="w", padx=2)
        ttk.Label(self.frm_adv, text="If handshake takes too long/no ACK, increase this value", style="Hint.TLabel") \
            .grid(row=2, column=4, columnspan=2, sticky="w", padx=2)

        ttk.Label(self.frm_adv, text="Warmup frames help reduce burst at start", style="Hint.TLabel") \
            .grid(row=3, column=0, columnspan=2, sticky="w", padx=2)
        ttk.Label(self.frm_adv, text="Increase by 0.0002s if frames drop", style="Hint.TLabel") \
            .grid(row=3, column=2, columnspan=2, sticky="w", padx=2)

        # ปุ่มอธิบายทั้งหมด
        btn_explain = ttk.Button(self.frm_adv, text="Explain All", command=self.show_explain)
        btn_explain.grid(row=3, column=4, columnspan=2, sticky="e", padx=8)

        # Controls
        frm_ctrl = ttk.Frame(self)
        frm_ctrl.pack(fill="x", **pad)

        self.btn_start = ttk.Button(frm_ctrl, text="Start Upload", command=self.start_flash)
        self.btn_start.pack(side="left")

        self.btn_cancel = ttk.Button(frm_ctrl, text="Cancel", command=self.cancel_flash, state="disabled")
        self.btn_cancel.pack(side="left", padx=8)

        # Progress
        frm_prog = ttk.LabelFrame(self, text="Progress")
        frm_prog.pack(fill="x", **pad)

        self.prog = ttk.Progressbar(frm_prog, orient="horizontal", mode="determinate", maximum=100)
        self.prog.pack(fill="x", padx=8, pady=10)

        self.lbl_prog = ttk.Label(frm_prog, text="0 %")
        self.lbl_prog.pack(anchor="e", padx=8)

        # Log
        frm_log = ttk.LabelFrame(self, text="Log")
        frm_log.pack(fill="both", expand=True, **pad)

        self.txt_log = tk.Text(frm_log, wrap="word", height=16)
        self.txt_log.pack(fill="both", expand=True, padx=8, pady=8)
        self.txt_log.configure(state="disabled")

    # ---------- UI helpers ----------
    def ui_log(self, msg: str):
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", msg + "\n")
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")

    def set_progress(self, sent, total):
        pct = 0 if total == 0 else min(100, max(0, sent * 100 / total))
        self.prog["value"] = pct
        self.lbl_prog["text"] = f"{pct:5.1f} %"

    def _enqueue(self, fn, *args, **kwargs):
        self.ui_queue.put((fn, args, kwargs))

    def _poll_ui_queue(self):
        try:
            while True:
                fn, args, kwargs = self.ui_queue.get_nowait()
                fn(*args, **kwargs)
        except queue.Empty:
            pass
        self.after(80, self._poll_ui_queue)

    # ---------- Actions ----------
    def start_flash(self):
        if self.worker and self.worker.is_alive():
            messagebox.showwarning("Working", "last process is still running")
            return

        fw = self.var_fwpath.get().strip()
        if not fw or not os.path.isfile(fw):
            messagebox.showerror("File Not Found", "Please select a valid .bin file")
            return

        # Lock UI
        self.btn_start.config(state="disabled")
        self.btn_cancel.config(state="normal")
        self.set_progress(0, 1)
        self.ui_log("== Start flashing ==")

        # callbacks ที่ข้าม thread → ใช้ queue กลับมา UI-thread
        def on_log(msg):
            self._enqueue(self.ui_log, msg)

        def on_progress(sent, total):
            self._enqueue(self.set_progress, sent, total)

        def on_done(success, message):
            def finish():
                self.btn_start.config(state="normal")
                self.btn_cancel.config(state="disabled")
                self.ui_log(("✅ " if success else "❌ ") + message)
                if not success:
                    messagebox.showerror("failed", message)
            self._enqueue(finish)

        self.worker = CANFlasher(
            channel=self.var_channel.get().strip(),
            bitrate=self.var_bitrate.get(),
            fw_path=fw,
            batch_size=self.var_batch.get(),
            ack_timeout=float(self.var_ackto.get()),
            start_max_ms=self.var_startms.get(),
            warmup_frames=self.var_warm_n.get(),
            warmup_sleep_s=float(self.var_warm_s.get()),
            on_log=on_log,
            on_progress=on_progress,
            on_done=on_done,
        )
        self.worker.start()

    def cancel_flash(self):
        if self.worker and self.worker.is_alive():
            self.worker.cancel()
            self.ui_log("Cancelling... Please wait")
        else:
            self.ui_log("No process to cancel")

    def show_explain(self):
        text = (
            "Parameter Guide (Summary):\n"
            "• Channel (Robotell): Format COMx@baud, e.g. COM3@115200 is the serial port/baud rate of the adapter (not the CAN bitrate)\n"
            "• Bitrate (CAN): CAN bus speed (bits/second) must be the same for all nodes, e.g. 1000000=1Mbps\n"
            "• BATCH_SIZE: Number of frames to send in a row before waiting for ACK/NACK — higher = faster but risk rewind if bus errors; start at 8, reduce to 4 if drops occur\n"
            "• ACK_TIMEOUT: Time to wait for MCU to respond to one batch — too short will timeout, too long will be slow; normal is 10s\n"
            "• START_MAX_MS: Total time trying to START handshake — increase if MCU just reset/erased flash\n"
            "• WARMUP_FRAMES: Number of initial frames sent slowly to allow system to stabilize (8–32)\n"
            "• WARMUP_SLEEP_S: Time to sleep between warmup frames (0.0005–0.0015 seconds)\n\n"
            "Tips:\n"
            "- '<rewind@…>' frequently → reduce BATCH_SIZE/increase WARMUP_SLEEP_S/reduce Bitrate\n"
            "- 'START ACK timeout' → reset board or increase START_MAX_MS\n"
            "- long hang at end of batch → increase ACK_TIMEOUT\n"
        )
        messagebox.showinfo("Option Explanation", text)


if __name__ == "__main__":
    # บน Windows ให้ Tkinter เร็วขึ้นนิดหน่อย
    if sys.platform.startswith("win"):
        try:
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    app = App()
    app.mainloop()
