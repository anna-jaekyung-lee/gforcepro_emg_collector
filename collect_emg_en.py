#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
collect_emg.py — gForcePRO EMG Collector (GUI)

Features:
  - 8-channel EMG real-time acquisition with timestamps
  - Optional IMU (Quaternion) simultaneous acquisition
  - 1kHz+12bit experimental (firmware response logged)
  - Software filters: Notch (60Hz), BPF (20~450Hz) — scipy
  - Auto-save by interval (seconds)
  - GUI: timer, connection status, live sample rate
  - Logo image support (set LOGO_PATH)

Usage:
  pip install bleak scipy numpy
  python collect_emg.py

Filter notes:
  - SDK delivers raw ADC values with no filtering
  - Filters applied optionally in software:
      Notch filter  : 60Hz power line noise rejection (Q=30)
      Bandpass filter: 20~450Hz (4th-order Butterworth) — EMG band

Packet structure:
  - BLE cannot transmit per-sample; data arrives in packets
  - Packet = header(1B) + 128B EMG = 8ch x 16 samples
  - ~62 packets received per second
"""

import asyncio
import struct
import time
import os
import threading
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from datetime import datetime

try:
    from scipy.signal import butter, sosfilt, sosfilt_zi, iirnotch, sosfiltfilt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("[WARNING] scipy not found. Run: pip install scipy")

from gforce import (
    GForceProfile, DataNotifFlags, NotifDataType, ResponseResult
)

# ── ────────────────────────────────────────────────────────────────
LOGO_PATH = ""          # Logo image path (e.g. "logo.png"). Leave empty if none.
SAVE_DIR  = "."         # Save folder
N_CH      = 8           # Number of EMG channels
FS        = 1000        # Sampling rate (Hz)
SAMPLES_PER_PACKET = 16 # Samples per packet (8bit mode)
# ────────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════
#
# ══════════════════════════════════════════════════════════════════════════
class EmgFilter:
    """
    Real-time EMG filter (scipy IIR, per-sample application)

    Filter structure:
      1. Notch filter  @ 60 Hz, Q=30  → power line noise rejection
      2. Bandpass filter 20–450 Hz, 4th-order Butterworth → EMG band

    [NOTE] SDK has no built-in filters. Applied on PC after reception.
    [NOTE] Uses sosfilt + zi for real-time per-sample filtering.
           For batch use, sosfiltfilt (zero-phase) is recommended.
    """
    def __init__(self, fs=1000, notch_hz=60, notch_q=30, bp_low=20, bp_high=450, order=4):
        if not SCIPY_AVAILABLE:
            self.enabled = False
            return
        self.enabled = True
        self.fs = fs

        # Notch filter (60Hz)
        w0 = notch_hz / (fs / 2)
        b_n, a_n = iirnotch(w0, notch_q)
        # iirnotch returns ba format → convert to sos
        from scipy.signal import tf2sos
        self.sos_notch = tf2sos(b_n, a_n)

        # Bandpass filter (20–450Hz), 4th-order Butterworth
        nyq = fs / 2
        low  = bp_low  / nyq
        high = bp_high / nyq
        self.sos_bp = butter(order, [low, high], btype='band', output='sos')

        # Per-channel filter state (zi)
        self.zi_notch = [sosfilt_zi(self.sos_notch) for _ in range(N_CH)]
        self.zi_bp    = [sosfilt_zi(self.sos_bp)    for _ in range(N_CH)]

    def apply(self, samples: np.ndarray) -> np.ndarray:
        """
        samples: shape (n_samples, N_CH), float
        returns: shape (n_samples, N_CH), float (filtered)
        """
        if not self.enabled:
            return samples
        out = samples.copy().astype(float)
        for ch in range(N_CH):
            x = out[:, ch]
            x, self.zi_notch[ch] = sosfilt(self.sos_notch, x, zi=self.zi_notch[ch])
            x, self.zi_bp[ch]    = sosfilt(self.sos_bp,    x, zi=self.zi_bp[ch])
            out[:, ch] = x
        return out


# ══════════════════════════════════════════════════════════════════════════
#
# ══════════════════════════════════════════════════════════════════════════
class DataRecorder:
    """
    Saves EMG + IMU data to CSV.
    Splits into new file every N seconds if auto_save_sec > 0.
    """
    def __init__(self, save_dir=SAVE_DIR, auto_save_sec=0):
        self.save_dir = save_dir
        self.auto_save_sec = auto_save_sec
        self.file = None
        self.writer_lock = threading.Lock()
        self.row_count = 0
        self.file_start_time = None
        self.current_path = None

    def _new_file(self):
        if self.file:
            self.file.close()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_path = os.path.join(self.save_dir, f"emg_{ts}.csv")
        self.file = open(self.current_path, "w", newline="")
        header = "timestamp_s," + ",".join([f"CH{i+1}" for i in range(N_CH)]) + ",quat_w,quat_x,quat_y,quat_z\n"
        self.file.write(header)
        self.file_start_time = time.perf_counter()
        self.row_count = 0
        print(f"[SAVE] New file: {self.current_path}")

    def start(self):
        self._new_file()

    def write_emg(self, timestamp, emg_row, quat=None):
        if self.file is None:
            return
        # Auto-split save
        if self.auto_save_sec > 0 and self.file_start_time is not None:
            if (time.perf_counter() - self.file_start_time) >= self.auto_save_sec:
                self._new_file()

        quat_str = ""
        if quat:
            quat_str = "," + ",".join(f"{v:.6f}" for v in quat)
        else:
            quat_str = ",,,,"

        line = f"{timestamp:.6f}," + ",".join(str(v) for v in emg_row) + quat_str + "\n"
        with self.writer_lock:
            self.file.write(line)
            self.row_count += 1

    def stop(self):
        if self.file:
            self.file.close()
            self.file = None
            print(f"[SAVE] Done: {self.current_path} ({self.row_count:,} rows)")


# ══════════════════════════════════════════════════════════════════════════
# Main GUI App
# ══════════════════════════════════════════════════════════════════════════
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("gForcePRO EMG Collector")
        self.root.resizable(False, False)

        self.gforce = GForceProfile()
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.loop_thread.start()

        self.recording = False
        self.connected = False
        self.scan_results = []
        self.start_time = None
        self.sample_counter = 0
        self.packet_cnt = 0
        self.last_rate_time = None
        self.last_rate_cnt = 0
        self.current_quat = None
        self.emg_filter = None
        self.recorder = None
        self.baseline = None
        self.baseline_buf = []
        self.baseline_done = True
        self.baseline_sec = 0

        self._build_ui()
        self._tick()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

 # ── UI ──────────────────────────────────────────────────────────
    def _build_ui(self):
        FONT = ("Consolas", 10)
        FONT_B = ("Consolas", 10, "bold")
        PAD = {"padx": 8, "pady": 4}

        # Logo
        if LOGO_PATH and os.path.exists(LOGO_PATH):
            try:
                from PIL import Image, ImageTk
                img = Image.open(LOGO_PATH).resize((200, 60))
                self._logo_img = ImageTk.PhotoImage(img)
                tk.Label(self.root, image=self._logo_img).pack(pady=(8, 0))
            except Exception:
                pass

        # Status
        status_frame = tk.LabelFrame(self.root, text="Status", font=FONT_B, **PAD)
        status_frame.pack(fill="x", padx=10, pady=4)

        self.lbl_conn   = tk.Label(status_frame, text="● Not connected", fg="gray", font=FONT_B)
        self.lbl_conn.grid(row=0, column=0, sticky="w", **PAD)

        self.lbl_timer  = tk.Label(status_frame, text="⏱  00:00:00", font=("Consolas", 14, "bold"))
        self.lbl_timer.grid(row=0, column=1, **PAD)

        self.lbl_rate   = tk.Label(status_frame, text="Sample rate: —", font=FONT)
        self.lbl_rate.grid(row=0, column=2, **PAD)

        self.lbl_file   = tk.Label(status_frame, text="Output file: —", font=FONT, fg="gray")
        self.lbl_file.grid(row=1, column=0, columnspan=3, sticky="w", **PAD)

        # Settings
        cfg_frame = tk.LabelFrame(self.root, text="Recording Settings", font=FONT_B, **PAD)
        cfg_frame.pack(fill="x", padx=10, pady=4)

        # Sample rate
        tk.Label(cfg_frame, text="Sample Rate (Hz)", font=FONT).grid(row=0, column=0, sticky="w", **PAD)
        self.var_samprate = tk.StringVar(value="1000")
        cb_sr = ttk.Combobox(cfg_frame, textvariable=self.var_samprate, values=["500", "650", "1000"], width=8, font=FONT)
        cb_sr.grid(row=0, column=1, **PAD)

        # Resolution
        tk.Label(cfg_frame, text="Resolution (bit)", font=FONT).grid(row=0, column=2, sticky="w", **PAD)
        self.var_resolution = tk.StringVar(value="8")
        cb_res = ttk.Combobox(cfg_frame, textvariable=self.var_resolution, values=["8", "12"], width=8, font=FONT)
        cb_res.grid(row=0, column=3, **PAD)

        tk.Label(cfg_frame, text="※ 1000Hz+12bit is experimental — check firmware response", fg="gray", font=("Consolas", 9)).grid(
            row=1, column=0, columnspan=4, sticky="w", padx=8)

        # IMU
        self.var_imu = tk.BooleanVar(value=False)
        tk.Checkbutton(cfg_frame, text="IMU (Quaternion)", variable=self.var_imu, font=FONT).grid(
            row=2, column=0, columnspan=2, sticky="w", **PAD)

        # Filter
        self.var_filter = tk.BooleanVar(value=SCIPY_AVAILABLE)
        cb_filt = tk.Checkbutton(cfg_frame, text="Software Filter",
                                 variable=self.var_filter, font=FONT,
                                 state="normal" if SCIPY_AVAILABLE else "disabled")
        cb_filt.grid(row=2, column=2, sticky="w", **PAD)

 # Filter
        filt_frame = tk.LabelFrame(self.root, text="Filter Settings (configure before recording)", font=FONT_B, **PAD)
        filt_frame.pack(fill="x", padx=10, pady=2)

        tk.Label(filt_frame, text="Notch (Hz)", font=FONT).grid(row=0, column=0, sticky="w", **PAD)
        self.var_notch = tk.StringVar(value="60")
        tk.Entry(filt_frame, textvariable=self.var_notch, width=6, font=FONT).grid(row=0, column=1, **PAD)

        tk.Label(filt_frame, text="Notch Q", font=FONT).grid(row=0, column=2, sticky="w", **PAD)
        self.var_notch_q = tk.StringVar(value="30")
        tk.Entry(filt_frame, textvariable=self.var_notch_q, width=6, font=FONT).grid(row=0, column=3, **PAD)

        tk.Label(filt_frame, text="BPF Low (Hz)", font=FONT).grid(row=0, column=4, sticky="w", **PAD)
        self.var_bp_low = tk.StringVar(value="20")
        tk.Entry(filt_frame, textvariable=self.var_bp_low, width=6, font=FONT).grid(row=0, column=5, **PAD)

        tk.Label(filt_frame, text="BPF High (Hz)", font=FONT).grid(row=0, column=6, sticky="w", **PAD)
        self.var_bp_high = tk.StringVar(value="450")
        tk.Entry(filt_frame, textvariable=self.var_bp_high, width=6, font=FONT).grid(row=0, column=7, **PAD)

        tk.Label(filt_frame, text="BPF Order", font=FONT).grid(row=0, column=8, sticky="w", **PAD)
        self.var_bp_order = tk.StringVar(value="4")
        ttk.Combobox(filt_frame, textvariable=self.var_bp_order, values=["2","4","6","8"], width=4, font=FONT).grid(row=0, column=9, **PAD)

        tk.Label(filt_frame, text="※ Notch: power line noise rejection (60Hz)  |  BPF: EMG band (20~450Hz recommended)",
                 fg="gray", font=("Consolas", 9)).grid(row=1, column=0, columnspan=10, sticky="w", padx=8)

        # baseline correction
        self.var_baseline = tk.BooleanVar(value=False)
        tk.Checkbutton(cfg_frame, text="Baseline Correction", variable=self.var_baseline,
                       font=FONT).grid(row=3, column=0, columnspan=2, sticky="w", **PAD)
        tk.Label(cfg_frame, text="REST duration (sec)", font=FONT).grid(row=3, column=2, sticky="w", **PAD)
        self.var_baseline_sec = tk.StringVar(value="3")
        tk.Entry(cfg_frame, textvariable=self.var_baseline_sec, width=5, font=FONT).grid(row=3, column=3, **PAD)
        tk.Label(cfg_frame, text="※ Subtracts mean of first N-sec REST period as zero-point offset",
                 fg="gray", font=("Consolas", 9)).grid(row=4, column=0, columnspan=5, sticky="w", padx=8)

        # Auto-save
        tk.Label(cfg_frame, text="Auto-save interval (sec, 0=off)", font=FONT).grid(row=5, column=0, sticky="w", **PAD)
        self.var_autosave = tk.StringVar(value="0")
        tk.Entry(cfg_frame, textvariable=self.var_autosave, width=6, font=FONT).grid(row=5, column=1, **PAD)

        # Save folder
        tk.Label(cfg_frame, text="Save Folder", font=FONT).grid(row=5, column=2, sticky="w", **PAD)
        self.var_savedir = tk.StringVar(value=os.path.abspath(SAVE_DIR))
        tk.Entry(cfg_frame, textvariable=self.var_savedir, width=24, font=FONT).grid(row=5, column=3, **PAD)
        tk.Button(cfg_frame, text="Browse", font=FONT, command=self._browse_dir).grid(row=5, column=4, **PAD)

        # Buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=8)

        self.btn_scan    = tk.Button(btn_frame, text="🔍 Scan",     font=FONT_B, width=12, command=self._on_scan)
        self.btn_scan.grid(row=0, column=0, padx=6)

        self.btn_connect = tk.Button(btn_frame, text="🔗 Connect",     font=FONT_B, width=12, command=self._on_connect, state="disabled")
        self.btn_connect.grid(row=0, column=1, padx=6)

        self.btn_start   = tk.Button(btn_frame, text="▶ Start", font=FONT_B, width=12, command=self._on_start, state="disabled", bg="#4CAF50", fg="white")
        self.btn_start.grid(row=0, column=2, padx=6)

        self.btn_stop    = tk.Button(btn_frame, text="■ Stop", font=FONT_B, width=12, command=self._on_stop, state="disabled", bg="#f44336", fg="white")
        self.btn_stop.grid(row=0, column=3, padx=6)

        # Device list
        list_frame = tk.LabelFrame(self.root, text="Detected Devices", font=FONT_B, **PAD)
        list_frame.pack(fill="x", padx=10, pady=4)
        self.listbox = tk.Listbox(list_frame, height=4, font=FONT, selectmode="single")
        self.listbox.pack(fill="x", padx=4, pady=4)

        # Log
        log_frame = tk.LabelFrame(self.root, text="Log", font=FONT_B, **PAD)
        log_frame.pack(fill="both", expand=True, padx=10, pady=4)
        self.log_text = tk.Text(log_frame, height=8, font=("Consolas", 9), state="disabled", bg="#1e1e1e", fg="#cccccc")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _log(self, msg):
        def _do():
            self.log_text.config(state="normal")
            self.log_text.insert("end", f"[{datetime.now().strftime('%H:%M:%S')}] {msg}\n")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self.root.after(0, _do)

    def _browse_dir(self):
        d = filedialog.askdirectory()
        if d:
            self.var_savedir.set(d)

    # ── Scan ──────────────────────────────────────────────────────────────
    def _on_scan(self):
        self.btn_scan.config(state="disabled", text="Scanning...")
        self.listbox.delete(0, "end")
        self._log("Scanning BLE (5s)...")
        self._submit(self._do_scan())

    async def _do_scan(self):
        try:
            results = await self.gforce.scan(5, "gForce")
            self.scan_results = results
            def _update():
                self.listbox.delete(0, "end")
                if not results:
                    self.listbox.insert("end", "No devices found.")
                    self._log("No devices found.")
                else:
                    for d in results:
                        self.listbox.insert("end", f"  {d['name']}  |  {d['address']}  |  RSSI={d['rssi']}dB")
                    self._log(f"{len(results)} device(s) found.")
                self.btn_scan.config(state="normal", text="🔍 Scan")
                self.btn_connect.config(state="normal" if results else "disabled")
            self.root.after(0, _update)
        except Exception as e:
            self._log(f"Scan error: {e}")
            self.root.after(0, lambda: self.btn_scan.config(state="normal", text="🔍 Scan"))

    # ── Connect ──────────────────────────────────────────────────────────────
    def _on_connect(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning("Selection required", "Please select a device to connect.")
            return
        idx = sel[0]
        addr = self.scan_results[idx]["address"]
        self._log(f"Connecting to: {addr}")
        self.btn_connect.config(state="disabled", text="Connecting...")
        self._submit(self._do_connect(addr))

    async def _do_connect(self, addr):
        try:
            await self.gforce.connect(addr)
            self.connected = True
            def _update():
                self.lbl_conn.config(text="● Connected", fg="green")
                self.btn_connect.config(text="🔗 Connected", state="disabled")
                self.btn_start.config(state="normal")
            self.root.after(0, _update)
            self._log("Connected successfully!")
        except Exception as e:
            self._log(f"Connection failed: {e}")
            self.root.after(0, lambda: self.btn_connect.config(state="normal", text="🔗 Connect"))

    # ── Start recording ─────────────────────────────────────────────────────────
    def _on_start(self):
        sampRate   = int(self.var_samprate.get())
        resolution = int(self.var_resolution.get())
        use_imu    = self.var_imu.get()
        use_filter = self.var_filter.get() and SCIPY_AVAILABLE
        auto_sec   = int(self.var_autosave.get() or 0)
        save_dir   = self.var_savedir.get()

 # Filter (GUI )
        if use_filter:
            try:
                notch_hz  = float(self.var_notch.get())
                notch_q   = float(self.var_notch_q.get())
                bp_low    = float(self.var_bp_low.get())
                bp_high   = float(self.var_bp_high.get())
                bp_order  = int(self.var_bp_order.get())
                self.emg_filter = EmgFilter(fs=sampRate, notch_hz=notch_hz, notch_q=notch_q,
                                            bp_low=bp_low, bp_high=bp_high, order=bp_order)
                self._log(f"Filter: Notch {notch_hz}Hz (Q={notch_q}), BPF {bp_low}~{bp_high}Hz order={bp_order}")
            except Exception as e:
                self._log(f"Filter config error: {e} → running without filter")
                self.emg_filter = None
        else:
            self.emg_filter = None

 #
        self.recorder = DataRecorder(save_dir=save_dir, auto_save_sec=auto_sec)
        self.recorder.start()

        # Baseline init
        self.baseline       = None
        self.baseline_buf   = []
        self.baseline_done  = not self.var_baseline.get()
        self.baseline_sec   = float(self.var_baseline_sec.get()) if self.var_baseline.get() else 0

        self.recording = True
        self.start_time = None
        self.sample_counter = 0   # Cumulative sample counter for timestamp
        self.packet_cnt = 0
        self.last_rate_time = None
        self.last_rate_cnt = 0
        self.current_quat = None

        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")

        self._log(f"Recording started — {sampRate}Hz / {resolution}bit / IMU={'ON' if use_imu else 'OFF'} / Filter={'ON' if use_filter else 'OFF'}")
        if resolution == 12 and sampRate == 1000:
            self._log("⚠ 1000Hz+12bit experimental — check firmware response code")

        self._submit(self._do_start(sampRate, resolution, use_imu))

    async def _do_start(self, sampRate, resolution, use_imu):
        def emg_config_cb(resp):
            if resp == ResponseResult.RSP_CODE_SUCCESS:
                self._log("EMG config: SUCCESS")
            elif resp == ResponseResult.RSP_CODE_BAD_PARAM:
                self._log("❌ EMG config failed: RSP_CODE_BAD_PARAM")
            elif resp == ResponseResult.RSP_CODE_NOT_SUPPORT:
                self._log("❌ EMG config failed: RSP_CODE_NOT_SUPPORT")
            else:
                self._log(f"EMG config response: {resp}")

        await self.gforce.setEmgRawDataConfig(
            sampRate=sampRate,
            channelMask=0xFF,
            dataLen=128,
            resolution=resolution,
            cb=emg_config_cb,
            timeout=1000,
        )
        await asyncio.sleep(0.3)

 # (EMG + IMU)
        flags = DataNotifFlags.DNF_EMG_RAW
        if use_imu:
            flags |= DataNotifFlags.DNF_QUATERNION
            self._log("IMU ON — may be unstable at 1kHz due to BLE bandwidth limits")

        await self.gforce.setDataNotifSwitch(flags, lambda r: self._log(f"DataNotifSwitch: {r}"), 1000)
        await asyncio.sleep(0.3)
        await self.gforce.startDataNotification(self._on_data)

    # ── Data callback ──────────────────────────────────────────────────
    def _on_data(self, data):
        if not self.recording:
            return

        now = time.perf_counter()

        # ── IMU ─────────────────────────────
        if data[0] == NotifDataType.NTF_QUAT_FLOAT_DATA and len(data) == 17:
            quat = list(struct.unpack("<4f", data[1:17]))
            self.current_quat = quat
            return

        # ── EMG ─────────────────────────────────────────
        # [Packet structure]
        # data[0]     : 0x08 (NTF_EMG_ADC_DATA)
        # data[1:129] : 128 bytes EMG
        #   8bit:  1 byte/ch, 16 samples x 8ch per packet
        #   12bit: 2 bytes/ch (LSB 12bit), 8 samples x 8ch per packet
        if data[0] != NotifDataType.NTF_EMG_ADC_DATA:
            return

        resolution = int(self.var_resolution.get())
        sampRate   = int(self.var_samprate.get())

        if resolution == 8:
            if len(data) != 129:
                return
            raw = np.array(data[1:], dtype=np.uint8).reshape(-1, N_CH)  # (16, 8)
            n_samples = 16
        else:  # 12bit
            if len(data) != 129:
                return
            # 12bit: 2 bytes per channel, LSB 12bit
            # Packet layout: [CH0_lo, CH0_hi, CH1_lo, CH1_hi, ...] per sample
            # 128 bytes / (2 bytes x 8ch) = 8 samples/packet
            raw_bytes = data[1:]
            n_samples = len(raw_bytes) // (2 * N_CH)
            raw = np.zeros((n_samples, N_CH), dtype=np.int16)
            for s in range(n_samples):
                for ch in range(N_CH):
                    idx = (s * N_CH + ch) * 2
                    unsigned = (raw_bytes[idx] | (raw_bytes[idx+1] << 8)) & 0x0FFF
                    # 12bit signed: subtract midpoint (2048)
                    raw[s, ch] = unsigned - 2048

        # Set reference at first EMG packet
        if self.start_time is None:
            self.start_time = now
            self.last_rate_time = now

        # Baseline collection period
        if not self.baseline_done:
            self.baseline_buf.append(raw.astype(float))
            collected_sec = sum(len(b) for b in self.baseline_buf) / sampRate
            pct = min(int(collected_sec / self.baseline_sec * 100), 100)
            self.root.after(0, lambda p=pct: self.lbl_rate.config(text=f"Collecting baseline... {p}%  (relax your arm)"))
            if collected_sec >= self.baseline_sec:
                all_data = np.concatenate(self.baseline_buf, axis=0)
                self.baseline = all_data.mean(axis=0)
                self.baseline_done = True
                self._log(f"Baseline done: {self.baseline.round(1).tolist()}")
                self.root.after(0, lambda: self.lbl_rate.config(text="✅ Baseline done"))
            self.sample_counter += n_samples
            return  # skip saving during baseline period

        # Baseline correction
        raw_f = raw.astype(float)
        if self.baseline is not None:
            raw_f = raw_f - self.baseline

 # Filter
        if self.emg_filter and self.emg_filter.enabled:
            filtered = self.emg_filter.apply(raw_f)
        else:
            filtered = raw_f

        # Timestamp: cumulative sample counter → always increasing, never negative
        dt = 1.0 / sampRate
        for s in range(n_samples):
            t = (self.sample_counter + s) * dt
            row = [int(round(v)) for v in filtered[s]]
            self.recorder.write_emg(t, row, quat=self.current_quat)

        self.sample_counter += n_samples

 # Sample rate (100 )
        self.packet_cnt += 1
        self.last_rate_cnt += 1
        if self.last_rate_cnt >= 100 and self.last_rate_time is not None:
            period = now - self.last_rate_time
            rate = 100 * n_samples / period
            self.root.after(0, lambda r=rate: self.lbl_rate.config(text=f"Sample rate: {r:.1f} Hz"))
            self.last_rate_time = now
            self.last_rate_cnt = 0

        # Update file label
        if self.recorder and self.recorder.current_path:
            fname = os.path.basename(self.recorder.current_path)
            self.root.after(0, lambda f=fname: self.lbl_file.config(text=f"Output file: {f}"))

    # ── Stop recording ─────────────────────────────────────────────────────────
    def _on_stop(self):
        self.recording = False
        self.btn_stop.config(state="disabled")
        self.btn_start.config(state="normal")
        self._log("Stopping...")
        self._submit(self._do_stop())

    async def _do_stop(self):
        await self.gforce.stopDataNotification()
        await asyncio.sleep(0.3)
        await self.gforce.setDataNotifSwitch(DataNotifFlags.DNF_OFF, lambda r: None, 1000)
        if self.recorder:
            self.recorder.stop()
            self._log(f"Saved: {self.recorder.current_path}")
        self._log("Recording stopped.")

    # ── Timer tick ───────────────────────────────────────────────────────
    def _tick(self):
        if self.recording and self.start_time is not None:
            elapsed = int(time.perf_counter() - self.start_time)
            h, r = divmod(elapsed, 3600)
            m, s = divmod(r, 60)
            self.lbl_timer.config(text=f"⏱  {h:02d}:{m:02d}:{s:02d}")
        self.root.after(500, self._tick)


# ══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
