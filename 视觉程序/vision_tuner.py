#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉与串口联调上位机。

启动时不打开串口；点击「串口：关」后才按 ASCII CSV 协议发送当前视觉量。
相机、识别、ROI、映射和速度参数均实时生效；左侧显示完整画面。
"""

from __future__ import annotations

import importlib
import json
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import cv2
import numpy as np

try:
    from PIL import Image, ImageTk
    _PIL_OK = True
except ImportError:
    _PIL_OK = False
    print("[警告] 未安装 Pillow：pip3 install Pillow", flush=True)

V5 = importlib.import_module("rdk_ball_stable_v5")

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_PATH = SCRIPT_DIR / "ball_stable_config.json"
EXPORT_PATH = SCRIPT_DIR / "ball_stable_config_export.json"

WIN_W, WIN_H = 800, 480
PANEL_W = 270
PREVIEW_W = WIN_W - PANEL_W - 12
PREVIEW_H = 300

BG = "#1a1a2e"
PANEL_BG = "#16213e"
TEXT_FG = "#e2e8f0"
ACCENT = "#38bdf8"
OK = "#22c55e"
WARN = "#f59e0b"
ERR = "#ef4444"
BTN_BG = "#0f3460"
BTN_SAVE = "#166534"

SLIDERS = [
    ("camera", "auto_exposure", "曝光模式", 0, 3, 0),
    ("camera", "exposure", "曝光", 1, 1000, 0),
    ("camera", "gain", "增益", 0, 128, 0),
    ("camera", "gamma", "伽马", 50, 500, 0),
    ("camera", "brightness", "亮度", -64, 64, 0),
    ("camera", "contrast", "对比度", 0, 100, 0),
    ("camera", "saturation", "饱和度", 0, 128, 0),
    ("camera", "sharpness", "锐度", 0, 100, 0),
    ("camera", "backlight", "背光", 0, 8, 0),
    ("camera", "auto_white_balance", "自动白平衡", 0, 1, 0),
    ("binary", "threshold", "二值阈值", 1, 80, 0),
    ("binary", "background_kernel", "背景核", 3, 61, 0),
    ("binary", "open_kernel", "开运算核", 1, 15, 0),
    ("binary", "close_kernel", "闭运算核", 1, 15, 0),
    ("roi", "x", "ROI X", 0, 639, 0),
    ("roi", "y", "ROI Y", 0, 479, 0),
    ("roi", "width", "ROI 宽度", 20, 640, 0),
    ("roi", "height", "ROI 高度", 10, 240, 0),
    ("mapping", "target_position", "目标位置", 0, 448, 0),
    ("signal", "velocity_alpha", "速度EMA×100", 0, 100, 0),
    ("signal", "velocity_deadband_unit_s", "速度死区", 0, 50, 0),
    ("signal", "max_dt_s_x100", "最大dt×100", 1, 100, 0),
    ("signal", "velocity_limit_unit_s", "速度限幅", 100, 10000, 0),
]


def signal_value(config: dict, key: str) -> int:
    signal = config.setdefault("signal", {})
    if key == "velocity_alpha":
        return int(round(float(signal.get(key, 0.35)) * 100.0))
    if key == "max_dt_s_x100":
        return int(round(float(signal.get("max_dt_s", 0.2)) * 100.0))
    return int(signal.get(key, 0))


def set_signal_value(config: dict, key: str, value: int) -> None:
    signal = config.setdefault("signal", {})
    if key == "velocity_alpha":
        signal[key] = float(value) / 100.0
    elif key == "max_dt_s_x100":
        signal["max_dt_s"] = float(value) / 100.0
    else:
        signal[key] = float(value)


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("视觉调参与串口联调")
        self.geometry(f"{WIN_W}x{WIN_H}+0+0")
        self.resizable(False, False)
        self.configure(bg=BG)
        try:
            self.attributes("-fullscreen", True)
        except Exception:
            pass

        self.config_data = V5.load_config(CONFIG_PATH)
        self.capture = V5.LatestRoiCapture(self.config_data, retain_full_frame=True)
        self.detector = V5.RoiBallDetector(self.config_data)
        self.motion = V5.MotionEstimator(self.config_data)
        self.serial = V5.SerialComm(self.config_data.get("serial", {}))
        self.serial.enabled = False
        self._serial_on = False
        self._frame_seq = 0
        self._px_per_unit = V5.csv_px_per_unit(self.config_data["mapping"])

        self._photo = None
        self._last_seq = -1
        self._show_mask = True
        self._uvc_after_id = None
        self._roi_after_id = None
        self._vars: dict[str, tk.IntVar] = {}
        self._val_lbls: dict[str, tk.Label] = {}

        self._build_ui()
        self._load_sliders()
        if not self.capture.open():
            self._set_status("摄像头打开失败", ERR)
        else:
            self._set_status("已启动（串口默认关闭，调好后再开）", OK)
        self._tick()

    def _build_ui(self) -> None:
        left = tk.Frame(self, bg=BG, width=WIN_W - PANEL_W, height=WIN_H)
        left.pack(side=tk.LEFT, fill=tk.BOTH)
        left.pack_propagate(False)
        tk.Label(left, text="视觉调参  |  offset + velocity 联调", bg=BG, fg=ACCENT,
                 font=("DejaVu Sans", 12, "bold")).pack(pady=(5, 1))
        self.canvas = tk.Canvas(left, width=PREVIEW_W, height=PREVIEW_H, bg="#0d0d0d",
                                highlightthickness=1, highlightbackground="#334155")
        self.canvas.pack(padx=6)
        self.info_lbl = tk.Label(left, text="等待画面…", bg=BG, fg=TEXT_FG,
                                 font=("DejaVu Sans Mono", 8), justify=tk.LEFT,
                                 wraplength=PREVIEW_W, anchor=tk.W)
        self.info_lbl.pack(pady=(3, 0), padx=6, anchor=tk.W, fill=tk.X)
        self.signal_lbl = tk.Label(left, text="", bg=BG, fg="#a5b4fc",
                                   font=("DejaVu Sans Mono", 8), justify=tk.LEFT,
                                   wraplength=PREVIEW_W, anchor=tk.W)
        self.signal_lbl.pack(pady=(1, 0), padx=6, anchor=tk.W, fill=tk.X)
        self.serial_lbl = tk.Label(left, text="串口：关闭", bg=BG, fg="#fbbf24",
                                   font=("DejaVu Sans Mono", 8), justify=tk.LEFT,
                                   wraplength=PREVIEW_W, anchor=tk.W)
        self.serial_lbl.pack(pady=(1, 0), padx=6, anchor=tk.W, fill=tk.X)
        self.status_lbl = tk.Label(left, text="", bg=BG, fg=OK, font=("DejaVu Sans", 8))
        self.status_lbl.pack(side=tk.BOTTOM, pady=1)

        right = tk.Frame(self, bg=PANEL_BG, width=PANEL_W, height=WIN_H)
        right.pack(side=tk.RIGHT, fill=tk.BOTH)
        right.pack_propagate(False)
        self._build_panel(right)

    def _build_panel(self, parent: tk.Frame) -> None:
        tk.Label(parent, text="参数（实时/防抖生效）", bg=PANEL_BG, fg=ACCENT,
                 font=("DejaVu Sans", 10, "bold")).pack(pady=(5, 1))
        btn = tk.Frame(parent, bg=PANEL_BG, height=112)
        btn.pack(side=tk.BOTTOM, fill=tk.X)
        btn.pack_propagate(False)
        self._btn_serial = tk.Button(btn, text="串口：关", bg=BTN_BG, fg=TEXT_FG,
                                     font=("DejaVu Sans", 9, "bold"), relief=tk.FLAT,
                                     command=self._toggle_serial)
        self._btn_serial.pack(fill=tk.X, padx=7, pady=(3, 1))
        tk.Button(btn, text="应用相机参数", bg=BTN_BG, fg=TEXT_FG,
                  font=("DejaVu Sans", 8), relief=tk.FLAT,
                  command=self._apply_uvc_now).pack(fill=tk.X, padx=7, pady=1)
        row = tk.Frame(btn, bg=PANEL_BG)
        row.pack(fill=tk.X, padx=7, pady=(1, 3))
        tk.Button(row, text="保存", bg=BTN_SAVE, fg="white", relief=tk.FLAT,
                  width=5, command=self._save).pack(side=tk.LEFT, padx=1)
        tk.Button(row, text="掩码", bg=BTN_BG, fg=TEXT_FG, relief=tk.FLAT,
                  width=5, command=self._toggle_mask).pack(side=tk.LEFT, padx=1)
        tk.Button(row, text="复位", bg=BTN_BG, fg=TEXT_FG, relief=tk.FLAT,
                  width=5, command=self._reset).pack(side=tk.LEFT, padx=1)
        tk.Button(row, text="退出", bg="#374151", fg=TEXT_FG, relief=tk.FLAT,
                  width=5, command=self._quit).pack(side=tk.RIGHT, padx=1)

        mid = tk.Frame(parent, bg=PANEL_BG)
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        box = tk.Canvas(mid, bg=PANEL_BG, highlightthickness=0, width=PANEL_W - 8)
        scroll = tk.Scrollbar(mid, orient=tk.VERTICAL, command=box.yview)
        frame = tk.Frame(box, bg=PANEL_BG)
        frame.bind("<Configure>", lambda e: box.configure(scrollregion=box.bbox("all")))
        box.create_window((0, 0), window=frame, anchor=tk.NW, width=PANEL_W - 26)
        box.configure(yscrollcommand=scroll.set)
        box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 0))
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TScale", background=PANEL_BG, troughcolor="#0f172a")

        sections = {"camera": "── 相机 UVC ──", "binary": "── 识别 ──",
                    "roi": "── ROI 区域 ──", "mapping": "── 映射 ──", "signal": "── 速度信号 ──"}
        last = None
        for section, key, name, lo, hi, dec in SLIDERS:
            if sections[section] != last:
                last = sections[section]
                tk.Label(frame, text=last, bg=PANEL_BG, fg=ACCENT,
                         font=("DejaVu Sans", 8, "bold")).pack(fill=tk.X, padx=5, pady=(6, 1))
            row = tk.Frame(frame, bg=PANEL_BG)
            row.pack(fill=tk.X, padx=4, pady=1)
            head = tk.Frame(row, bg=PANEL_BG)
            head.pack(fill=tk.X)
            tk.Label(head, text=name, bg=PANEL_BG, fg=TEXT_FG,
                     font=("DejaVu Sans", 7)).pack(side=tk.LEFT)
            var = tk.IntVar(value=0)
            full = f"{section}.{key}"
            self._vars[full] = var
            label = tk.Label(head, text="0", bg=PANEL_BG, fg=ACCENT,
                             font=("DejaVu Sans Mono", 7), width=7, anchor=tk.E)
            label.pack(side=tk.RIGHT)
            self._val_lbls[full] = label
            scale = ttk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=var,
                              command=lambda _v, s=section, k=key, d=dec: self._on_slider(s, k, d))
            scale.pack(fill=tk.X)
            scale.bind("<Button-1>", lambda e, w=scale: w.focus_set())
        self.bind("<KeyPress-s>", lambda e: self._save())

    def _load_sliders(self) -> None:
        for section, key, _name, _lo, _hi, _dec in SLIDERS:
            if section == "signal":
                value = signal_value(self.config_data, key)
            else:
                value = int(self.config_data.get(section, {}).get(key, 0))
            full = f"{section}.{key}"
            self._vars[full].set(value)
            self._val_lbls[full].config(text=str(value))

    def _on_slider(self, section: str, key: str, _dec: int) -> None:
        full = f"{section}.{key}"
        value = int(self._vars[full].get())
        if section == "binary" and key in ("background_kernel", "open_kernel", "close_kernel"):
            if value % 2 == 0:
                value += 1
                self._vars[full].set(value)
        self._val_lbls[full].config(text=str(value))
        if section == "camera":
            self.config_data.setdefault("camera", {})[key] = value
            if self._uvc_after_id is not None:
                self.after_cancel(self._uvc_after_id)
            self._uvc_after_id = self.after(220, self._apply_uvc_now)
        elif section == "signal":
            set_signal_value(self.config_data, key, value)
            self.motion = V5.MotionEstimator(self.config_data)
        elif section == "roi":
            if self._roi_after_id is not None:
                self.after_cancel(self._roi_after_id)
            self._roi_after_id = self.after(80, self._apply_roi_now)
        else:
            self.config_data.setdefault(section, {})[key] = value
            self.detector.config = self.config_data
            if section == "mapping":
                self.motion = V5.MotionEstimator(self.config_data)

    def _apply_uvc_now(self) -> None:
        self._uvc_after_id = None
        camera = self.config_data.setdefault("camera", {})
        for section, key, *_ in SLIDERS:
            if section == "camera":
                camera[key] = int(self._vars[f"camera.{key}"].get())
        applied = V5.CameraControls(self.capture.device).apply(camera)
        if applied:
            self._set_status(f"已应用 {len(applied)} 项相机参数", OK)
        else:
            self._set_status("没有成功应用相机参数，请检查 v4l2-ctl/设备", WARN)

    def _current_roi(self) -> tuple[int, int, int, int]:
        width, height = self.capture.actual_size
        if width <= 0:
            width = int(self.config_data.get("camera", {}).get("width", 640))
        if height <= 0:
            height = int(self.config_data.get("camera", {}).get("height", 480))
        values = [int(self._vars[f"roi.{key}"].get()) for key in ("x", "y", "width", "height")]
        rect = V5.clamp_rect({"x": values[0], "y": values[1], "width": values[2], "height": values[3]}, width, height)
        return rect

    def _apply_roi_now(self) -> None:
        self._roi_after_id = None
        x, y, w, h = self._current_roi()
        for key, value in zip(("x", "y", "width", "height"), (x, y, w, h)):
            full = f"roi.{key}"
            self._vars[full].set(value)
            self._val_lbls[full].config(text=str(value))
        self.config_data["roi"] = {"x": x, "y": y, "width": w, "height": h}
        self.config_data["mapping"]["map_x_left"] = 0.0
        self.config_data["mapping"]["map_x_right"] = float(max(0, w - 1))
        self.capture.set_roi((x, y, w, h))
        self.detector.reset()
        self.motion.reset()
        self._last_seq = -1
        self._set_status(f"ROI 已更新 x={x} y={y} w={w} h={h}", OK)

    def _sync_sliders(self) -> None:
        if self._roi_after_id is not None:
            self.after_cancel(self._roi_after_id)
            self._roi_after_id = None
        self._apply_roi_now()
        for section, key, *_ in SLIDERS:
            value = int(self._vars[f"{section}.{key}"].get())
            if section == "signal":
                set_signal_value(self.config_data, key, value)
            else:
                self.config_data.setdefault(section, {})[key] = value

    def _save(self) -> None:
        try:
            self._sync_sliders()
            text = json.dumps(self.config_data, indent=2, ensure_ascii=False) + "\n"
            CONFIG_PATH.write_text(text, encoding="utf-8")
            EXPORT_PATH.write_text(text, encoding="utf-8")
            self._set_status(f"已保存 → {CONFIG_PATH.name} / export", OK)
        except Exception as exc:
            self._set_status(f"保存失败: {exc}", ERR)

    def _toggle_serial(self) -> None:
        if self._serial_on:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial.ser = None
            self._serial_on = False
            self._btn_serial.config(text="串口：关", bg=BTN_BG)
            self.serial_lbl.config(text="串口：关闭", fg="#fbbf24")
            self._set_status("串口已关闭，不再发送", WARN)
            return
        self.serial.enabled = True
        ok = self.serial.open()
        if ok and self.serial.ser is not None and self.serial.ser.is_open:
            self._serial_on = True
            self._btn_serial.config(text="串口：开 ●", bg=ERR)
            self.serial_lbl.config(text=f"串口：已打开 {self.serial.port}（正在发送 CSV 帧）", fg=ERR)
            self._set_status(f"串口已开 {self.serial.port}，将发送视觉数据", OK)
        else:
            self.serial.ser = None
            self._serial_on = False
            self._btn_serial.config(text="串口：关", bg=BTN_BG)
            self.serial_lbl.config(text="串口：打开失败", fg=ERR)
            if not getattr(V5, "_SERIAL_AVAILABLE", False):
                self._set_status("pyserial 未安装：pip3 install pyserial", ERR)
            else:
                self._set_status(f"打开失败，检查 {self.serial.port} 是否存在/被占用", ERR)

    def _toggle_mask(self) -> None:
        self._show_mask = not self._show_mask

    def _reset(self) -> None:
        self.detector.reset()
        self.motion.reset()
        self._set_status("检测与速度状态已复位", WARN)

    def _set_status(self, text: str, color: str) -> None:
        self.status_lbl.config(text=text, fg=color)

    def _draw_preview(self, full: np.ndarray, detection: dict, signal: dict) -> None:
        view = full.copy()
        rx, ry, rw, rh = self.capture.roi_rect
        cv2.rectangle(view, (rx, ry), (rx + rw - 1, ry + rh - 1), (34, 211, 238), 2)
        for item in detection.get("candidates", []):
            cx, cy = item["center"]
            color = (0, 0, 255) if detection["status"] == "JUMP_REJECTED" else (0, 165, 255)
            cv2.circle(view, (int(cx + rx), int(cy + ry)), int(item["radius"]), color, 1)
        selected = detection.get("selected")
        if selected is not None:
            cx, cy = selected["center"]
            accepted = signal["valid"] and detection.get("x_roi") is not None
            color = (0, 255, 0) if accepted else (0, 0, 255)
            point = (int(cx + rx), int(cy + ry))
            cv2.circle(view, point, int(selected.get("radius", 10)), color, 2)
            cv2.drawMarker(view, point, color, cv2.MARKER_CROSS, 16, 2)
        mapping = self.config_data["mapping"]
        span = float(mapping["position_max"] - mapping["position_min"])
        if abs(span) > 1e-6:
            fraction = (float(mapping["target_position"]) - float(mapping["position_min"])) / span
            target_x = int(rx + float(mapping["map_x_left"]) + np.clip(fraction, 0, 1) * (float(mapping["map_x_right"]) - float(mapping["map_x_left"])))
            left_x = int(rx + mapping["map_x_left"])
            right_x = int(rx + mapping["map_x_right"])
            cv2.line(view, (left_x, ry), (left_x, ry + rh - 1), (255, 180, 0), 1)
            cv2.line(view, (right_x, ry), (right_x, ry + rh - 1), (255, 180, 0), 1)
            cv2.line(view, (target_x, ry), (target_x, ry + rh - 1), (168, 85, 247), 2)
        if self._show_mask and detection.get("mask") is not None:
            mask = detection["mask"]
            if mask.shape[:2] == (rh, rw):
                area = view[ry:ry + rh, rx:rx + rw]
                overlay = np.zeros_like(area)
                overlay[:, :] = (0, 80, 0)
                active = mask > 0
                area[active] = cv2.addWeighted(area, 0.45, overlay, 0.55, 0)[active]
                view[ry:ry + rh, rx:rx + rw] = area
        if not _PIL_OK:
            return
        h, w = view.shape[:2]
        scale = min(PREVIEW_W / max(1, w), PREVIEW_H / max(1, h))
        size = (max(1, int(w * scale)), max(1, int(h * scale)))
        image = Image.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB)).resize(size, Image.BILINEAR)
        self._photo = ImageTk.PhotoImage(image)
        self.canvas.delete("all")
        self.canvas.create_image(PREVIEW_W // 2, PREVIEW_H // 2, anchor=tk.CENTER, image=self._photo)

    def _tick(self) -> None:
        roi, full, seq, ts = self.capture.latest()
        if roi is not None and seq != self._last_seq:
            self._last_seq = seq
            detection = self.detector.detect(roi, ts)
            signal = self.motion.update(detection["x_roi"], ts)
            self._frame_seq = (self._frame_seq % 0xFFFFFFFF) + 1
            frame = V5.build_csv_line(
                self._frame_seq,
                signal["valid"],
                signal["offset"],
                signal["velocity"],
                self._px_per_unit,
            )
            if self._serial_on:
                self.serial.send(frame)
            rx, ry, rw, rh = self.capture.roi_rect
            if full is None:
                full = np.zeros((max(1, ry + rh), max(1, rx + rw), 3), dtype=np.uint8)
                full[ry:ry + rh, rx:rx + rw] = roi
            self._draw_preview(full, detection, signal)
            status = V5.STATUS_ZH.get(detection["status"], detection["status"])
            self.info_lbl.config(text=(f"状态={status}  CAP={self.capture.capture_fps:.0f}fps  DET={detection['process_ms']:.1f}ms\n"
                                       f"ROI=({rx},{ry},{rw},{rh})  候选={len(detection.get('candidates', []))}  设备={self.capture.device}"))
            if signal["valid"]:
                self.signal_lbl.config(text=(f"valid=1  position={signal['position']:.2f}  offset={signal['offset']:+.2f}  velocity={signal['velocity']:+.2f}\n"
                                             f"raw_vel={signal['raw_velocity']:+.2f}  dt={signal['dt']:.3f}  TX={frame.decode('ascii').strip()}"))
            else:
                self.signal_lbl.config(text=f"valid=0  offset=0.00  velocity=0.00  TX={frame.decode('ascii').strip()}（丢球/跳变）")
        self.after(30, self._tick)

    def _quit(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass
        self.capture.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
