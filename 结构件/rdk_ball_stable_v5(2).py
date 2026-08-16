#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RDK X5 钢球 v5 数据发送版。

视觉识别沿用 v4：ROI 暗残差、几何筛选、候选评分、高速跳变门限与
边缘遮挡增强。RDK 不计算 PID 或目标杆角度，只向 STM32 发送：
  1. ROI 像素偏移 offset_px（右正左负）
  2. ROI 像素速度 speed_px_s（右移为正）

ASCII协议：
  frame_seq,offset_px,speed_px_s,valid\\n
valid=0 时 offset_px=0、speed_px_s=0，由 STM32 执行回中安全策略。

摄像头帧解码后立即裁剪 ROI；检测、映射、速度估计、串口均基于 ROI。

快捷键：
  M      显示/隐藏二值图
  S      保存 ROI、二值图和结果
  R      清空跟踪状态
  Q/ESC  退出
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

try:
    import serial as _pyserial
    _SERIAL_AVAILABLE = True
except ImportError:
    _pyserial = None
    _SERIAL_AVAILABLE = False

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # Pillow is optional; OpenCV text is the fallback.
    Image = None
    ImageDraw = None
    ImageFont = None


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "ball_stable_config(2).json"
SNAPSHOT_DIR = SCRIPT_DIR / "stable_snapshots"
WINDOW_NAME = "RDK_X5_Ball_V5_Send_Offset_Velocity"
FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
    "C:/Windows/Fonts/simsun.ttc",
)
_FONT_CACHE: dict[int, object | None] = {}
_HEADER_CACHE: np.ndarray | None = None
_HEADER_CACHE_TIME = 0.0
_HEADER_CACHE_STATUS = ""
_HEADER_CACHE_WIDTH = 0


def chinese_font(size: int) -> object | None:
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    if ImageFont is None:
        _FONT_CACHE[size] = None
        return None
    for candidate in FONT_CANDIDATES:
        if not Path(candidate).exists():
            continue
        try:
            font = ImageFont.truetype(candidate, size)
            _FONT_CACHE[size] = font
            return font
        except OSError:
            continue
    _FONT_CACHE[size] = None
    return None


def draw_chinese_lines(
    image: np.ndarray,
    lines: list[str],
    positions: list[tuple[int, int]],
    size: int = 18,
    colors: list[tuple[int, int, int]] | None = None,
) -> None:
    """Draw UTF-8 text when Pillow and a CJK font are available."""
    font = chinese_font(size)
    if font is None or Image is None or ImageDraw is None:
        for index, text in enumerate(lines):
            safe = text.encode("ascii", "replace").decode("ascii")
            color = (
                colors[index]
                if colors is not None and index < len(colors)
                else (255, 255, 255)
            )
            cv2.putText(
                image,
                safe,
                (positions[index][0], positions[index][1] + size),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                1,
                cv2.LINE_AA,
            )
        return
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    canvas = Image.fromarray(rgb)
    drawer = ImageDraw.Draw(canvas)
    for index, text in enumerate(lines):
        color_bgr = (
            colors[index]
            if colors is not None and index < len(colors)
            else (255, 255, 255)
        )
        color_rgb = tuple(reversed(color_bgr))
        drawer.text(
            positions[index],
            text,
            font=font,
            fill=color_rgb,
            stroke_width=1,
            stroke_fill=(0, 0, 0),
        )
    image[:] = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


STATUS_ZH = {
    "TRACKING": "跟踪中",
    "LOST": "丢失",
    "JUMP_REJECTED": "跳变拒绝",
}


class SerialComm:
    """USART1 ASCII视觉帧发送器，fire-and-forget。"""

    def __init__(self, config: dict) -> None:
        self.enabled = bool(config.get("enabled", False))
        self.port = str(config.get("port", "/dev/ttyUSB0"))
        self.baudrate = int(config.get("baudrate", 115200))
        self.ser = None

    def open(self) -> bool:
        if not self.enabled:
            print("[串口] 串口未启用（enabled=false）", flush=True)
            return True
        if not _SERIAL_AVAILABLE:
            print("[串口] pyserial 未安装，串口不可用；请执行：pip3 install pyserial", flush=True)
            return False
        try:
            self.ser = _pyserial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=_pyserial.EIGHTBITS,
                stopbits=_pyserial.STOPBITS_ONE,
                parity=_pyserial.PARITY_NONE,
                timeout=0.1,
            )
            print(f"[串口] 已打开 {self.port} @ {self.baudrate}", flush=True)
            return True
        except Exception as exc:
            print(f"[串口] 打开失败: {exc}", flush=True)
            self.ser = None
            return False

    def send(self, frame: bytearray) -> None:
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(frame)
        except Exception as exc:
            print(f"[串口] 发送失败: {exc}", flush=True)

    def close(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("[串口] 已关闭", flush=True)


def _clamp_int(value: float, minimum: int, maximum: int) -> int:
    """将有限浮点量四舍五入为整数并限幅到 STM32 协议范围。"""
    if not math.isfinite(value):
        return 0
    return max(minimum, min(maximum, int(round(value))))


def build_frame(frame_seq: int,
                valid: bool,
                offset_px: float,
                speed_px_s: float,
                offset_limit_px: int = 200,
                speed_limit_px_s: int = 20000) -> bytes:
    """构造 STM32 科目二 ASCII 帧：frame_seq,offset_px,speed_px_s,valid\\n。"""
    sequence = int(frame_seq) & 0xFFFFFFFF
    if not valid:
        offset = 0
        speed = 0
        valid_int = 0
    else:
        offset = _clamp_int(offset_px, -abs(int(offset_limit_px)), abs(int(offset_limit_px)))
        speed = _clamp_int(speed_px_s, -abs(int(speed_limit_px_s)), abs(int(speed_limit_px_s)))
        valid_int = 1
    return f"{sequence},{offset},{speed},{valid_int}\\n".encode("ascii")


def built_in_config() -> dict:
    return {
        "device": "/dev/v4l/by-id/usb-Generic_USB_Camera_20000001-video-index0",
        "camera": {
            "width": 640,
            "height": 480,
            "requested_fps": 100,
            "mjpg": 1,
            "buffers": 1,
            "auto_exposure": 1,
            "exposure": 50,
            "gain": 96,
            "gamma": 300,
            "brightness": 0,
            "contrast": 40,
            "saturation": 64,
            "sharpness": 50,
            "backlight": 0,
            "auto_white_balance": 1,
        },
        "roi": {"x": 5, "y": 220, "width": 630, "height": 65},
        "binary": {
            "background_kernel": 31,
            "threshold": 18,
            "open_kernel": 3,
            "open_iterations": 1,
            "close_kernel": 5,
            "close_iterations": 1,
        },
        "geometry": {
            "min_radius": 7.0,
            "max_radius": 16.0,
            "expected_radius": 10.5,
            "min_area": 100.0,
            "max_area": 600.0,
            "min_circularity": 0.65,
            "max_aspect": 1.60,
        },
        "tracking": {
            "max_jump_px": 95.0,
            "position_alpha": 0.62,
            "velocity_alpha": 0.35,
            "motion_weight": 6.0,
            "lost_hold_frames": 5,
        },
        "v5": {
            "edge_margin_px": 12.0,
            "edge_confidence_boost": 1.2,
            "jump_speed_gain": 0.045,
            "coast_frames": 5,
            "min_circularity_edge": 0.50,
            "min_area_edge": 65.0,
        },
        "mapping": {
            "map_x_left": 0.0,
            "map_x_right": 629.0,
            "position_min": 0.0,
            "position_max": 448.0,
            "target_position": 224.0,
        },
        "signal": {
            "velocity_alpha": 0.35,
            "velocity_deadband_unit_s": 0.0,
            "max_dt_s": 0.20,
            "velocity_limit_unit_s": 10000.0,
        },
        "runtime": {
            "display": 1,
            "show_mask": 0,
            "display_width": 800,
            "print_every_frames": 5,
        },
        "serial": {
            "enabled": 1,
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
        },
    }


def deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: Path) -> dict:
    config = built_in_config()
    if path.exists():
        config = deep_merge(config, json.loads(path.read_text(encoding="utf-8")))
    return config


def odd(value: int) -> int:
    value = max(1, int(value))
    return value if value % 2 else value + 1


def clamp_rect(rect: dict, frame_width: int, frame_height: int) -> tuple[int, int, int, int]:
    x = max(0, min(int(rect["x"]), max(0, frame_width - 1)))
    y = max(0, min(int(rect["y"]), max(0, frame_height - 1)))
    width = max(1, min(int(rect["width"]), frame_width - x))
    height = max(1, min(int(rect["height"]), frame_height - y))
    return x, y, width, height


def capture_device_candidates(requested: str) -> list[str]:
    if requested != "auto":
        return [requested]
    preferred: list[str] = []
    fallback: list[str] = []
    for candidate in sorted(Path("/dev").glob("video*")):
        name = str(candidate)
        fallback.append(name)
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", name, "--all"],
                check=False,
                capture_output=True,
                text=True,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        details = (result.stdout or "") + (result.stderr or "")
        if (
            result.returncode == 0
            and "Video Capture" in details
            and "Format Video Capture" in details
        ):
            preferred.append(name)
    ordered = preferred + [item for item in fallback if item not in preferred]
    return ordered or ["auto"]


class CameraControls:
    ALIASES = {
        "auto_exposure": ("auto_exposure", "exposure_auto"),
        "exposure": ("exposure_time_absolute", "exposure_absolute"),
        "gain": ("gain",),
        "gamma": ("gamma",),
        "brightness": ("brightness",),
        "contrast": ("contrast",),
        "saturation": ("saturation",),
        "sharpness": ("sharpness",),
        "backlight": ("backlight_compensation",),
        "auto_white_balance": (
            "white_balance_automatic",
            "white_balance_temperature_auto",
        ),
    }
    LABELS_ZH = {
        "auto_exposure": "自动曝光",
        "exposure": "曝光",
        "gain": "增益",
        "gamma": "伽马",
        "brightness": "亮度",
        "contrast": "对比度",
        "saturation": "饱和度",
        "sharpness": "锐度",
        "backlight": "背光补偿",
        "auto_white_balance": "自动白平衡",
    }

    def __init__(self, device: str) -> None:
        self.device = device
        self.available: set[str] = set()

    def discover(self) -> None:
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", self.device, "-L"],
                check=False,
                capture_output=True,
                text=True,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired):
            return
        output = (result.stdout or "") + (result.stderr or "")
        self.available = set(
            re.findall(r"^\s*([A-Za-z0-9_]+)\s+0x", output, re.MULTILINE)
        )

    def set_one(self, logical: str, value: int) -> bool:
        actual = next(
            (name for name in self.ALIASES[logical] if name in self.available),
            None,
        )
        if actual is None:
            return False
        try:
            result = subprocess.run(
                [
                    "v4l2-ctl",
                    "-d",
                    self.device,
                    f"--set-ctrl={actual}={int(value)}",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def apply(self, camera: dict) -> list[str]:
        self.discover()
        values = [
            ("auto_exposure", int(camera.get("auto_exposure", 1))),
            ("exposure", int(camera.get("exposure", 50))),
            ("gain", int(camera.get("gain", 96))),
            ("gamma", int(camera.get("gamma", 300))),
            ("brightness", int(camera.get("brightness", 0))),
            ("contrast", int(camera.get("contrast", 40))),
            ("saturation", int(camera.get("saturation", 64))),
            ("sharpness", int(camera.get("sharpness", 50))),
            ("backlight", int(camera.get("backlight", 0))),
            ("auto_white_balance", int(camera.get("auto_white_balance", 1))),
        ]
        applied = [
            f"{self.LABELS_ZH.get(name, name)}={value}"
            for name, value in values
            if self.set_one(name, value)
        ]
        print(
            "[相机参数] " + (", ".join(applied) if applied else "没有应用任何参数"),
            flush=True,
        )
        return applied


class LatestRoiCapture:
    """Capture full frames, immediately crop, and retain only the latest ROI."""

    def __init__(self, config: dict, retain_full_frame: bool = False) -> None:
        self.config = config
        self.requested_device = config["device"]
        self.device = config["device"]
        self.retain_full_frame = bool(retain_full_frame)
        self.cap: cv2.VideoCapture | None = None
        self.roi_rect = (0, 0, 1, 1)
        self.actual_size = (0, 0)
        self.lock = threading.Lock()
        self.roi_frame: np.ndarray | None = None
        self.full_frame: np.ndarray | None = None
        self.timestamp = 0.0
        self.sequence = 0
        self.running = False
        self.capture_times: deque[float] = deque(maxlen=90)
        self.capture_fps = 0.0
        self.thread: threading.Thread | None = None

    def open(self) -> bool:
        camera = self.config["camera"]
        for candidate in capture_device_candidates(self.requested_device):
            CameraControls(candidate).apply(camera)
            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                print(f"[相机] 跳过不可用设备：{candidate}", flush=True)
                continue
            self.device = candidate
            self.cap = cap
            break
        if self.cap is None:
            print(
                "[错误] 没有可用摄像头，请关闭其他占用摄像头的程序，"
                "再检查 v4l2-ctl --list-devices",
                flush=True,
            )
            return False
        if camera["mjpg"]:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera["width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera["height"])
        self.cap.set(cv2.CAP_PROP_FPS, camera["requested_fps"])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, camera["buffers"])
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or int(camera.get("width", 640))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or int(camera.get("height", 480))
        self.actual_size = (actual_width, actual_height)
        self.roi_rect = clamp_rect(self.config["roi"], actual_width, actual_height)
        print(
            f"[相机] 设备={self.device} 实际={actual_width}x{actual_height} "
            f"驱动帧率={self.cap.get(cv2.CAP_PROP_FPS):.1f} ROI={self.roi_rect}",
            flush=True,
        )
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def _loop(self) -> None:
        assert self.cap is not None
        while self.running:
            ok, full_frame = self.cap.read()
            captured_at = time.perf_counter()
            if not ok or full_frame is None:
                time.sleep(0.002)
                continue
            with self.lock:
                rx, ry, rw, rh = self.roi_rect
            frame_h, frame_w = full_frame.shape[:2]
            rx, ry, rw, rh = clamp_rect(
                {"x": rx, "y": ry, "width": rw, "height": rh},
                frame_w,
                frame_h,
            )
            roi = full_frame[ry : ry + rh, rx : rx + rw].copy()
            self.capture_times.append(captured_at)
            if len(self.capture_times) > 1:
                elapsed = self.capture_times[-1] - self.capture_times[0]
                self.capture_fps = (
                    (len(self.capture_times) - 1) / elapsed if elapsed > 0 else 0.0
                )
            with self.lock:
                self.roi_rect = (rx, ry, rw, rh)
                self.roi_frame = roi
                self.full_frame = full_frame.copy() if self.retain_full_frame else None
                self.timestamp = captured_at
                self.sequence += 1

    def set_roi(self, rect: tuple[int, int, int, int]) -> None:
        with self.lock:
            width, height = self.actual_size
            if width > 0 and height > 0:
                self.roi_rect = clamp_rect(
                    {"x": rect[0], "y": rect[1], "width": rect[2], "height": rect[3]},
                    width,
                    height,
                )
            else:
                self.roi_rect = tuple(int(value) for value in rect)
            self.roi_frame = None
            self.full_frame = None
            self.sequence += 1

    def latest(self):
        with self.lock:
            roi = None if self.roi_frame is None else self.roi_frame.copy()
            full = None if self.full_frame is None else self.full_frame.copy()
            return roi, full, self.sequence, self.timestamp

    def close(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()


class RoiBallDetector:
    """v5：沿用 v4 检测前端，输出当前帧可靠钢球位置。"""

    def __init__(self, config: dict) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.x: float | None = None
        self.y: float | None = None
        self.velocity_px_s = 0.0
        self.last_time: float | None = None
        self.lost_frames = 0

    def detect(self, roi: np.ndarray, timestamp: float) -> dict:
        started = time.perf_counter()
        binary = self.config["binary"]
        geometry = self.config["geometry"]
        tracking = self.config["tracking"]
        v5 = self.config.get("v5", {})
        edge_margin = float(v5.get("edge_margin_px", 12.0))
        edge_boost = float(v5.get("edge_confidence_boost", 1.2))
        jump_speed_gain = float(v5.get("jump_speed_gain", 0.045))
        coast_frames = int(v5.get("coast_frames", tracking.get("lost_hold_frames", 3)))
        min_circ_edge = float(v5.get("min_circularity_edge", 0.50))
        min_area_edge = float(v5.get("min_area_edge", max(60.0, geometry["min_area"] * 0.65)))

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        background_kernel = odd(binary["background_kernel"])
        broad = cv2.boxFilter(
            gray,
            -1,
            (background_kernel, background_kernel),
            normalize=True,
        )
        dark_residual = cv2.subtract(broad, gray)
        mask = (dark_residual >= binary["threshold"]).astype(np.uint8) * 255
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (odd(binary["open_kernel"]),) * 2
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (odd(binary["close_kernel"]),) * 2
        )
        if binary["open_iterations"]:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                open_kernel,
                iterations=binary["open_iterations"],
            )
        if binary["close_iterations"]:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                close_kernel,
                iterations=binary["close_iterations"],
            )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 用上一帧速度预测当前应在位置（高速选球更稳）
        dt_pred = 0.0
        if self.last_time is not None:
            dt_pred = max(0.0, timestamp - self.last_time)
        predicted_x = None
        if self.x is not None:
            predicted_x = self.x + self.velocity_px_s * dt_pred
            predicted_x = float(np.clip(predicted_x, 0.0, max(0.0, roi.shape[1] - 1)))

        ref_x = predicted_x if predicted_x is not None else self.x
        roi_w = max(1.0, float(roi.shape[1]))
        roi_h = max(1.0, float(roi.shape[0]))

        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 1e-5:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            bx, by, bw, bh = cv2.boundingRect(contour)
            # 障碍物位于轨道左右端，边缘放宽只看 x；ROI 很矮时若把上下边缘
            # 也算进去，几乎所有候选都会被错误地当成边缘候选。
            near_edge = (
                bx <= edge_margin
                or (bx + bw) >= (roi_w - edge_margin)
            )
            min_area = min_area_edge if near_edge else geometry["min_area"]
            min_circ = min_circ_edge if near_edge else geometry["min_circularity"]
            if area < min_area or area > geometry["max_area"]:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            aspect = max(bw, bh) / max(1.0, min(bw, bh))
            # 边缘允许略扁（部分遮挡）
            max_aspect = geometry["max_aspect"] * (1.25 if near_edge else 1.0)
            if (
                radius < geometry["min_radius"] * (0.85 if near_edge else 1.0)
                or radius > geometry["max_radius"] * (1.15 if near_edge else 1.0)
                or circularity < min_circ
                or aspect > max_aspect
            ):
                continue
            contour_mask = np.zeros_like(mask)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            pixels = dark_residual[contour_mask > 0]
            darkness = float(pixels.mean()) if pixels.size else 0.0
            expected = max(0.1, geometry["expected_radius"])
            radius_score = max(0.0, 1.0 - abs(radius - expected) / expected)
            confidence = 4.0 * circularity + darkness / 8.0 + 2.0 * radius_score
            if near_edge:
                confidence += edge_boost
            distance_penalty = (
                0.0 if ref_x is None else abs(cx - ref_x) / roi_w
            )
            score = confidence - tracking["motion_weight"] * distance_penalty
            candidates.append(
                {
                    "center": (float(cx), float(cy)),
                    "radius": float(radius),
                    "area": area,
                    "circularity": circularity,
                    "aspect": aspect,
                    "confidence": confidence,
                    "score": score,
                    "near_edge": near_edge,
                }
            )
        selected = max(candidates, key=lambda item: item["score"]) if candidates else None
        status = "LOST"
        control_x: float | None = None
        control_y: float | None = None
        if selected is not None:
            detected_x, detected_y = selected["center"]
            # 跳变门限设上限，避免一次速度尖峰把门限永久放得过宽。
            base_jump = float(tracking["max_jump_px"])
            jump_limit = base_jump + jump_speed_gain * abs(self.velocity_px_s)
            jump_limit = min(jump_limit, base_jump * 2.0)
            if self.x is not None and abs(detected_x - self.x) > jump_limit:
                status = "JUMP_REJECTED"
                self.lost_frames += 1
            else:
                previous_x = self.x
                previous_time = self.last_time
                self.x = float(detected_x)
                self.y = float(detected_y)
                if previous_x is not None and previous_time is not None:
                    dt = timestamp - previous_time
                    if 1e-4 < dt <= 0.2:
                        self.velocity_px_s = (self.x - previous_x) / dt
                    else:
                        self.velocity_px_s = 0.0
                self.last_time = timestamp
                self.lost_frames = 0
                status = "TRACKING"
                control_x = self.x
                control_y = self.y
        else:
            self.lost_frames += 1

        # 被拒绝或丢检时内部可短暂保留上次位置用于下一帧重新选球，但绝不能
        # 把旧位置作为本帧有效数据发送给 STM32。
        if self.lost_frames > coast_frames:
            self.x = None
            self.y = None
            self.velocity_px_s = 0.0
            self.last_time = None
        return {
            "status": status,
            "x_roi": control_x,
            "y_roi": control_y,
            "tracked_x_roi": self.x,
            "tracked_y_roi": self.y,
            "velocity_px_s": self.velocity_px_s,
            "mask": mask,
            "candidates": candidates,
            "selected": selected,
            "process_ms": (time.perf_counter() - started) * 1000.0,
        }


class MotionEstimator:
    """把可靠的 ROI 横坐标映射为位置偏移量和轻量 EMA 速度。"""

    def __init__(self, config: dict) -> None:
        self.mapping = config["mapping"]
        signal = config.get("signal", {})
        self.velocity_alpha = float(
            signal.get("velocity_alpha", config["tracking"].get("velocity_alpha", 0.35))
        )
        self.velocity_deadband = max(
            0.0, float(signal.get("velocity_deadband_unit_s", 0.0))
        )
        self.max_dt = max(0.01, float(signal.get("max_dt_s", 0.20)))
        self.velocity_limit = max(
            0.0,
            float(signal.get("speed_limit_px_s",
                           signal.get("velocity_limit_unit_s", 20000.0)))
        )
        self.last_position: float | None = None
        self.last_position_px: float | None = None
        self.last_time: float | None = None
        self.velocity = 0.0
        self.target_x_px = self._target_x_px()

    def reset(self) -> None:
        self.last_position = None
        self.last_position_px = None
        self.last_time = None
        self.velocity = 0.0

    def _target_x_px(self) -> float:
        mapping = self.mapping
        position_span = float(mapping["position_max"]) - float(mapping["position_min"])
        if abs(position_span) < 1e-6:
            return (float(mapping["map_x_left"]) + float(mapping["map_x_right"])) * 0.5
        fraction = (float(mapping["target_position"]) - float(mapping["position_min"])) / position_span
        fraction = float(np.clip(fraction, 0.0, 1.0))
        return float(mapping["map_x_left"]) + fraction * (
            float(mapping["map_x_right"]) - float(mapping["map_x_left"])
        )

    def map_position(self, x_roi: float) -> float:
        left = float(self.mapping["map_x_left"])
        right = float(self.mapping["map_x_right"])
        fraction = (float(x_roi) - left) / max(1e-6, right - left)
        fraction = float(np.clip(fraction, 0.0, 1.0))
        return float(
            self.mapping["position_min"]
            + fraction
            * (self.mapping["position_max"] - self.mapping["position_min"])
        )

    def update(self, x_roi: float | None, timestamp: float) -> dict:
        if x_roi is None:
            self.reset()
            return {
                "valid": False,
                "position": None,
                "offset": 0.0,
                "velocity": 0.0,
                "raw_velocity": 0.0,
                "position_px": None,
                "target_x_px": self.target_x_px,
                "offset_px": 0.0,
                "speed_px_s": 0.0,
                "raw_speed_px_s": 0.0,
                "dt": 0.0,
            }

        position = self.map_position(x_roi)
        position_px = float(x_roi)
        raw_velocity_px_s = 0.0
        dt = 0.0
        if self.last_position_px is not None and self.last_time is not None:
            measured_dt = float(timestamp - self.last_time)
            if 1e-4 < measured_dt <= self.max_dt:
                dt = measured_dt
                raw_velocity_px_s = (position_px - self.last_position_px) / measured_dt
                if self.velocity_limit > 0.0:
                    raw_velocity_px_s = float(
                        np.clip(raw_velocity_px_s, -self.velocity_limit, self.velocity_limit)
                    )
                alpha = float(np.clip(self.velocity_alpha, 0.0, 1.0))
                self.velocity = (
                    alpha * raw_velocity_px_s + (1.0 - alpha) * self.velocity
                )
            else:
                self.velocity = 0.0
        else:
            self.velocity = 0.0

        self.last_position = position
        self.last_position_px = position_px
        self.last_time = float(timestamp)
        if abs(self.velocity) < self.velocity_deadband:
            self.velocity = 0.0
        offset_px = position_px - self.target_x_px
        return {
            "valid": True,
            "position": position,
            "offset": offset_px,
            "velocity": float(self.velocity),
            "raw_velocity": float(raw_velocity_px_s),
            "position_px": position_px,
            "target_x_px": self.target_x_px,
            "offset_px": offset_px,
            "speed_px_s": float(self.velocity),
            "raw_speed_px_s": float(raw_velocity_px_s),
            "dt": dt,
        }


def draw_roi(
    roi: np.ndarray,
    detection: dict,
    signal: dict,
    capture_fps: float,
    config: dict,
) -> np.ndarray:
    global _HEADER_CACHE
    global _HEADER_CACHE_TIME
    global _HEADER_CACHE_STATUS
    global _HEADER_CACHE_WIDTH
    output = roi.copy()
    mapping = config["mapping"]
    left = int(
        round(
            np.clip(
                mapping["map_x_left"],
                0,
                max(0, output.shape[1] - 1),
            )
        )
    )
    right = int(
        round(
            np.clip(
                mapping["map_x_right"],
                0,
                max(0, output.shape[1] - 1),
            )
        )
    )
    position_span = max(
        1e-6,
        mapping["position_max"] - mapping["position_min"],
    )
    target_fraction = (
        mapping["target_position"] - mapping["position_min"]
    ) / position_span
    target_x = int(
        round(
            np.clip(
                left + np.clip(target_fraction, 0.0, 1.0) * (right - left),
                0,
                max(0, output.shape[1] - 1),
            )
        )
    )
    cv2.line(output, (left, 0), (left, output.shape[0] - 1), (255, 180, 0), 1)
    cv2.line(output, (right, 0), (right, output.shape[0] - 1), (255, 180, 0), 1)
    cv2.line(
        output,
        (target_x, 0),
        (target_x, output.shape[0] - 1),
        (255, 0, 255),
        2,
    )
    for item in detection["candidates"]:
        cx, cy = item["center"]
        cv2.circle(
            output,
            (int(round(cx)), int(round(cy))),
            int(round(item["radius"])),
            (0, 180, 255),
            1,
        )
    # 用 selected（当前帧最佳候选）直接画绿色框，不用滤波后的 x_roi/y_roi
    selected = detection.get("selected")
    if selected is not None:
        cx, cy = selected["center"]
        r = int(round(selected["radius"]))
        cv2.circle(output, (int(round(cx)), int(round(cy))), r, (0, 255, 0), 2)
        cv2.drawMarker(output, (int(round(cx)), int(round(cy))), (0, 255, 0), cv2.MARKER_CROSS, 13, 2)
    lines = [
        (
            f"状态={STATUS_ZH.get(detection['status'], detection['status'])} "
            f"采集={capture_fps:.1f}帧/秒 "
            f"处理={detection['process_ms']:.2f}毫秒"
        ),
        (
            f"ROI横坐标={detection['x_roi']:.2f} "
            f"映射位置={signal['position']:.2f} "
            f"偏移={signal['offset']:+.2f} "
            f"速度={signal['velocity']:+.2f}/秒"
            if signal["valid"]
            else "ROI横坐标=丢失 映射位置=-- 偏移=-- 速度=--"
        ),
        (
            f"发送：valid=1 offset={signal['offset']:+.2f} "
            f"velocity={signal['velocity']:+.2f}"
            if signal["valid"]
            else "发送：valid=0 offset=0 velocity=0"
        ),
    ]
    display_width = max(
        output.shape[1],
        int(config["runtime"]["display_width"]),
    )
    scale = display_width / max(1, output.shape[1])
    shown = cv2.resize(
        output,
        (display_width, max(1, round(output.shape[0] * scale))),
        interpolation=cv2.INTER_NEAREST,
    )
    header_height = 74
    canvas = np.zeros(
        (header_height + shown.shape[0], display_width, 3),
        dtype=np.uint8,
    )
    canvas[header_height:, :] = shown
    now = time.monotonic()
    if (
        _HEADER_CACHE is None
        or now - _HEADER_CACHE_TIME >= 0.25
        or _HEADER_CACHE_STATUS != detection["status"]
        or _HEADER_CACHE_WIDTH != display_width
    ):
        header = np.zeros((header_height, display_width, 3), dtype=np.uint8)
        draw_chinese_lines(
            header,
            lines,
            [(8, 8 + index * 22) for index in range(len(lines))],
            size=18,
            colors=[
                (0, 255, 255),
                (255, 255, 255),
                (255, 255, 255),
            ],
        )
        _HEADER_CACHE = header
        _HEADER_CACHE_TIME = now
        _HEADER_CACHE_STATUS = str(detection["status"])
        _HEADER_CACHE_WIDTH = display_width
    canvas[:header_height, :] = _HEADER_CACHE
    return canvas


def save_snapshot(roi: np.ndarray, annotated: np.ndarray, detection: dict, signal: dict) -> None:
    SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
    stem = SNAPSHOT_DIR / time.strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(str(stem) + "_roi.jpg", roi)
    cv2.imwrite(str(stem) + "_annotated.jpg", annotated)
    cv2.imwrite(str(stem) + "_mask.png", cv2.bitwise_not(detection["mask"]))
    Path(str(stem) + "_result.json").write_text(
        json.dumps(
            {
                "detection": {
                    key: detection[key]
                    for key in ("status", "x_roi", "y_roi", "velocity_px_s", "process_ms")
                },
                "signal": signal,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"[截图] 已保存 ROI、二值图和结果：{stem}_*", flush=True)


def main() -> int:
    if chinese_font(18) is None:
        print(
            "[中文字体警告] 未找到 Pillow 或中文字体，画面文字会回退为英文问号。"
            "请安装：sudo apt install python3-pil fonts-wqy-zenhei",
            flush=True,
        )
    parser = argparse.ArgumentParser(description="RDK X5 钢球 v5 发送位置偏移量与速度")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.headless:
        config["runtime"]["display"] = 0
    serial_comm = SerialComm(config.get("serial", {}))
    if not serial_comm.open():
        print("[错误] v5 串口打开失败，停止运行，避免视觉正常但数据未发送", flush=True)
        return 3
    capture = LatestRoiCapture(config)
    if not capture.open():
        return 2
    detector = RoiBallDetector(config)
    motion = MotionEstimator(config)
    display = bool(config["runtime"]["display"])
    show_mask = bool(config["runtime"]["show_mask"])
    last_sequence = -1
    tx_frame_seq = 0
    frame_index = 0
    annotated: np.ndarray | None = None
    last_roi: np.ndarray | None = None
    last_detection: dict | None = None
    last_signal: dict | None = None
    if display:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 800, 480)
        cv2.moveWindow(WINDOW_NAME, 0, 0)
    try:
        while True:
            roi, _full, sequence, timestamp = capture.latest()
            if roi is None or sequence == last_sequence:
                if display:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q"), ord("Q")):
                        break
                else:
                    time.sleep(0.001)
                continue
            last_sequence = sequence
            frame_index += 1
            detection = detector.detect(roi, timestamp)
            signal = motion.update(detection["x_roi"], timestamp)
            # 只有当前帧可靠跟踪才发送有效 offset/velocity；LOST/JUMP 清零。
            tx_frame = build_frame(
                tx_frame_seq,
                signal["valid"],
                signal.get("offset_px", signal["offset"]),
                signal.get("speed_px_s", signal["velocity"]),
                int(config.get("signal", {}).get("offset_limit_px", 200)),
                int(config.get("signal", {}).get("speed_limit_px_s", 20000)),
            )
            serial_comm.send(tx_frame)
            tx_frame_seq = (tx_frame_seq + 1) & 0xFFFFFFFF
            annotated = draw_roi(
                roi,
                detection,
                signal,
                capture.capture_fps,
                config,
            )
            last_roi = roi
            last_detection = detection
            last_signal = signal
            if frame_index % max(1, config["runtime"]["print_every_frames"]) == 0:
                print(
                    json.dumps(
                        {
                            "valid": signal["valid"],
                            "status_zh": STATUS_ZH.get(
                                detection["status"],
                                detection["status"],
                            ),
                            "x_roi": detection["x_roi"],
                            "position": signal.get("position"),
                            "offset_px": signal.get("offset_px", signal["offset"]),
                            "speed_px_s": signal.get("speed_px_s", signal["velocity"]),
                            "tx_frame": tx_frame.decode("ascii").rstrip("\n"),
                            "raw_velocity": signal["raw_velocity"],
                            "dt": signal["dt"],
                            "capture_fps": capture.capture_fps,
                            "process_ms": detection["process_ms"],
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
            if display:
                panel = annotated
                if show_mask:
                    mask_panel = cv2.cvtColor(
                        cv2.bitwise_not(detection["mask"]), cv2.COLOR_GRAY2BGR
                    )
                    mask_panel = cv2.resize(
                        mask_panel,
                        (
                            annotated.shape[1],
                            max(
                                1,
                                round(
                                    mask_panel.shape[0]
                                    * annotated.shape[1]
                                    / mask_panel.shape[1]
                                ),
                            ),
                        ),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    panel = np.vstack((annotated, mask_panel))
                cv2.imshow(WINDOW_NAME, panel)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("m"), ord("M")):
                    show_mask = not show_mask
                if key in (ord("r"), ord("R")):
                    detector.reset()
                    motion.reset()
                if (
                    key in (ord("s"), ord("S"))
                    and last_roi is not None
                    and annotated is not None
                    and last_detection is not None
                    and last_signal is not None
                ):
                    save_snapshot(
                        last_roi, annotated, last_detection, last_signal
                    )
    finally:
        serial_comm.close()
        capture.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
